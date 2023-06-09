// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "HBAOHelper2.h"
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/PixelFormat.h>

#include <random>

// #include "src/shaders/ssao/common.h" // can't access this from here

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {
namespace ssao {

namespace {

// sloppy: duplicate these from common.h
#define AO_RANDOMTEX_SIZE 4
static const int MAX_SAMPLES = 8;
static const int HBAO_RANDOM_SIZE = AO_RANDOMTEX_SIZE;
static const int HBAO_RANDOM_ELEMENTS = HBAO_RANDOM_SIZE * HBAO_RANDOM_SIZE;

struct SceneData {
  Mn::Matrix4 viewProjMatrix;
  Mn::Matrix4 viewMatrix;
  Mn::Matrix4 viewMatrixIT;

  Mn::Vector2ui viewport;
  Mn::Vector2ui _pad;
};

struct HBAOData {
  float RadiusToScreen;  // radius
  float R2;              // 1/radius
  float NegInvR2;        // radius * radius
  float NDotVBias;

  Mn::Vector2 InvFullResolution;
  Mn::Vector2 InvQuarterResolution;

  float AOMultiplier;
  float PowExponent;
  Mn::Vector2 _pad0;

  Mn::Vector4 projInfo;
  Mn::Vector2 projScale;
  int projOrtho;
  int _pad1;

  Mn::Vector4 float2Offsets[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
  Mn::Vector4 jitters[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
};

// class BilateralBlurShader: Mn::GL::AbstractShaderProgram {
// public:
//   BilateralBlurShader() {
//     Cr::Utility::Resource rs{"default-shaders"};

//     Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
//     Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

//     vert.addSource(rs.getString("ssao/common.h"));
//     vert.addSource(rs.getString("ssao/fullscreenquad.vert.glsl"));
//     frag.addSource(rs.getString("ssao/bilateralblur.frag.glsl"));

//     CORRADE_INTERNAL_ASSERT(vert.compile() && frag.compile());

//     attachShaders({vert, frag});
//     CORRADE_INTERNAL_ASSERT(link());
//   }
// };

class HBAOShader : public Mn::GL::AbstractShaderProgram {
 public:
  using Mn::GL::AbstractShaderProgram::setUniform;

  HBAOShader() = default;
  HBAOShader(const std::vector<Cr::Containers::StringView>& vertSources,
             const std::vector<Cr::Containers::StringView>& fragSources,
             const std::vector<Cr::Containers::StringView>& geomSources = {}) {
    Cr::Utility::Resource rs{"default-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
    // HBAO is untested for WebGL
    constexpr Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
    constexpr Mn::GL::Version glVersion = Mn::GL::Version::GL430;
#endif

    Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
    Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

    for (const auto& vertSource : vertSources) {
      vert.addSource(rs.getString(vertSource));
    }
    for (const auto& fragSource : fragSources) {
      // sloppy: allow both filenames and code snippets that start with #define
      frag.addSource((fragSource[0] == '#') ? fragSource
                                            : rs.getString(fragSource));
    }

    CORRADE_INTERNAL_ASSERT(vert.compile() && frag.compile());
    attachShaders({vert, frag});

    if (geomSources.size()) {
      Mn::GL::Shader geom{glVersion, Mn::GL::Shader::Type::Geometry};
      for (const auto& geomSource : geomSources) {
        geom.addSource(rs.getString(geomSource));
      }
      CORRADE_INTERNAL_ASSERT(geom.compile());
      attachShader(geom);
    }

    CORRADE_INTERNAL_ASSERT(link());
  }
};

}  // namespace

// optimizes blur, by storing depth along with ssao calculation
// avoids accessing two different textures
#define USE_AO_SPECIALBLUR 1

#define AO_LAYERED_OFF 0
#define AO_LAYERED_IMAGE 1
#define AO_LAYERED_GS 2

#define USE_AO_LAYERED_SINGLEPASS AO_LAYERED_OFF

static const int NUM_MRT = 8;

struct HBAOHelper2::Impl {
  struct Config {
    int samples = 1;
    float intensity = 0.732f;  // hand-tuned
    float bias = 0.05f;
    float radius = 1.24f;
    float blurSharpness = 10.0f;
    bool blur = true;  // true;
  };

  Impl() {
#ifdef MAGNUM_TARGET_WEBGL
    ESP_CHECK(false, "HBAO is untested for WebGL");
#endif
  }

  void init(Mn::GL::Texture2D* depthRenderTexture,
            Mn::GL::Framebuffer* framebuffer) {
    depthRenderTexture_ = depthRenderTexture;
    framebuffer_ = framebuffer;

    initShaders();
    initMisc();
    constexpr int samples = 1;
    const auto viewportRange = framebuffer->viewport();
    initFramebuffers(viewportRange.sizeX(), viewportRange.sizeY(), samples);

    // note actual vertex positions are calculated inside a vertex shader
    // todo: rename to what this mesh actually is (dummyTriangle)
    fullScreenTriangle_.setCount(3);
  }

  void initShaders() {
    // todo: don't create all variants; just the ones needed based on config

    // programs.bilateralblur = m_progManager.createProgram(
    //     nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                     "fullscreenquad.vert.glsl"),
    //     nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                     "bilateralblur.frag.glsl"));

    programs.bilateralblur =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"ssao/common.h", "ssao/bilateralblur.frag.glsl"});

    // programs.depth_linearize = m_progManager.createProgram(
    //     nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                     "fullscreenquad.vert.glsl"),
    //     nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                     "#define DEPTHLINEARIZE_MSAA 0\n",
    //                                     "depthlinearize.frag.glsl"));

    programs.depth_linearize = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {"#define DEPTHLINEARIZE_MSAA 0\n", "ssao/depthlinearize.frag.glsl"});

    //   programs.depth_linearize_msaa = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "#define DEPTHLINEARIZE_MSAA 1\n",
    //                                       "depthlinearize.frag.glsl"));

    programs.depth_linearize_msaa = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {"#define DEPTHLINEARIZE_MSAA 1\n", "ssao/depthlinearize.frag.glsl"});

    //   programs.viewnormal = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "viewnormal.frag.glsl"));

    programs.viewnormal = HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                                     {"ssao/viewnormal.frag.glsl"});

    //   programs.displaytex = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "displaytex.frag.glsl"));

    programs.displaytex = HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                                     {"ssao/displaytex.frag.glsl"});

    // m_progManager.m_prepend = nvgl::ProgramManager::format(
    //     "#define AO_LAYERED %d\n", USE_AO_LAYERED_SINGLEPASS);
    const std::string aoLayeredPrepend =
        "#define AO_LAYERED " + std::to_string(USE_AO_LAYERED_SINGLEPASS) +
        "\n";

    //   programs.hbao_calc = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(
    //           GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 0\n#define
    //           AO_BLUR 0\n", "hbao.frag.glsl"));

    // note common.h must be explicitly included here; Magnum doesn't support
    // #includes inside glsl.
    programs.hbao_calc = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {aoLayeredPrepend, "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 0\n",
         "ssao/common.h", "ssao/hbao.frag.glsl"});

    //   programs.hbao_calc_blur = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(
    //           GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 0\n#define
    //           AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao_calc_blur = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {aoLayeredPrepend, "#define AO_DEINTERLEAVED 0\n#define AO_BLUR 1\n",
         "ssao/common.h", "ssao/hbao.frag.glsl"});

    //   programs.hbao_blur = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "#define AO_BLUR_PRESENT 0\n",
    //                                       "hbao_blur.frag.glsl"));

    programs.hbao_blur =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"#define AO_BLUR_PRESENT 0\n", "ssao/hbao_blur.frag.glsl"});

    //   programs.hbao_blur2 = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(
    //           GL_VERTEX_SHADER,
    //           "fullscreenquad.vert.glsl"),  // "halfscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "#define AO_BLUR_PRESENT 1\n",
    //                                       "hbao_blur.frag.glsl"));

    programs.hbao_blur2 =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"#define AO_BLUR_PRESENT 1\n", "ssao/hbao_blur.frag.glsl"});

    //   programs.hbao2_calc = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    // #if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
    //       nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,
    //                                       "fullscreenquad.geo.glsl"),
    // #endif
    //       nvgl::ProgramManager::Definition(
    //           GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 1\n#define
    //           AO_BLUR 0\n", "hbao.frag.glsl"));

    programs.hbao2_calc = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {
          aoLayeredPrepend, "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 0\n",
              "ssao/common.h", "ssao/hbao.frag.glsl"
        }
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
        ,
        { "ssao/fullscreenquad.geo.glsl" }
#endif
    );

    //   programs.hbao2_calc_blur = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    // #if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
    //       nvgl::ProgramManager::Definition(GL_GEOMETRY_SHADER,
    //                                       "fullscreenquad.geo.glsl"),
    // #endif
    //       nvgl::ProgramManager::Definition(
    //           GL_FRAGMENT_SHADER, "#define AO_DEINTERLEAVED 1\n#define
    //           AO_BLUR 1\n", "hbao.frag.glsl"));

    programs.hbao2_calc_blur = HBAOShader(
        {"ssao/fullscreenquad.vert.glsl"},
        {
          aoLayeredPrepend, "#define AO_DEINTERLEAVED 1\n#define AO_BLUR 1\n",
              "ssao/common.h", "ssao/hbao.frag.glsl"
        }
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
        ,
        { "ssao/fullscreenquad.geo.glsl" }
#endif
    );

    //   programs.hbao2_deinterleave = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "hbao_deinterleave.frag.glsl"));

    programs.hbao2_deinterleave =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"ssao/hbao_deinterleave.frag.glsl"});

    //   programs.hbao2_reinterleave = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "#define AO_BLUR 0\n",
    //                                       "hbao_reinterleave.frag.glsl"));

    programs.hbao2_reinterleave =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"#define AO_BLUR 0\n", "ssao/hbao_reinterleave.frag.glsl"});

    //   programs.hbao2_reinterleave_blur = m_progManager.createProgram(
    //       nvgl::ProgramManager::Definition(GL_VERTEX_SHADER,
    //                                       "fullscreenquad.vert.glsl"),
    //       nvgl::ProgramManager::Definition(GL_FRAGMENT_SHADER,
    //                                       "#define AO_BLUR 1\n",
    //                                       "hbao_reinterleave.frag.glsl"));

    programs.hbao2_reinterleave_blur =
        HBAOShader({"ssao/fullscreenquad.vert.glsl"},
                   {"#define AO_BLUR 1\n", "ssao/hbao_reinterleave.frag.glsl"});
  }

  bool initMisc() {
    std::mt19937 rmt;

    float numDir = 8;  // keep in sync to glsl

    signed short hbaoRandomShort[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES * 4];

    for (int i = 0; i < HBAO_RANDOM_ELEMENTS * MAX_SAMPLES; i++) {
      float Rand1 = static_cast<float>(rmt()) / 4294967296.0f;
      float Rand2 = static_cast<float>(rmt()) / 4294967296.0f;

      // Use random rotation angles in [0,2PI/NUM_DIRECTIONS)
      float Angle = 2.f * Mn::Constants::pi() * Rand1 / numDir;
      m_hbaoRandom[i].x() = cosf(Angle);
      m_hbaoRandom[i].y() = sinf(Angle);
      m_hbaoRandom[i].z() = Rand2;
      m_hbaoRandom[i].w() = 0;
#define SCALE ((1 << 15))
      hbaoRandomShort[i * 4 + 0] = (signed short)(SCALE * m_hbaoRandom[i].x());
      hbaoRandomShort[i * 4 + 1] = (signed short)(SCALE * m_hbaoRandom[i].y());
      hbaoRandomShort[i * 4 + 2] = (signed short)(SCALE * m_hbaoRandom[i].z());
      hbaoRandomShort[i * 4 + 3] = (signed short)(SCALE * m_hbaoRandom[i].w());
#undef SCALE
    }

    // nvgl::newTexture(textures.hbao_random, GL_TEXTURE_2D_ARRAY);
    // glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao_random);
    // glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGBA16_SNORM, HBAO_RANDOM_SIZE,
    //               HBAO_RANDOM_SIZE, MAX_SAMPLES);
    // glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, HBAO_RANDOM_SIZE,
    //                 HBAO_RANDOM_SIZE, MAX_SAMPLES, GL_RGBA, GL_SHORT,
    //                 hbaoRandomShort);
    // glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    // glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
    Mn::Vector3i size{HBAO_RANDOM_SIZE, HBAO_RANDOM_SIZE, MAX_SAMPLES};

    textures.hbao_random.setMinificationFilter(Mn::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::SamplerFilter::Nearest)
        .setStorage(1, Mn::GL::TextureFormat::RGBA16Snorm, size)
        .setSubImage(0, {},
                     Mn::ImageView3D{Mn::PixelFormat::RGBA16Snorm, size,
                                     hbaoRandomShort});

    // for (int i = 0; i < MAX_SAMPLES; i++) {
    //   glGenTextures(1, &textures.hbao_randomview[i]);
    //   glTextureView(textures.hbao_randomview[i], GL_TEXTURE_2D,
    //                 textures.hbao_random, GL_RGBA16_SNORM, 0, 1, i, 1);
    //   glBindTexture(GL_TEXTURE_2D, textures.hbao_randomview[i]);
    //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //   glBindTexture(GL_TEXTURE_2D, 0);
    // }

    for (int i = 0; i < MAX_SAMPLES; i++) {
      textures.hbao_randomview[i]
          .setMinificationFilter(Mn::SamplerFilter::Nearest)
          .setMagnificationFilter(Mn::SamplerFilter::Nearest);
      glTextureView(textures.hbao_randomview[i].id(), GL_TEXTURE_2D,
                    textures.hbao_random.id(), GL_RGBA16_SNORM, 0, 1, i, 1);
    }

    // nvgl::newBuffer(buffers.hbao_ubo);
    // glNamedBufferStorage(buffers.hbao_ubo, sizeof(HBAOData), NULL,
    //                     GL_DYNAMIC_STORAGE_BIT);
    // todo

    buffers.hbao_ubo.setStorage(sizeof(HBAOData),
                                Mn::GL::Buffer::StorageFlag::DynamicStorage);

    return true;
  }

  bool initFramebuffers(int width, int height, int samples) {
    // nvgl::newTexture(textures.scene_depthlinear, GL_TEXTURE_2D);
    // glBindTexture(GL_TEXTURE_2D, textures.scene_depthlinear);
    // glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, width, height);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // glBindTexture(GL_TEXTURE_2D, 0);
    textures.scene_depthlinear
        .setStorage(1, Mn::GL::TextureFormat::R32F, Mn::Vector2i{width, height})
        .setWrapping({Mn::SamplerWrapping::ClampToEdge,
                      Mn::SamplerWrapping::ClampToEdge})
        .setMinificationFilter(Mn::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::SamplerFilter::Nearest);

    // nvgl::newFramebuffer(fbos.depthlinear);
    // glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);
    // glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
    //                     textures.scene_depthlinear, 0);
    // glBindFramebuffer(GL_FRAMEBUFFER, 0);
    fbos.depthlinear = Mn::GL::Framebuffer({{}, {width, height}});
    fbos.depthlinear.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                                   textures.scene_depthlinear, 0);

#if 0  // todo: port
    nvgl::newTexture(textures.scene_viewnormal, GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textures.scene_viewnormal);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    nvgl::newFramebuffer(fbos.viewnormal);
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.viewnormal);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                        textures.scene_viewnormal, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

#if USE_AO_SPECIALBLUR
    GLenum formatAO = GL_RG16F;
    GLint swizzle[4] = {GL_RED, GL_GREEN, GL_ZERO, GL_ZERO};
#else
    GLenum formatAO = GL_R8;
    GLint swizzle[4] = {GL_RED, GL_RED, GL_RED, GL_RED};
#endif

    nvgl::newTexture(textures.hbao_result, GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textures.hbao_result);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    nvgl::newTexture(textures.hbao_blur, GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textures.hbao_blur);
    glTexStorage2D(GL_TEXTURE_2D, 1, formatAO, width, height);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    nvgl::newFramebuffer(fbos.hbao_calc);
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao_calc);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                        textures.hbao_result, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, textures.hbao_blur,
                        0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // interleaved hbao

    int quarterWidth = ((width + 3) / 4);
    int quarterHeight = ((height + 3) / 4);

    nvgl::newTexture(textures.hbao2_deptharray, GL_TEXTURE_2D_ARRAY);
    glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao2_deptharray);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32F, quarterWidth, quarterHeight,
                  HBAO_RANDOM_ELEMENTS);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

    for (int i = 0; i < HBAO_RANDOM_ELEMENTS; i++) {
      if (textures.hbao2_depthview[i]) {
        glDeleteTextures(1, &textures.hbao2_depthview[i]);
      }
      glGenTextures(1, &textures.hbao2_depthview[i]);
      glTextureView(textures.hbao2_depthview[i], GL_TEXTURE_2D,
                    textures.hbao2_deptharray, GL_R32F, 0, 1, i, 1);
      glBindTexture(GL_TEXTURE_2D, textures.hbao2_depthview[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glBindTexture(GL_TEXTURE_2D, 0);
    }

    nvgl::newTexture(textures.hbao2_resultarray, GL_TEXTURE_2D_ARRAY);
    glBindTexture(GL_TEXTURE_2D_ARRAY, textures.hbao2_resultarray);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, formatAO, quarterWidth, quarterHeight,
                  HBAO_RANDOM_ELEMENTS);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

    GLenum drawbuffers[NUM_MRT];
    for (int layer = 0; layer < NUM_MRT; layer++) {
      drawbuffers[layer] = GL_COLOR_ATTACHMENT0 + layer;
    }

    nvgl::newFramebuffer(fbos.hbao2_deinterleave);
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_deinterleave);
    glDrawBuffers(NUM_MRT, drawbuffers);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    nvgl::newFramebuffer(fbos.hbao2_calc);
    glBindFramebuffer(GL_FRAMEBUFFER, fbos.hbao2_calc);
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_IMAGE
    // this fbo will not have any attachments and therefore requires rasterizer to
    // be configured through default parameters
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH,
                            quarterWidth);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT,
                            quarterHeight);
#endif
#if USE_AO_LAYERED_SINGLEPASS == AO_LAYERED_GS
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                        textures.hbao2_resultarray, 0);
#endif

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

#endif

    return true;
  }

  void drawLinearDepth(float projNearPlane,
                       float projFarPlane,
                       bool isProjOrtho,
                       int width,
                       int height,
                       int sampleIdx) {
    // temp resetState required because we're mixing Magnum OpenGL and external
    // OpenGL.
    Mn::GL::Context::current().resetState();

    // glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);
    //  todo
    //  setViewport() doesn't need to be called if a correct viewport was passed
    //  to hbao2CalcFramebuffer constructor already
    //.setViewport({{}, {quarterWidth, quarterHeight}})
    fbos.depthlinear.bind();

    if (m_config.samples > 1) {
      // glUseProgram(m_progManager.get(programs.depth_linearize_msaa));
      // glUniform4f(0, projNearPlane * projFarPlane,
      //             projNearPlane - projFarPlane, projFarPlane,
      //             isProjOrtho ? 0.0f : 1.0f);
      // glUniform1i(1, sampleIdx);
      programs.depth_linearize_msaa.setUniform(
          0, Mn::Vector4(projNearPlane * projFarPlane,
                         projNearPlane - projFarPlane, projFarPlane,
                         isProjOrtho ? 0.0f : 1.0f));
      programs.depth_linearize_msaa.setUniform(1, sampleIdx);

      // nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE,
      //                       scene_depthstencil);
      depthRenderTexture_->bind(0);

      // glDrawArrays(GL_TRIANGLES, 0, 3);
      programs.depth_linearize_msaa.draw(fullScreenTriangle_);

      // nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D_MULTISAMPLE, 0);
      // don't need to clear state in Magnum
    } else {
      // glUseProgram(m_progManager.get(programs.depth_linearize));
      // glUniform4f(0, projNearPlane * projFarPlane,
      //             projNearPlane - projFarPlane, projFarPlane,
      //             isProjOrtho ? 0.0f : 1.0f);
      programs.depth_linearize.setUniform(
          0, Mn::Vector4(projNearPlane * projFarPlane,
                         projNearPlane - projFarPlane, projFarPlane,
                         isProjOrtho ? 0.0f : 1.0f));

      // nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, scene_depthstencil);
      depthRenderTexture_->bind(0);  // todo: double-check

      // glDrawArrays(GL_TRIANGLES, 0, 3);
      programs.depth_linearize.draw(fullScreenTriangle_);

      // nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D, 0);
      // don't need to clear state in Magnum
    }
  }

  void tempCall(const std::string& name) {
    // temp resetState required because we're mixing Magnum OpenGL and external
    // OpenGL.
    Mn::GL::Context::current().resetState();

    if (name == "placeholder") {
      // placeholder
    } else if (name == "textures.scene_depthlinear 0") {
      // nvgl::bindMultiTexture(GL_TEXTURE0, GL_TEXTURE_2D,
      //                        textures.scene_depthlinear);
      // inline void bindMultiTexture(GLenum target, GLenum textarget, GLuint
      // tex) {
      //   glActiveTexture(target);
      //   glBindTexture(textarget, tex);
      // }
      textures.scene_depthlinear.bind(0);
    } else if (name == "textures.scene_depthlinear 1") {
      // nvgl::bindMultiTexture(GL_TEXTURE1, GL_TEXTURE_2D,
      //                        textures.scene_depthlinear);
      // todo
      textures.scene_depthlinear.bind(1);
    } else if (name == "fbos.depthlinear") {
      // glBindFramebuffer(GL_FRAMEBUFFER, fbos.depthlinear);
      fbos.depthlinear.bind();

    } else if (name == "programs.depth_linearize") {
      programs.depth_linearize.use();
    } else if (name == "programs.depth_linearize_msaa") {
      programs.depth_linearize_msaa.use();
    } else if (name == "programs.depth_linearize draw") {
      // glUseProgram(m_progManager.get(programs.depth_linearize));
      // glDrawArrays(GL_TRIANGLES, 0, 3);
      programs.depth_linearize.draw(fullScreenTriangle_);
    } else if (name == "programs.depth_linearize_msaa draw") {
      // glUseProgram(m_progManager.get(programs.depth_linearize_msaa));
      // glDrawArrays(GL_TRIANGLES, 0, 3);
      programs.depth_linearize_msaa.draw(fullScreenTriangle_);
    } else if (name == "programs.viewnormal") {
      programs.viewnormal.use();
    } else if (name == "programs.bilateralblur") {
      programs.bilateralblur.use();
    } else if (name == "programs.displaytex") {
      programs.displaytex.use();
    } else if (name == "programs.hbao_calc") {
      programs.hbao_calc.use();
    } else if (name == "programs.hbao_calc_blur") {
      programs.hbao_calc_blur.use();
    } else if (name == "programs.hbao_blur") {
      programs.hbao_blur.use();
    } else if (name == "programs.hbao_blur2") {
      programs.hbao_blur2.use();
    } else if (name == "programs.hbao2_deinterleave") {
      programs.hbao2_deinterleave.use();
    } else if (name == "programs.hbao2_calc") {
      programs.hbao2_calc.use();
    } else if (name == "programs.hbao2_calc_blur") {
      programs.hbao2_calc_blur.use();
    } else if (name == "programs.hbao2_reinterleave") {
      programs.hbao2_reinterleave.use();
    } else if (name == "programs.hbao2_reinterleave_blur") {
      programs.hbao2_reinterleave_blur.use();
    } else {
      CORRADE_ASSERT_UNREACHABLE("tempCall unhandled case", {});
    }
  }

  struct {
    HBAOShader depth_linearize, depth_linearize_msaa, viewnormal, bilateralblur,
        displaytex,

        hbao_calc, hbao_calc_blur, hbao_blur, hbao_blur2,

        hbao2_deinterleave, hbao2_calc, hbao2_calc_blur, hbao2_reinterleave,
        hbao2_reinterleave_blur;

  } programs;

  struct {
    Mn::GL::Texture2D scene_viewnormal;
    Mn::GL::Texture2D scene_depthlinear;
    Mn::GL::Texture2D hbao_result;
    Mn::GL::Texture2D hbao_blur;
    Mn::GL::Texture2DArray hbao_random;
    Mn::GL::Texture2DArray hbao_randomview[MAX_SAMPLES];
    Mn::GL::Texture2D hbao2_deptharray;
    Mn::GL::Texture2D hbao2_depthview[HBAO_RANDOM_ELEMENTS];
    Mn::GL::Texture2D hbao2_resultarray;
  } textures;

  struct {
    Mn::GL::Buffer hbao_ubo;
  } buffers;

  struct {
    // GLuint scene              = 0;
    Mn::GL::Framebuffer depthlinear{Mn::NoCreate};
    // Mn::GL::Framebuffer viewnormal(Cr::NoCreateT);
    // Mn::GL::Framebuffer hbao_calc(Cr::NoCreateT);
    // Mn::GL::Framebuffer hbao2_deinterleave(Cr::NoCreateT);
    // Mn::GL::Framebuffer hbao2_calc(Cr::NoCreateT);
  } fbos;

  // HBAOData m_hbaoUbo;
  // nvmath::vec4f m_hbaoRandom[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES];
  Mn::Vector4 m_hbaoRandom[HBAO_RANDOM_ELEMENTS * MAX_SAMPLES];
  // nvgl::ProgramManager m_progManager;
  // GLuint scene_depthstencil = 0;  // not owned

  Mn::GL::Mesh fullScreenTriangle_;
  Config m_config;
  Mn::GL::Texture2D* depthRenderTexture_ = nullptr;
  Mn::GL::Framebuffer* framebuffer_ = nullptr;
};

HBAOHelper2::HBAOHelper2() : pimpl_(spimpl::make_unique_impl<Impl>()) {}

void HBAOHelper2::tempCall(const std::string& name) {
  pimpl_->tempCall(name);
}

void HBAOHelper2::init(Mn::GL::Texture2D* depthRenderTexture,
                       Mn::GL::Framebuffer* framebuffer) {
  pimpl_->init(depthRenderTexture, framebuffer);
}

void HBAOHelper2::tempDrawLinearDepth(float projNearPlane,
                                      float projFarPlane,
                                      bool isProjOrtho,
                                      int width,
                                      int height,
                                      int sampleIdx) {
  pimpl_->drawLinearDepth(projNearPlane, projFarPlane, isProjOrtho, width,
                          height, sampleIdx);
}

}  // namespace ssao
}  // namespace gfx
}  // namespace esp
