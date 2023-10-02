// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Hbao.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Format.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Framebuffer.h>
#ifndef MAGNUM_TARGET_WEBGL
#include <Magnum/GL/ImageFormat.h>
#endif
#include <Magnum/GL/Mesh.h>
#ifndef MAGNUM_TARGET_WEBGL
#include <Magnum/GL/MultisampleTexture.h>
#endif
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/PixelFormat.h>
#include <random>

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxBatchShaderResources)
}

namespace esp {
namespace gfx_batch {

namespace Cr = Corrade;
namespace Mn = Magnum;
using Cr::Containers::Literals::operator""_s;

enum {
  AoRandomTextureSize = 4,
  MaxSamples = 8,
  HbaoRandomSize = AoRandomTextureSize,
  FragmentOutputCount = 8,
  HbaoRandomNumElements = HbaoRandomSize * HbaoRandomSize
};

#ifndef MAGNUM_TARGET_GLES
constexpr Mn::GL::Version GlslVersion = Mn::GL::Version::GL330;
#else
constexpr Mn::GL::Version GlslVersion = Mn::GL::Version::GLES300;
#endif

class HbaoBlurShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int { SourceTextureBinding = 0, LinearDepthTextureBinding = 1 };

 public:
  explicit HbaoBlurShader(bool bilateral, Mn::Int aoBlurPass)
      : bilateral_{bilateral} {
    CORRADE_INTERNAL_ASSERT(!bilateral || !aoBlurPass);

    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
    if (bilateral) {
      frag.addSource(rs.getString("hbao/bilateralblur.frag"));
    } else {
      // Either first pass (0) or second pass (1)
      if (aoBlurPass == 1) {
        frag.addSource("#define AO_BLUR_SECOND_PASS\n"_s);
      }
      frag.addSource(rs.getString("hbao/hbao_blur.frag"));
    }

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    sharpnessUniform_ = uniformLocation("uGaussSharpness");
    inverseResolutionDirectionUniform_ =
        uniformLocation("uGaussInvResDirection");
    setUniform(uniformLocation("uTexSource"), SourceTextureBinding);
    if (bilateral) {
      setUniform(uniformLocation("uTexLinearDepth"), LinearDepthTextureBinding);
    }
  }

  HbaoBlurShader& setSharpness(Mn::Float sharpness) {
    setUniform(sharpnessUniform_, sharpness);
    return *this;
  }

  HbaoBlurShader& setInverseResolutionDirection(const Mn::Vector2& direction) {
    setUniform(inverseResolutionDirectionUniform_, direction);
    return *this;
  }

  HbaoBlurShader& bindSourceTexture(Mn::GL::Texture2D& texture) {
    texture.bind(SourceTextureBinding);
    return *this;
  }

  HbaoBlurShader& bindLinearDepthTexture(Mn::GL::Texture2D& texture) {
    CORRADE_INTERNAL_ASSERT(bilateral_);
    texture.bind(LinearDepthTextureBinding);
    return *this;
  }

 private:
  Mn::Int sharpnessUniform_, inverseResolutionDirectionUniform_;
  bool bilateral_;
};

// TODO replace with own
class DepthLinearizeShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int { InputTextureBinding = 0 };

 public:
  // TODO MSAA is never used, drop
  explicit DepthLinearizeShader(bool msaa) : msaa_{msaa} {
    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
    if (msaa) {
      frag.addSource("#define DEPTHLINEARIZE_MSAA\n"_s);
    }
    frag.addSource(rs.getString("hbao/depthlinearize.frag"));

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    clipInfoUniform_ = uniformLocation("uClipInfo");
    if (msaa) {
      sampleIndexUniform_ = uniformLocation("uSampleIndex");
    }
    setUniform(uniformLocation("uInputTexture"), InputTextureBinding);
  }

  DepthLinearizeShader& setClipInfo(const Mn::Vector4& info) {
    setUniform(clipInfoUniform_, info);
    return *this;
  }

  DepthLinearizeShader& setSampleIndex(Mn::Int index) {
    CORRADE_INTERNAL_ASSERT(msaa_);
    setUniform(sampleIndexUniform_, index);
    return *this;
  }

  DepthLinearizeShader& bindInputTexture(Mn::GL::Texture2D& texture) {
    CORRADE_INTERNAL_ASSERT(!msaa_);
    texture.bind(InputTextureBinding);
    return *this;
  }

#ifndef MAGNUM_TARGET_WEBGL
  DepthLinearizeShader& bindInputTexture(
      Mn::GL::MultisampleTexture2D& texture) {
    CORRADE_INTERNAL_ASSERT(msaa_);
    texture.bind(InputTextureBinding);
    return *this;
  }
#endif

 private:
  Mn::Int clipInfoUniform_, sampleIndexUniform_;
  bool msaa_;
};

class ViewNormalShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int { LinearDepthTextureBinding = 0 };

 public:
  explicit ViewNormalShader() {
    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
    frag.addSource(rs.getString("hbao/viewnormal.frag"));

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    projectionInfoUniform_ = uniformLocation("uProjInfo");
    projectionOrthographicUniform_ = uniformLocation("uProjOrtho");
    inverseFullResolutionUniform_ = uniformLocation("uInvFullResolution");
    setUniform(uniformLocation("uTexLinearDepth"), LinearDepthTextureBinding);
  }

  ViewNormalShader& setProjectionInfo(const Mn::Vector4& info) {
    setUniform(projectionInfoUniform_, info);
    return *this;
  }

  ViewNormalShader& setProjectionOrthographic(Mn::Int orthographic) {
    setUniform(projectionOrthographicUniform_, orthographic);
    return *this;
  }

  ViewNormalShader& setInverseFullResolution(const Mn::Vector2& resolution) {
    setUniform(inverseFullResolutionUniform_, resolution);
    return *this;
  }

  ViewNormalShader& bindLinearDepthTexture(Mn::GL::Texture2D& texture) {
    texture.bind(LinearDepthTextureBinding);
    return *this;
  }

 private:
  Mn::Int projectionInfoUniform_, projectionOrthographicUniform_,
      inverseFullResolutionUniform_;
};

class HbaoCalcShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int {
    UniformBufferBinding = 0,
    LinearDepthTextureBinding = 0,
    ViewNormalTextureBinding = 1,
    RandomTextureBinding = 1,
    OutputImageBinding = 0
  };

 public:
  enum Layered { Off = 0, ImageLoadStore = 1, GeometryShader = 2 };

  explicit HbaoCalcShader(Mn::NoCreateT)
      : Mn::GL::AbstractShaderProgram{Mn::NoCreate} {}

  explicit HbaoCalcShader(bool deinterleaved,
                          bool specialBlur,
                          Layered layered,
                          bool textureArrayLayer)
      : deinterleaved_{deinterleaved},
#ifndef MAGNUM_TARGET_WEBGL
        specialBlur_{specialBlur},
#endif
        textureArrayLayer_{textureArrayLayer},
        layered_{layered} {
    CORRADE_INTERNAL_ASSERT(deinterleaved || layered == Layered::Off);
    CORRADE_INTERNAL_ASSERT(deinterleaved || !textureArrayLayer);

    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
#ifndef MAGNUM_TARGET_WEBGL

    Mn::GL::Shader geom{Mn::NoCreate};
    if (layered == Layered::GeometryShader) {
      const bool passthroughSupported =
/* Needs
 * https://github.com/mosra/magnum/commit/5f287df332fee7fcc2df6e8055853b86d1db3ed2
 */
#if 0
        Mn::GL::Context::current().isExtensionSupported<Mn::GL::Extensions::NV::geometry_shader_passthrough>(GlslVersion);
#else
          Mn::GL::Context::current().detectedDriver() >=
          Mn::GL::Context::DetectedDriver::NVidia;
#endif
          ;

      geom = Mn::GL::Shader{GlslVersion, Mn::GL::Shader::Type::Geometry};
      if (passthroughSupported) {
        geom.addSource("#define USE_GEOMETRY_SHADER_PASSTHROUGH\n"_s);

        frag.addSource("#define USE_GEOMETRY_SHADER_PASSTHROUGH\n"_s);
      }
      geom.addSource(rs.getString("hbao/fullscreenquad.geom"));
    }  // layered == Layered::GeometryShader

#endif

    frag
        .addSource(Cr::Utility::format(
            "{}{}{}"
            "#define AO_LAYERED {}\n"
            "#define AO_RANDOMTEX_SIZE {}\n",
            deinterleaved ? "#define AO_DEINTERLEAVED\n"_s : "",
            specialBlur ? "#define AO_SPECIAL_BLUR\n"_s : "",
            textureArrayLayer ? "#define AO_TEXTUREARRAY_LAYER\n"_s : "",
            Mn::Int(layered), AoRandomTextureSize))
        .addSource(rs.getString("hbao/hbao.frag"));

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

#ifndef MAGNUM_TARGET_WEBGL
    if (layered == Layered::GeometryShader) {
      CORRADE_INTERNAL_ASSERT(geom.compile());
      attachShaders({vert, geom, frag});
    } else
#endif
      attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    setUniformBlockBinding(uniformBlockIndex("uControlBuffer"),
                           UniformBufferBinding);
    if (deinterleaved) {
      if (layered == Layered::Off) {
        float2OffsetUniform_ = uniformLocation("uFloat2Offset");
        jitterUniform_ = uniformLocation("uJitter");
      } else if (layered == Layered::ImageLoadStore) {
        setUniform(uniformLocation("uImgOutput"), OutputImageBinding);
      }
    }
    setUniform(uniformLocation("texLinearDepth"), LinearDepthTextureBinding);
    if (deinterleaved) {
      setUniform(uniformLocation("texViewNormal"), ViewNormalTextureBinding);
      if (textureArrayLayer) {
        linearDepthTextureSliceUniform_ = uniformLocation("uLinearDepthSlice");
      }
    } else {
      setUniform(uniformLocation("texRandom"), RandomTextureBinding);
      randomSliceUniform_ = uniformLocation("uRandomSlice");
    }
  }

  HbaoCalcShader& setFloat2Offset(const Mn::Vector2& offset) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ == Layered::Off);
    setUniform(float2OffsetUniform_, offset);
    return *this;
  }

  HbaoCalcShader& setJitter(const Mn::Vector4& jitter) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ == Layered::Off);
    setUniform(jitterUniform_, jitter);
    return *this;
  }

  HbaoCalcShader& bindLinearDepthTexture(Mn::GL::Texture2D& texture) {
    // classic algorithm
    CORRADE_INTERNAL_ASSERT(!deinterleaved_ || layered_ == Layered::Off);
    texture.bind(LinearDepthTextureBinding);
    return *this;
  }

  HbaoCalcShader& bindLinearDepthTexture(Mn::GL::Texture2DArray& texture) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ != Layered::Off &&
                            !textureArrayLayer_);
    // If cached and layered
    texture.bind(LinearDepthTextureBinding);
    return *this;
  }

  HbaoCalcShader& bindLinearDepthTexture(Mn::GL::Texture2DArray& texture,
                                         Mn::Int slice) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ == Layered::Off &&
                            textureArrayLayer_);
    // If cached, not layered and using textureArrayLayer
    texture.bind(LinearDepthTextureBinding);
    setUniform(linearDepthTextureSliceUniform_, Mn::Float(slice));
    return *this;
  }

  HbaoCalcShader& bindViewNormalTexture(Mn::GL::Texture2D& texture) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_);
    texture.bind(ViewNormalTextureBinding);
    return *this;
  }

  HbaoCalcShader& bindRandomTexture(Mn::GL::Texture2DArray& texture,
                                    Mn::Int slice) {
    CORRADE_INTERNAL_ASSERT(!deinterleaved_);
    texture.bind(RandomTextureBinding);
    setUniform(randomSliceUniform_, Mn::Float(slice));
    return *this;
  }

  HbaoCalcShader& bindUniformBuffer(Mn::GL::Buffer& buffer) {
    buffer.bind(Mn::GL::Buffer::Target::Uniform, UniformBufferBinding);
    return *this;
  }

#ifndef MAGNUM_TARGET_WEBGL
  HbaoCalcShader& bindOutputImage(Mn::GL::Texture2DArray& texture,
                                  Mn::Int level) {
    CORRADE_INTERNAL_ASSERT(deinterleaved_ &&
                            layered_ == Layered::ImageLoadStore);
    texture.bindImageLayered(
        OutputImageBinding, level, Mn::GL::ImageAccess::WriteOnly,
        specialBlur_ ? Mn::GL::ImageFormat::RG16F : Mn::GL::ImageFormat::R8);
    return *this;
  }
#endif

 private:
  Mn::Int float2OffsetUniform_, jitterUniform_, linearDepthTextureSliceUniform_,
      randomSliceUniform_;
  bool deinterleaved_,
#ifndef MAGNUM_TARGET_WEBGL
      specialBlur_,
#endif

      textureArrayLayer_;
  Layered layered_;
};

class HbaoDeinterleaveShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int {
    LinearDepthTextureBinding = 0,
  };

 public:
  explicit HbaoDeinterleaveShader() {
    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
#ifndef MAGNUM_TARGET_GLES
    if (Mn::GL::Context::current()
            .isExtensionSupported<Mn::GL::Extensions::ARB::gpu_shader5>(
                GlslVersion))
      frag.addSource("#define USE_TEXTURE_GATHER\n"_s);
#endif
    frag.addSource(rs.getString("hbao/hbao_deinterleave.frag"));

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    projectionInfoUniform_ = uniformLocation("uUVOffsetInvResInfo");
    setUniform(uniformLocation("uTexLinearDepth"), LinearDepthTextureBinding);
  }

  HbaoDeinterleaveShader& setUVOffsetInvResInfo(
      const Mn::Vector2& uvOffset,
      const Mn::Vector2& inverseResolution) {
    setUniform(projectionInfoUniform_,
               Mn::Vector4{uvOffset.x(), uvOffset.y(), inverseResolution.x(),
                           inverseResolution.y()});
    return *this;
  }

  HbaoDeinterleaveShader& bindLinearDepthTexture(Mn::GL::Texture2D& texture) {
    texture.bind(LinearDepthTextureBinding);
    return *this;
  }

 private:
  Mn::Int projectionInfoUniform_;
};

class HbaoReinterleaveShader : public Mn::GL::AbstractShaderProgram {
 private:
  enum : Mn::Int { ResultsTextureBinding = 0 };

 public:
  explicit HbaoReinterleaveShader(bool specialBlur) {
    Cr::Utility::Resource rs{"gfx-batch-shaders"};

    // TODO use our own
    Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
    vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

    Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
    if (specialBlur) {
      frag.addSource("#define AO_SPECIAL_BLUR\n"_s);
    }
    frag.addSource(rs.getString("hbao/hbao_reinterleave.frag"));

    CORRADE_INTERNAL_ASSERT(vert.compile());
    CORRADE_INTERNAL_ASSERT(frag.compile());

    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    setUniform(uniformLocation("uTexResultsArray"), ResultsTextureBinding);
  }

  HbaoReinterleaveShader& bindResultsTexture(Mn::GL::Texture2DArray& texture) {
    texture.bind(ResultsTextureBinding);
    return *this;
  }
};

/**
 * @brief This struct holds the uniform data that are passed to the various
 * shaders. If fields are added they should be sized multiples of 16 bytes,
 * with padding added if appropriate.
 */
struct HbaoUniformData {
  Mn::Float radiusToScreen;
  Mn::Float r2;
  Mn::Float negInvR2;
  Mn::Float nDotVBias;

  Mn::Vector2 invFullResolution;
  Mn::Vector2 invQuarterResolution;

  Mn::Float aoMultiplier;
  Mn::Float powExponent;
  // padding
  Mn::Int : 32;
  Mn::Int : 32;

  Mn::Vector4 projInfo;

  Mn::Vector2 projScale;
  Mn::Int projOrtho;
  Mn::Int _pad1;

  Mn::Vector4 float2Offsets[AoRandomTextureSize * AoRandomTextureSize];
  Mn::Vector4 jitters[AoRandomTextureSize * AoRandomTextureSize];
};

static_assert(sizeof(HbaoUniformData) % 16 == 0, "Not a nice uniform struct");

struct Hbao::State {
  Mn::GL::Texture2D sceneDepthLinear;
  Mn::GL::Framebuffer depthLinear{Mn::NoCreate};

  Mn::GL::Texture2D sceneViewNormal;
  Mn::GL::Framebuffer viewNormal{Mn::NoCreate};

  Mn::GL::Texture2D hbaoResult;
  Mn::GL::Texture2D hbaoBlur;
  Mn::GL::Texture2DArray hbaoRandom;
  Mn::GL::Framebuffer hbaoCalc{Mn::NoCreate};
  Mn::GL::Framebuffer hbao2Deinterleave{Mn::NoCreate};
  Mn::GL::Framebuffer hbao2Calc{Mn::NoCreate};

  Mn::GL::Texture2DArray hbao2DepthArray;
  Mn::GL::Texture2DArray hbao2ResultArray;

  HbaoBlurShader bilateralBlurShader{
      /*bilateral*/ true,
      /*aoBlurPass value is ignored for bilateral*/ 0};
  HbaoBlurShader hbaoSpecialBlurShaderFirstPass{/*bilateral*/ false,
                                                /*aoBlurPass*/ 0};
  HbaoBlurShader hbaoSpecialBlurShader2ndPass{/*bilateral*/ false,
                                              /*aoBlurPass*/ 1};
  DepthLinearizeShader depthLinearizeShader{false};
  DepthLinearizeShader depthLinearizeShaderMsaa{true};
  ViewNormalShader viewNormalShader;
  HbaoCalcShader hbaoCalcShader{/*deinterleaved*/ false,
                                /*specialBlur*/ false,
                                {},
                                false};
  HbaoCalcShader hbaoCalcSpecialBlurShader{/*deinterleaved*/ false,
                                           /*specialBlur*/ true,
                                           {},
                                           false};
  /* These depend on config, are created in the constructor */
  HbaoCalcShader hbao2CalcShader{Mn::NoCreate};
  HbaoCalcShader hbao2CalcSpecialBlurShader{Mn::NoCreate};
  HbaoDeinterleaveShader hbao2DeinterleaveShader;
  HbaoReinterleaveShader hbao2ReinterleaveShader{/*specialBlur*/ false};
  HbaoReinterleaveShader hbao2ReinterleaveSpecialBlurShader{
      /*specialBlur*/ true};

  Mn::GL::Buffer hbaoUniform{Mn::GL::Buffer::TargetHint::Uniform};
  HbaoUniformData hbaoUniformData;

  Mn::GL::Mesh triangle, triangleLayered;

  HbaoConfiguration configuration;

  Mn::Vector4 random[HbaoRandomNumElements * MaxSamples];
};

Hbao::Hbao(const HbaoConfiguration& configuration) {
  if (!Cr::Utility::Resource::hasGroup("gfx-batch-shaders")) {
    importShaderResources();
  }

  state_.emplace();
  setConfiguration(configuration);
}

Hbao::Hbao(Hbao&&) noexcept = default;

Hbao::~Hbao() = default;

Hbao& Hbao::operator=(Hbao&&) noexcept = default;

void Hbao::setConfiguration(const HbaoConfiguration& configuration) {
  // TODO probably most of the config should be at runtime, not setup time
  //  though size definitely can't be at runtime unless there's a way to
  //  recreate all framebuffers
  CORRADE_INTERNAL_ASSERT(!configuration.size().isZero());

  /* Only one of these can be set */
  CORRADE_INTERNAL_ASSERT(
      !(configuration.flags() & HbaoFlag::LayeredGeometryShader) ||
      !(configuration.flags() & HbaoFlag::LayeredImageLoadStore));

  /* "init misc" */
  {
    std::mt19937 rmt;

    float numDir = 8;  // keep in sync to glsl
    for (std::size_t i = 0; i < HbaoRandomNumElements * MaxSamples; ++i) {
      // TODO use better random gen method
      float rand1 = Mn::Float(rmt()) / 4294967296.0f;
      float rand2 = Mn::Float(rmt()) / 4294967296.0f;

      // Use random rotation angles in [0,2PI/NUM_DIRECTIONS)
      Mn::Rad angle{Mn::Constants::tau() * rand1 / numDir};

      state_->random[i] = {Mn::Math::cos(angle), Mn::Math::sin(angle), rand2,
                           0.0f};
    }

    /* Pack those to 16-bit and upload to a texture */
    Mn::Vector4s hbaoRandomShort[HbaoRandomNumElements * MaxSamples];
    Mn::Math::packInto(Cr::Containers::arrayCast<2, Mn::Float>(
                           Cr::Containers::stridedArrayView(state_->random)),
                       Cr::Containers::arrayCast<2, Mn::Short>(
                           Cr::Containers::stridedArrayView(hbaoRandomShort)));

    constexpr Mn::Vector3i size{Mn::Vector2i{HbaoRandomSize}, MaxSamples};

    state_->hbaoRandom.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setStorage(1, Mn::GL::TextureFormat::RGBA16Snorm, size)
        .setSubImage(0, {},
                     Mn::ImageView3D{Mn::PixelFormat::RGBA16Snorm, size,
                                     hbaoRandomShort});

    // TODO is it really dynamic?! i don't think so
    state_->hbaoUniform.setData({nullptr, sizeof(HbaoUniformData)},
                                Mn::GL::BufferUsage::DynamicDraw);
  }

  /* "init framebuffers" */
  {
    state_->sceneDepthLinear
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, Mn::GL::TextureFormat::R32F, configuration.size());

    state_->depthLinear = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->depthLinear.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                                      state_->sceneDepthLinear, 0);

    state_->sceneViewNormal
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, Mn::GL::TextureFormat::RGBA8, configuration.size());

    state_->viewNormal = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->viewNormal.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                                     state_->sceneViewNormal, 0);

    const Mn::GL::TextureFormat aoFormat =
        configuration.flags() & HbaoFlag::UseAoSpecialBlur
            ? Mn::GL::TextureFormat::RG16F
            : Mn::GL::TextureFormat::R8;
    state_->hbaoResult.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, aoFormat, configuration.size());
    state_->hbaoBlur.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, aoFormat, configuration.size());

#ifndef MAGNUM_TARGET_WEBGL
    if (configuration.flags() & HbaoFlag::UseAoSpecialBlur) {
      state_->hbaoResult.setSwizzle<'r', 'g', '0', '0'>();
      state_->hbaoBlur.setSwizzle<'r', 'g', '0', '0'>();
    } else {
      state_->hbaoResult.setSwizzle<'r', 'r', 'r', 'r'>();
      state_->hbaoBlur.setSwizzle<'r', 'r', 'r', 'r'>();
    }
#endif

    state_->hbaoCalc = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->hbaoCalc
        .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0},
                       state_->hbaoResult, 0)
        .attachTexture(Mn::GL::Framebuffer::ColorAttachment{1},
                       state_->hbaoBlur, 0);

    const Mn::Vector2i quarterSize =
        (configuration.size() + Mn::Vector2i{3}) / 4;

    state_->hbao2DepthArray
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, Mn::GL::TextureFormat::R32F,
                    {quarterSize, HbaoRandomNumElements});

    state_->hbao2ResultArray
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, aoFormat, {quarterSize, HbaoRandomNumElements});

    state_->hbao2Deinterleave = Mn::GL::Framebuffer{{{}, quarterSize}};
    state_->hbao2Deinterleave.mapForDraw({
        {0, Mn::GL::Framebuffer::ColorAttachment{0}},
        {1, Mn::GL::Framebuffer::ColorAttachment{1}},
        {2, Mn::GL::Framebuffer::ColorAttachment{2}},
        {3, Mn::GL::Framebuffer::ColorAttachment{3}},
        {4, Mn::GL::Framebuffer::ColorAttachment{4}},
        {5, Mn::GL::Framebuffer::ColorAttachment{5}},
        {6, Mn::GL::Framebuffer::ColorAttachment{6}},
        {7, Mn::GL::Framebuffer::ColorAttachment{7}},
    });
    /* Attachments are set up dynamically in drawCacheAwareInternal() */

    state_->hbao2Calc = Mn::GL::Framebuffer{{{}, quarterSize}};

#ifndef MAGNUM_TARGET_WEBGL
    if (configuration.flags() & HbaoFlag::LayeredImageLoadStore) {
      state_->hbao2Calc.setDefaultSize(quarterSize);
    } else if (configuration.flags() & HbaoFlag::LayeredGeometryShader) {
      state_->hbao2Calc.attachLayeredTexture(
          Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbao2ResultArray, 0);
    }
#endif
  }

  {
    HbaoCalcShader::Layered layered = HbaoCalcShader::Layered::Off;
    bool textureArrayLayer = true;
    if (configuration.flags() & HbaoFlag::LayeredImageLoadStore) {
      layered = HbaoCalcShader::Layered::ImageLoadStore;
      textureArrayLayer = false;
    } else if (configuration.flags() & HbaoFlag::LayeredGeometryShader) {
      layered = HbaoCalcShader::Layered::GeometryShader;
      textureArrayLayer = false;
    }
    state_->hbao2CalcShader =
        HbaoCalcShader{/*deinterleaved*/ true, /*specialBlur*/ false, layered,
                       textureArrayLayer};
    state_->hbao2CalcSpecialBlurShader =
        HbaoCalcShader{/*deinterleaved*/ true, /*specialBlur*/ true, layered,
                       textureArrayLayer};
  }

  state_->triangle.setCount(3);
  state_->triangleLayered.setCount(3 * HbaoRandomNumElements);
  state_->configuration = configuration;
}

Magnum::Vector2i Hbao::getFrameBufferSize() const {
  return state_->configuration.size();
}

namespace {

/**
 * Populate uniform data with appropriate values from configuration. Should only
 * be performed when configuration changes.
 */
void prepareHbaoData(
    const HbaoConfiguration& configuration,
    const Mn::Matrix4& projection,
    HbaoUniformData& uniformData,
    Mn::GL::Buffer& uniform,
    Cr::Containers::StaticArrayView<HbaoRandomNumElements * MaxSamples,
                                    const Mn::Vector4> random) {
  /* Radius */
  const Mn::Float meters2viewspace = 1.0f;
  const Mn::Float r = configuration.radius() * meters2viewspace;
  uniformData.r2 = r * r;
  uniformData.negInvR2 = -1.0f / uniformData.r2;
  const Mn::Float projectionScale =
      (uniformData.projOrtho != 0
           ? configuration.size().y() / uniformData.projInfo[1]  // ortho
           // For perspective, projection[0][0] is 1/tan(fov/2)
           : 0.5f * configuration.size().y() * projection[0][0]);  // persp
  uniformData.radiusToScreen = r * 0.5f * projectionScale;

  /* AO */
  uniformData.powExponent = Mn::Math::max(configuration.intensity(), 0.0f);
  uniformData.nDotVBias = Mn::Math::clamp(configuration.bias(), 0.0f, 1.0f);
  uniformData.aoMultiplier = 1.0f / (1.0f - uniformData.nDotVBias);

  /* Resolution */
  const Mn::Vector2i quarterSize = (configuration.size() + Mn::Vector2i{3}) / 4;
  uniformData.invQuarterResolution =
      Mn::Vector2{1.0f} / Mn::Vector2{quarterSize};
  uniformData.invFullResolution =
      Mn::Vector2{1.0f} / Mn::Vector2{configuration.size()};

  if (configuration.flags() &
      (HbaoFlag::LayeredGeometryShader | HbaoFlag::LayeredImageLoadStore))
    for (Mn::Int i = 0; i != HbaoRandomNumElements; ++i) {
      uniformData.float2Offsets[i] = {
          Mn::Float(i % 4) + 0.5f,
          // NOLINTNEXTLINE(bugprone-integer-division). This is deliberate
          Mn::Float(i / 4) + 0.5f,
          0.0f,
          0.0f,
      };
      uniformData.jitters[i] = random[i];
    }

  uniform.setSubData(0, {&uniformData, 1});
}

Mn::Vector4 buildClipInfo(const Mn::Matrix4& projectionMatrix,
                          bool orthographic) {
  // TODO this still looks like there's some better way this should be
  // extracted.  Definitely change to use near and far calcs upstream when
  // available!!
  // Ref :
  // https://github.com/mosra/magnum/commit/0d31f7461b31698ea5bf92ec66ff5056a6ad7360
  if (orthographic) {
    auto nearPlane = (projectionMatrix[3][2] + 1.0f) / projectionMatrix[2][2];
    auto farPlane = (projectionMatrix[3][2] - 1.0f) / projectionMatrix[2][2];
    return {nearPlane * farPlane, nearPlane - farPlane, farPlane, 0.0f};
  }
  // perspective
  auto nearPlane = projectionMatrix[3][2] / (projectionMatrix[2][2] - 1.0f);
  auto farPlane = projectionMatrix[3][2] / (projectionMatrix[2][2] + 1.0f);
  return {nearPlane * farPlane, nearPlane - farPlane, farPlane, 1.0f};
}
}  // namespace

void Hbao::drawLinearDepth(const Mn::Matrix4& projection,
                           Mn::GL::Texture2D& depthStencilInput) {
  state_->depthLinear.bind();
  state_->depthLinearizeShader
      .setClipInfo(
          buildClipInfo(projection, state_->hbaoUniformData.projOrtho == 1))
      .bindInputTexture(depthStencilInput)
      .draw(state_->triangle);
}

void Hbao::drawHbaoBlur(Mn::GL::AbstractFramebuffer& output) {
  constexpr Mn::Float meters2viewspace = 1.0f;

  state_->hbaoCalc.mapForDraw(Mn::GL::Framebuffer::ColorAttachment{1}).bind();
  // Only special blur
  HbaoBlurShader& shader =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur
          ? state_->hbaoSpecialBlurShaderFirstPass
          : state_->bilateralBlurShader;
  shader.setSharpness(state_->configuration.blurSharpness() / meters2viewspace)
      .setInverseResolutionDirection(
          {1.0f / state_->configuration.size().x(), 0.0f})
      .bindSourceTexture(state_->hbaoResult);
  if (!(state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur)) {
    shader.bindLinearDepthTexture(state_->sceneDepthLinear);
  }
  shader.draw(state_->triangle);

  output.bind();
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
  Mn::GL::Renderer::setBlendFunction(
      Mn::GL::Renderer::BlendFunction::Zero,
      Mn::GL::Renderer::BlendFunction::SourceColor,
      Mn::GL::Renderer::BlendFunction::Zero,
      Mn::GL::Renderer::BlendFunction::One);
  // TODO multi samples masking

  // Only special blur
  HbaoBlurShader& secondShader =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur
          ? state_->hbaoSpecialBlurShader2ndPass
          : state_->bilateralBlurShader;
  secondShader
      .setSharpness(state_->configuration.blurSharpness() / meters2viewspace)
      .setInverseResolutionDirection(
          {0.0f, 1.0f / state_->configuration.size().y()})
      .bindSourceTexture(state_->hbaoBlur);
  if (!(state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur)) {
    secondShader.bindLinearDepthTexture(state_->sceneDepthLinear);
  }

  secondShader.draw(state_->triangle);
}

void Hbao::drawEffect(const Mn::Matrix4& projection,
                      HbaoType algType,
                      Mn::GL::Texture2D& depthStencilInput,
                      Mn::GL::AbstractFramebuffer& output) {
  if (projection[3][3] != 0) {
    // Orthographic rendering
    state_->hbaoUniformData.projInfo = {
        2.0f / projection[0][0],
        2.0f / projection[1][1],
        -(1.0f + projection[3][0]) / projection[0][0],
        -(1.0f - projection[3][1]) / projection[1][1],
    };
    state_->hbaoUniformData.projOrtho = 1;
  } else {
    // Perspective rendering -> projection[3][3] == 0
    state_->hbaoUniformData.projInfo = {
        2.0f / projection[0][0],
        2.0f / projection[1][1],
        -(1.0f - projection[2][0]) / projection[0][0],
        -(1.0f + projection[2][1]) / projection[1][1],
    };
    state_->hbaoUniformData.projOrtho = 0;
  }

  // TODO much of this data mapping does not need to be redone every frame
  prepareHbaoData(state_->configuration, projection, state_->hbaoUniformData,
                  state_->hbaoUniform, state_->random);
  drawLinearDepth(projection, depthStencilInput);
  if (algType == HbaoType::CacheAware) {
    drawCacheAwareInternal(output);
  } else {
    drawClassicInternal(output);
  }
}  // Hbao::draw

void Hbao::drawClassicInternal(Mn::GL::AbstractFramebuffer& output) {
  if (state_->configuration.flags() & HbaoFlag::NoBlur) {
    output.bind();
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
    Mn::GL::Renderer::setBlendFunction(
        Mn::GL::Renderer::BlendFunction::Zero,
        Mn::GL::Renderer::BlendFunction::SourceColor,
        Mn::GL::Renderer::BlendFunction::Zero,
        Mn::GL::Renderer::BlendFunction::One);
    // TODO Set sample mask if samples > 1
  } else {
    state_->hbaoCalc.mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0}).bind();
  }

  // only special blur
  HbaoCalcShader& shader =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur
          ? state_->hbaoCalcSpecialBlurShader
          : state_->hbaoCalcShader;
  shader.bindLinearDepthTexture(state_->sceneDepthLinear)
      .bindRandomTexture(state_->hbaoRandom, 0)
      .bindUniformBuffer(state_->hbaoUniform)
      .draw(state_->triangle);

  if (!(state_->configuration.flags() & HbaoFlag::NoBlur)) {
    drawHbaoBlur(output);
  }

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  // TODO reset sample mask if ever used
}

void Hbao::drawCacheAwareInternal(Mn::GL::AbstractFramebuffer& output) {
  state_->viewNormal.bind();

  state_->viewNormalShader.setProjectionInfo(state_->hbaoUniformData.projInfo)
      .setProjectionOrthographic(state_->hbaoUniformData.projOrtho)
      .setInverseFullResolution(state_->hbaoUniformData.invFullResolution)
      .bindLinearDepthTexture(state_->sceneDepthLinear)
      .draw(state_->triangle);

  state_->hbao2Deinterleave.bind();
  state_->hbao2DeinterleaveShader.bindLinearDepthTexture(
      state_->sceneDepthLinear);

  for (Mn::Int i = 0; i < HbaoRandomNumElements; i += FragmentOutputCount) {
    for (Mn::UnsignedInt layer = 0; layer != FragmentOutputCount; ++layer) {
      state_->hbao2Deinterleave.attachTextureLayer(
          Mn::GL::Framebuffer::ColorAttachment{layer}, state_->hbao2DepthArray,
          0, i + layer);
    }

    state_->hbao2DeinterleaveShader
        .setUVOffsetInvResInfo(
            // NOLINTNEXTLINE(bugprone-integer-division). This is deliberate
            {Mn::Float(i % 4) + 0.5f, Mn::Float(i / 4) + 0.5f},
            state_->hbaoUniformData.invFullResolution)
        .draw(state_->triangle);
  }

  state_->hbao2Calc.bind();

  // Only special blur
  HbaoCalcShader& shader =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur
          ? state_->hbao2CalcSpecialBlurShader
          : state_->hbao2CalcShader;

  shader.bindViewNormalTexture(state_->sceneViewNormal)
      .bindUniformBuffer(state_->hbaoUniform);

#ifndef MAGNUM_TARGET_WEBGL
  if (state_->configuration.flags() &
      (HbaoFlag::LayeredGeometryShader | HbaoFlag::LayeredImageLoadStore)) {
    shader.bindLinearDepthTexture(state_->hbao2DepthArray);
    if (state_->configuration.flags() & HbaoFlag::LayeredImageLoadStore) {
      shader.bindOutputImage(state_->hbao2ResultArray, 0);
    }
    shader.draw(state_->triangleLayered);
    if (state_->configuration.flags() & HbaoFlag::LayeredImageLoadStore) {
      Mn::GL::Renderer::setMemoryBarrier(
          Mn::GL::Renderer::MemoryBarrier::TextureFetch |
          Mn::GL::Renderer::MemoryBarrier::ShaderImageAccess);
    }
  } else {
    for (Mn::Int i = 0; i != HbaoRandomNumElements; ++i) {
      state_->hbao2Calc.attachTextureLayer(
          Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbao2ResultArray, 0,
          i);

      // NOLINTNEXTLINE(bugprone-integer-division). This is deliberate
      shader.setFloat2Offset({Mn::Float(i % 4) + 0.5f, Mn::Float(i / 4) + 0.5f})
          .setJitter(state_->random[i])
          .bindLinearDepthTexture(state_->hbao2DepthArray, i)
          .draw(state_->triangle);
    }
  }
#else
  for (Mn::Int i = 0; i != HbaoRandomNumElements; ++i) {
    state_->hbao2Calc.attachTextureLayer(
        Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbao2ResultArray, 0,
        i);

    // NOLINTNEXTLINE(bugprone-integer-division). This is deliberate
    shader.setFloat2Offset({Mn::Float(i % 4) + 0.5f, Mn::Float(i / 4) + 0.5f})
        .setJitter(state_->random[i])
        .bindLinearDepthTexture(state_->hbao2DepthArray, i)
        .draw(state_->triangle);
  }
#endif

  if (state_->configuration.flags() & HbaoFlag::NoBlur) {
    output.bind();
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
    Mn::GL::Renderer::setBlendFunction(
        Mn::GL::Renderer::BlendFunction::Zero,
        Mn::GL::Renderer::BlendFunction::SourceColor,
        Mn::GL::Renderer::BlendFunction::Zero,
        Mn::GL::Renderer::BlendFunction::One);
    // TODO Set sample mask if samples > 1

  } else {
    state_->hbaoCalc.mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0}).bind();
  }

  // Only special blur
  HbaoReinterleaveShader& reinterleaveShader =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur
          ? state_->hbao2ReinterleaveSpecialBlurShader
          : state_->hbao2ReinterleaveShader;

  reinterleaveShader.bindResultsTexture(state_->hbao2ResultArray)
      .draw(state_->triangle);

  if (!(state_->configuration.flags() & HbaoFlag::NoBlur)) {
    drawHbaoBlur(output);
  }
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  // TODO reset sample mask if ever used
}  // drawCacheAwareInternal

}  // namespace gfx_batch
}  // namespace esp
