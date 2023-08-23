#include "Hbao.h"

#include <random>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Utility/Format.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/ImageFormat.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/MultisampleTexture.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/PackingBatch.h>

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxBatchShaderResources)
}

namespace esp { namespace gfx_batch {

namespace Cr = Corrade;
namespace Mn = Magnum;

enum: std::size_t {
  AoRandomTextureSize = 4,
  MaxSamples = 8,
  HbaoRandomSize = AoRandomTextureSize,
  FragmentOutputCount = 8
};

constexpr Mn::GL::Version GlslVersion = Mn::GL::Version::GL430;

class HbaoBlurShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      SourceTextureBinding = 0,
      LinearDepthTextureBinding = 1
    };

  public:
    explicit HbaoBlurShader(bool bilateral, Mn::Int preset): bilateral_{bilateral} {
      CORRADE_INTERNAL_ASSERT(!bilateral || !preset);

      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      if(bilateral) {
        frag.addSource(rs.getString("hbao/bilateralblur.frag"));
      } else {
        frag
          .addSource(Cr::Utility::format("#define AO_BLUR_PRESET {}\n", preset))
          .addSource(rs.getString("hbao/hbao_blur.frag"));
      }

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());

      attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
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
    Mn::Int sharpnessUniform_ = 0,
      inverseResolutionDirectionUniform_ = 1;
    bool bilateral_;
};

// TODO replace with own
class DepthLinearizeShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      InputTextureBinding = 0
    };

  public:
    // TODO MSAA is never used, drop
    explicit DepthLinearizeShader(bool msaa): msaa_{msaa} {
      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      frag
        .addSource(Cr::Utility::format("#define DEPTHLINEARIZE_MSAA {}\n", msaa ? 1 :0))
        .addSource(rs.getString("hbao/depthlinearize.frag"));

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());

      attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
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

    DepthLinearizeShader& bindInputTexture(Mn::GL::MultisampleTexture2D& texture) {
      CORRADE_INTERNAL_ASSERT(msaa_);
      texture.bind(InputTextureBinding);
      return *this;
    }

  private:
    Mn::Int clipInfoUniform_ = 0,
      sampleIndexUniform_ = 1;
    bool msaa_;
};

class ViewNormalShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      LinearDepthTextureBinding = 0
    };

  public:
    explicit ViewNormalShader() {
      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      frag.addSource(rs.getString("hbao/viewnormal.frag"));

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());

      attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
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
    Mn::Int projectionInfoUniform_ = 0,
      projectionOrthographicUniform_ = 1,
      inverseFullResolutionUniform_ = 2;
};

class HbaoCalcShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      UniformBufferBinding = 0,
      LinearDepthTextureBinding = 0,
      ViewNormalTextureBinding = 1,
      RandomTextureBinding = 1,
      OutputImageBinding = 0
    };

  public:
    enum Layered {
      Off = 0,
      ImageLoadStore = 1,
      GeometryShaderPassthrough = 2
    };

    explicit HbaoCalcShader(Mn::NoCreateT): Mn::GL::AbstractShaderProgram{Mn::NoCreate} {}

    explicit HbaoCalcShader(bool deinterleaved, bool blur, Layered layered, bool textureArrayLayer): deinterleaved_{deinterleaved}, blur_{blur}, textureArrayLayer_{textureArrayLayer}, layered_{layered} {
      CORRADE_INTERNAL_ASSERT(deinterleaved || layered == Layered::Off);
      CORRADE_INTERNAL_ASSERT(deinterleaved || !textureArrayLayer);

      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader geom{Mn::NoCreate};
      if(layered == Layered::GeometryShaderPassthrough) {
        geom = Mn::GL::Shader{GlslVersion, Mn::GL::Shader::Type::Geometry};
        geom.addSource(rs.getString("hbao/fullscreenquad.geom"));
      }

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      frag
        .addSource(Cr::Utility::format(
          "#define AO_DEINTERLEAVED {}\n"
          "#define AO_BLUR {}\n"
          "#define AO_LAYERED {}\n"
          "#define AO_TEXTUREARRAY_LAYER {}\n"
          "#define AO_RANDOMTEX_SIZE {}\n",
          deinterleaved ? 1 :0,
          blur ? 1 : 0,
          Mn::Int(layered),
          textureArrayLayer ? 1 : 0,
          AoRandomTextureSize))
        .addSource(rs.getString("hbao/hbao.frag"));

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());
      if(layered == Layered::GeometryShaderPassthrough)
        CORRADE_INTERNAL_ASSERT(geom.compile());

      if(layered == Layered::GeometryShaderPassthrough)
        attachShaders({vert, geom, frag});
      else
        attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
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
      CORRADE_INTERNAL_ASSERT(!deinterleaved_ || layered_ == Layered::Off);
      texture.bind(LinearDepthTextureBinding);
      return *this;
    }

    HbaoCalcShader& bindLinearDepthTexture(Mn::GL::Texture2DArray& texture) {
      CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ != Layered::Off && !textureArrayLayer_);
      texture.bind(LinearDepthTextureBinding);
      return *this;
    }

    HbaoCalcShader& bindLinearDepthTexture(Mn::GL::Texture2DArray& texture, Mn::Int slice) {
      CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ == Layered::Off && textureArrayLayer_);
      texture.bind(LinearDepthTextureBinding);
      setUniform(linearDepthTextureSliceUniform_, Mn::Float(slice));
      return *this;
    }

    HbaoCalcShader& bindViewNormalTexture(Mn::GL::Texture2D& texture) {
      CORRADE_INTERNAL_ASSERT(deinterleaved_);
      texture.bind(ViewNormalTextureBinding);
      return *this;
    }

    HbaoCalcShader& bindRandomTexture(Mn::GL::Texture2DArray& texture, Mn::Int slice) {
      CORRADE_INTERNAL_ASSERT(!deinterleaved_);
      texture.bind(RandomTextureBinding);
      setUniform(randomSliceUniform_, Mn::Float(slice));
      return *this;
    }

    HbaoCalcShader& bindUniformBuffer(Mn::GL::Buffer& buffer) {
      buffer.bind(Mn::GL::Buffer::Target::Uniform, UniformBufferBinding);
      return *this;
    }

    HbaoCalcShader& bindOutputImage(Mn::GL::Texture2DArray& texture, Mn::Int level) {
      CORRADE_INTERNAL_ASSERT(deinterleaved_ && layered_ == Layered::ImageLoadStore);
      texture.bindImageLayered(OutputImageBinding, level, Mn::GL::ImageAccess::WriteOnly,  blur_ ? Mn::GL::ImageFormat::R16F : Mn::GL::ImageFormat::R8);
      return *this;
    }

  private:
    Mn::Int float2OffsetUniform_ = 0,
      jitterUniform_ = 1,
      linearDepthTextureSliceUniform_ = 2,
      randomSliceUniform_ = 3;
    bool deinterleaved_, blur_, textureArrayLayer_;
    Layered layered_;
};

class HbaoDeinterleaveShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      LinearDepthTextureBinding = 0,
    };

  public:
    explicit HbaoDeinterleaveShader() {
      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      frag.addSource(rs.getString("hbao/hbao_deinterleave.frag"));

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());

      attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
    }

    HbaoDeinterleaveShader& setProjectionInfo(const Mn::Vector2& uvOffset, const Mn::Vector2& inverseResolution) {
      setUniform(projectionInfoUniform_, Mn::Vector4{uvOffset.x(), uvOffset.y(), inverseResolution.x(), inverseResolution.y()});
      return *this;
    }

    HbaoDeinterleaveShader& bindLinearDepthTexture(Mn::GL::Texture2D& texture) {
      texture.bind(LinearDepthTextureBinding);
      return *this;
    }

  private:
    Mn::Int projectionInfoUniform_ = 0;
};

class HbaoReinterleaveShader: public Mn::GL::AbstractShaderProgram {
  private:
    enum: Mn::Int {
      ResultsTextureBinding = 0
    };

  public:
    explicit HbaoReinterleaveShader(bool blur) {
      Cr::Utility::Resource rs{"gfx-shaders"};

      // TODO use our own
      Mn::GL::Shader vert{GlslVersion, Mn::GL::Shader::Type::Vertex};
      vert.addSource(rs.getString("hbao/fullscreenquad.vert"));

      Mn::GL::Shader frag{GlslVersion, Mn::GL::Shader::Type::Fragment};
      frag
        .addSource(Cr::Utility::format("#define AO_BLUR {}\n", blur ? 1 :0))
        .addSource(rs.getString("hbao/hbao_reinterleave.frag"));

      CORRADE_INTERNAL_ASSERT(vert.compile());
      CORRADE_INTERNAL_ASSERT(frag.compile());

      attachShaders({vert, frag});
      CORRADE_INTERNAL_ASSERT(link());

      // TODO query uniforms, set texture bindings for older versions
    }

    HbaoReinterleaveShader& bindResultsTexture(Mn::GL::Texture2DArray& texture) {
      texture.bind(ResultsTextureBinding);
      return *this;
    }
};

struct HbaoUniformData {
  Mn::Float radiusToScreen;
  Mn::Float r2;
  Mn::Float negInvR2;
  Mn::Float nDotVBias;

  Mn::Vector2 invFullResolution;
  Mn::Vector2 invQuarterResolution;

  Mn::Float aoMultiplier;
  Mn::Float powExponent;
  Mn::Int:32;
  Mn::Int:32;

  Mn::Vector4 projInfo;

  Mn::Vector2 projScale;
  Mn::Int projOrtho;
  Mn::Int _pad1;

  Mn::Vector4 float2Offsets[AoRandomTextureSize*AoRandomTextureSize];
  Mn::Vector4 jitters[AoRandomTextureSize*AoRandomTextureSize];
};

static_assert(sizeof(HbaoUniformData) % 4 == 0, "not a nice uniform struct");

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

  HbaoBlurShader bilateralBlurShader{/*bilateral*/ true, 0};
  HbaoBlurShader hbaoBlurShader{/*bilateral*/ false, /*preset*/ 0};
  HbaoBlurShader hbaoBlur2Shader{/*bilateral*/ false, /*preset*/ 1};
  DepthLinearizeShader depthLinearizeShader{false};
  DepthLinearizeShader depthLinearizeShaderMsaa{true};
  ViewNormalShader viewNormalShader;
  HbaoCalcShader hbaoCalcShader{/*deinterleaved*/ false, /*blur*/ false, {}, false};
  HbaoCalcShader hbaoCalcBlurShader{/*deinterleaved*/ false, /*blur*/ true, {}, false};
  /* These depend on config, are created in the constructor */
  HbaoCalcShader hbao2CalcShader{Mn::NoCreate};
  HbaoCalcShader hbao2CalcBlurShader{Mn::NoCreate};
  HbaoDeinterleaveShader hbao2DeinterleaveShader;
  HbaoReinterleaveShader hbao2ReinterleaveShader{/*blur*/ false};
  HbaoReinterleaveShader hbao2ReinterleaveBlurShader{/*blur*/ true};

  Mn::GL::Buffer hbaoUniform{Mn::GL::Buffer::TargetHint::Uniform};
  HbaoUniformData hbaoUniformData;

  Mn::GL::Mesh triangle, triangleLayered;

  HbaoConfiguration configuration;

  Mn::Vector4 random[HbaoRandomSize*HbaoRandomSize*MaxSamples];
};

Hbao::Hbao(Mn::NoCreateT) noexcept {}

Hbao::Hbao(const HbaoConfiguration& configuration) {
  if (!Cr::Utility::Resource::hasGroup("gfx-shaders"))
    importShaderResources();

  state_.emplace();

  // TODO probably most of the config should be at runtime, not setup time
  //  though size definitely can't be at setup time unless there's a way to
  //  recreate all framebuffers
  CORRADE_INTERNAL_ASSERT(!configuration.size().isZero());

  /* Only one of these can be set */
  CORRADE_INTERNAL_ASSERT(
    !(configuration.flags() & HbaoFlag::LayeredGeometryShaderPassthrough) ||
    !(configuration.flags() & HbaoFlag::LayeredImageLoadStore));

  /* "init misc" */
  {
    std::mt19937 rmt;

    float numDir = 8;  // keep in sync to glsl

    for (std::size_t i = 0; i < HbaoRandomSize*HbaoRandomSize*MaxSamples; i++) {
      // TODO what the fuck, you people can't use a float random engine instead??
      float rand1 = Mn::Float(rmt()) / 4294967296.0f;
      float rand2 = Mn::Float(rmt()) / 4294967296.0f;

      // Use random rotation angles in [0,2PI/NUM_DIRECTIONS)
      Mn::Rad angle{Mn::Constants::tau()*rand1/numDir};
      state_->random[i] = {
        Mn::Math::cos(angle),
        Mn::Math::sin(angle),
        rand2,
        0.0f
      };
    }

    /* Pack those to 16-bit and upload to a texture */
    Mn::Vector4us hbaoRandomShort[HbaoRandomSize*HbaoRandomSize*MaxSamples];
    Mn::Math::packInto(
      Cr::Containers::arrayCast<2, Mn::Float>(Cr::Containers::stridedArrayView(state_->random)),
      Cr::Containers::arrayCast<2, Mn::Short>(Cr::Containers::stridedArrayView(hbaoRandomShort)));

    constexpr Mn::Vector3i size{Mn::Vector2i{HbaoRandomSize}, MaxSamples};

    state_->hbaoRandom
      .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setStorage(1, Mn::GL::TextureFormat::RGBA16Snorm, size)
      .setSubImage(0, {}, Mn::ImageView3D{Mn::PixelFormat::RGBA16Snorm, size, hbaoRandomShort});

    // TODO is it really dynamic?! i don't think so
    state_->hbaoUniform.setData({nullptr, sizeof(HbaoUniformData)}, Mn::GL::BufferUsage::DynamicDraw);
  }

  /* "init framebuffers" */
  {
    state_->sceneDepthLinear
      .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, configuration.size());

    state_->depthLinear = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->depthLinear.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, state_->sceneDepthLinear, 0);

    state_->sceneViewNormal
      .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::RGBA8, configuration.size());

    state_->viewNormal = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->viewNormal.attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, state_->sceneViewNormal, 0);

    const Mn::GL::TextureFormat aoFormat =
      state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur ?
      Mn::GL::TextureFormat::RG16F : Mn::GL::TextureFormat::R8;
    state_->hbaoResult
      // TODO filter?!
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, aoFormat, configuration.size());
    state_->hbaoBlur
      // TODO filter?!
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, aoFormat, configuration.size());
    if(state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur) {
      state_->hbaoResult.setSwizzle<'r', 'g', '0', '0'>();
      state_->hbaoBlur.setSwizzle<'r', 'g', '0', '0'>();
    } else {
      state_->hbaoResult.setSwizzle<'r', 'r', 'r', 'r'>();
      state_->hbaoBlur.setSwizzle<'r', 'r', 'r', 'r'>();
    }

    state_->hbaoCalc = Mn::GL::Framebuffer{{{}, configuration.size()}};
    state_->hbaoCalc
      .attachTexture(Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbaoResult, 0)
      .attachTexture(Mn::GL::Framebuffer::ColorAttachment{1}, state_->hbaoBlur, 0);

    const Mn::Vector2i quarterSize = (configuration.size() + Mn::Vector2i{3})/4;

    state_->hbao2DepthArray
      .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::R32F, {quarterSize, HbaoRandomSize*HbaoRandomSize});

    state_->hbao2ResultArray
      .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, aoFormat, {quarterSize, HbaoRandomSize*HbaoRandomSize});

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

    if(configuration.flags() & HbaoFlag::LayeredImageLoadStore)
      state_->hbao2Calc.setDefaultSize(quarterSize);
    else if(configuration.flags() & HbaoFlag::LayeredGeometryShaderPassthrough)
      state_->hbao2Calc.attachLayeredTexture(Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbao2ResultArray, 0);
  }

  {
    HbaoCalcShader::Layered layered;
    bool textureArrayLayer;
    if(configuration.flags() & HbaoFlag::LayeredImageLoadStore) {
      layered = HbaoCalcShader::Layered::ImageLoadStore;
      textureArrayLayer = false;
    } else if(configuration.flags() & HbaoFlag::LayeredGeometryShaderPassthrough) {
      layered = HbaoCalcShader::Layered::GeometryShaderPassthrough;
      textureArrayLayer = false;
    } else {
      layered = HbaoCalcShader::Layered::Off;
      textureArrayLayer = true;
    }
    state_->hbao2CalcShader = HbaoCalcShader{/*deinterleaved*/ true, /*blur*/ false,
      layered, textureArrayLayer};
    state_->hbao2CalcBlurShader = HbaoCalcShader{/*deinterleaved*/ true, /*blur*/ true,
      layered, textureArrayLayer};
  }

  state_->triangle.setCount(3);
  state_->triangleLayered.setCount(3*HbaoRandomSize*HbaoRandomSize);
  state_->configuration = configuration;
}

Hbao::Hbao(Hbao&&) noexcept = default;

Hbao::~Hbao() = default;

Hbao& Hbao::operator=(Hbao&&) noexcept = default;

namespace {

void prepareHbaoData(const HbaoConfiguration& configuration, const Mn::Matrix4& projection, HbaoUniformData& uniformData, Mn::GL::Buffer& uniform, Cr::Containers::StaticArrayView<HbaoRandomSize*HbaoRandomSize*MaxSamples, const Mn::Vector4> random) {
  /* Radius */
  const Mn::Float meters2viewspace = 1.0f;
  const Mn::Float r = configuration.radius()*meters2viewspace;
  uniformData.r2 = r*r;
  uniformData.negInvR2 = -1.0f / uniformData.r2;
  /* For perspective, projection[1][1] is 1/tan(fov/2) */
  const Mn::Float projectionScale = 2*configuration.size().y()*projection[1][1];
  uniformData.radiusToScreen = r*0.5f*projectionScale;

  /* AO */
  uniformData.powExponent = Mn::Math::max(configuration.intensity(), 0.0f);
  uniformData.nDotVBias = Mn::Math::clamp(configuration.bias(), 0.0f, 1.0f);
  uniformData.aoMultiplier = 1.0f/(1.0f - uniformData.nDotVBias);

  /* Resolution */
  const Mn::Vector2i quarterSize = (configuration.size() + Mn::Vector2i{3})/4;
  uniformData.invQuarterResolution = Mn::Vector2{1.0f}/Mn::Vector2{quarterSize};
  uniformData.invFullResolution = Mn::Vector2{1.0f}/Mn::Vector2{configuration.size()};

  if(configuration.flags() & (HbaoFlag::LayeredGeometryShaderPassthrough|HbaoFlag::LayeredImageLoadStore)) for(Mn::Int i = 0; i != HbaoRandomSize*HbaoRandomSize; ++i) {
    uniformData.float2Offsets[i] = {
      Mn::Float(i % 4) + 0.5f,
      Mn::Float(i / 4) + 0.5f,
      0.0f,
      0.0f,
    };
    uniformData.jitters[i] = random[i];
  }

  uniform.setSubData(0, {&uniformData, 1});
}

void prepareHbaoDataPerspective(const HbaoConfiguration& configuration, const Mn::Matrix4& projection, HbaoUniformData& uniformData, Mn::GL::Buffer& uniform, Cr::Containers::StaticArrayView<HbaoRandomSize*HbaoRandomSize*MaxSamples, const Mn::Vector4> random) {
  uniformData.projInfo = {
    2.0f/projection[0][0],
    2.0f/projection[1][1],
    -(1.0f - projection[2][0])/projection[0][0],
    -(1.0f + projection[2][1])/projection[1][1],
  };
  uniformData.projOrtho = false;
  prepareHbaoData(configuration, projection, uniformData, uniform, random);
}

void prepareHbaoDataOrthographic(const HbaoConfiguration& configuration, const Mn::Matrix4& projection, HbaoUniformData& uniformData, Mn::GL::Buffer& uniform, Cr::Containers::StaticArrayView<HbaoRandomSize*HbaoRandomSize*MaxSamples, const Mn::Vector4> random) {
  uniformData.projInfo = {
    2.0f/projection[0][0],
    2.0f/projection[1][1],
    -(1.0f + projection[3][0])/projection[0][0],
    -(1.0f - projection[3][1])/projection[1][1],
  };
  uniformData.projOrtho = true;
  prepareHbaoData(configuration, projection, uniformData, uniform, random);
}

// TODO upstream!!
Mn::Float near(const Mn::Matrix4& projectionMatrix) {
  return projectionMatrix[3][2]/(projectionMatrix[2][2] - 1.0f);
}

Mn::Float far(const Mn::Matrix4& projectionMatrix) {
  return projectionMatrix[3][2]/(projectionMatrix[2][2] + 1.0f);
}

}

void Hbao::drawLinearDepth(const Mn::Matrix4& projection, bool orthographic, Mn::GL::Texture2D& depthStencilInput) {
  state_->depthLinear.bind();

  state_->depthLinearizeShader
    .setClipInfo({
      // TODO this still looks like there's some better way this should be extracted
      near(projection)*far(projection),
      near(projection) - far(projection),
      far(projection), orthographic ? 0.0f : 1.0f
    })
    .bindInputTexture(depthStencilInput)
    .draw(state_->triangle);
}

void Hbao::drawHbaoBlur(Mn::GL::AbstractFramebuffer& output) {
  constexpr Mn::Float meters2viewspace = 1.0f;

  state_->hbaoCalc
    .mapForDraw(Mn::GL::Framebuffer::ColorAttachment{1})
    .bind();

  HbaoBlurShader& shader =
    state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur ?
    state_->hbaoBlurShader : state_->bilateralBlurShader;
  shader
    .setSharpness(state_->configuration.blurSharpness()/meters2viewspace)
    .setInverseResolutionDirection({1.0f/state_->configuration.size().x(), 0.0f})
    .bindSourceTexture(state_->hbaoResult);
  if(!(state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur))
    shader.bindLinearDepthTexture(state_->sceneDepthLinear);
  shader.draw(state_->triangle);

  output.bind();
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
  Mn::GL::Renderer::setBlendFunction(Mn::GL::Renderer::BlendFunction::Zero,
                                     Mn::GL::Renderer::BlendFunction::SourceColor);

  HbaoBlurShader& secondShader =
    state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur ?
    state_->hbaoBlur2Shader : state_->bilateralBlurShader;
  secondShader
    .setSharpness(state_->configuration.blurSharpness()/meters2viewspace)
    .setInverseResolutionDirection({0.0f, 1.0f/state_->configuration.size().y()})
    .bindSourceTexture(state_->hbaoBlur);
  if(!(state_->configuration.flags() & HbaoFlag::UseAoSpecialBlur))
    shader.bindLinearDepthTexture(state_->sceneDepthLinear);
  shader.draw(state_->triangle);
}

void Hbao::drawClassicOrthographic(const Mn::Matrix4& projection, Mn::GL::Texture2D& depthStencilInput, Mn::GL::AbstractFramebuffer& output) {
  // TODO this absolutely does not need to be redone every time
  prepareHbaoDataOrthographic(state_->configuration, projection, state_->hbaoUniformData, state_->hbaoUniform, state_->random);
  drawLinearDepth(projection, true, depthStencilInput);
  drawClassicInternal(output);
}

void Hbao::drawClassicPerspective(const Mn::Matrix4& projection, Mn::GL::Texture2D& depthStencilInput, Mn::GL::AbstractFramebuffer& output) {
  // TODO this absolutely does not need to be redone every time
  prepareHbaoDataPerspective(state_->configuration, projection, state_->hbaoUniformData, state_->hbaoUniform, state_->random);
  drawLinearDepth(projection, false, depthStencilInput);
  drawClassicInternal(output);
}

void Hbao::drawClassicInternal(Mn::GL::AbstractFramebuffer& output) {
  if(state_->configuration.flags() & HbaoFlag::Blur) {
    state_->hbaoCalc
      .mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0})
      .bind();
  } else {
    output.bind();
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
    Mn::GL::Renderer::setBlendFunction(Mn::GL::Renderer::BlendFunction::Zero,
                                       Mn::GL::Renderer::BlendFunction::SourceColor);
  }

  HbaoCalcShader& shader = state_->configuration.flags() >= (HbaoFlag::Blur|HbaoFlag::UseAoSpecialBlur) ?
    state_->hbaoCalcBlurShader : state_->hbaoCalcShader;
  shader
    .bindLinearDepthTexture(state_->sceneDepthLinear)
    .bindRandomTexture(state_->hbaoRandom, 0)
    .bindUniformBuffer(state_->hbaoUniform)
    .draw(state_->triangle);

  if(state_->configuration.flags() & HbaoFlag::Blur)
    drawHbaoBlur(output);

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  // TODO reset sample mask if ever used
}

void Hbao::drawCacheAwareOrthographic(const Mn::Matrix4& projection, Mn::GL::Texture2D& depthStencilInput, Mn::GL::AbstractFramebuffer& output) {
  // TODO this absolutely does not need to be redone every time
  prepareHbaoDataOrthographic(state_->configuration, projection, state_->hbaoUniformData, state_->hbaoUniform, state_->random);
  drawLinearDepth(projection, true, depthStencilInput);
  drawCacheAwareInternal(output);
}

void Hbao::drawCacheAwarePerspective(const Mn::Matrix4& projection, Mn::GL::Texture2D& depthStencilInput, Mn::GL::AbstractFramebuffer& output) {
  // TODO this absolutely does not need to be redone every time
  prepareHbaoDataPerspective(state_->configuration, projection, state_->hbaoUniformData, state_->hbaoUniform, state_->random);
  drawLinearDepth(projection, false, depthStencilInput);
  drawCacheAwareInternal(output);
}

void Hbao::drawCacheAwareInternal(Mn::GL::AbstractFramebuffer& output) {
  state_->viewNormal.bind();

  state_->viewNormalShader
    .setProjectionInfo(state_->hbaoUniformData.projInfo)
    .setProjectionOrthographic(state_->hbaoUniformData.projOrtho)
    .setInverseFullResolution(state_->hbaoUniformData.invFullResolution)
    .bindLinearDepthTexture(state_->sceneDepthLinear)
    .draw(state_->triangle);

  state_->hbao2Deinterleave.bind();
  state_->hbao2DeinterleaveShader
    .bindLinearDepthTexture(state_->sceneDepthLinear);

  for(Mn::Int i = 0; i != HbaoRandomSize*HbaoRandomSize; i += FragmentOutputCount) {
    for(Mn::UnsignedInt layer = 0; layer != FragmentOutputCount; ++layer)
      state_->hbao2Deinterleave.attachTextureLayer(Mn::GL::Framebuffer::ColorAttachment{layer}, state_->hbao2DepthArray, 0, i + layer);

    state_->hbao2DeinterleaveShader
      .setProjectionInfo({Mn::Float(i % 4) + 0.5f,
                Mn::Float(i / 4) + 0.5f},
               state_->hbaoUniformData.invFullResolution)
      .draw(state_->triangle);
  }

  state_->hbao2Calc.bind();

  HbaoCalcShader& shader = state_->configuration.flags() >= (HbaoFlag::Blur|HbaoFlag::UseAoSpecialBlur) ?
    state_->hbao2CalcBlurShader : state_->hbao2CalcShader;

  shader
    .bindViewNormalTexture(state_->sceneViewNormal)
    .bindUniformBuffer(state_->hbaoUniform);

  if(state_->configuration.flags() & (HbaoFlag::LayeredGeometryShaderPassthrough|HbaoFlag::LayeredImageLoadStore)) {
    shader.bindLinearDepthTexture(state_->hbao2DepthArray);
    if(state_->configuration.flags() & HbaoFlag::LayeredImageLoadStore)
      shader.bindOutputImage(state_->hbao2ResultArray, 0);
    shader.draw(state_->triangleLayered);
    if(state_->configuration.flags() & HbaoFlag::LayeredImageLoadStore)
      Mn::GL::Renderer::setMemoryBarrier(Mn::GL::Renderer::MemoryBarrier::TextureFetch|Mn::GL::Renderer::MemoryBarrier::ShaderImageAccess);
  } else {
    for(Mn::Int i = 0; i != HbaoRandomSize*HbaoRandomSize; ++i) {
      state_->hbao2Calc.attachTextureLayer(Mn::GL::Framebuffer::ColorAttachment{0}, state_->hbao2ResultArray, 0, i);
      shader
        .setFloat2Offset({Mn::Float(i % 4) + 0.5f,
                          Mn::Float(i / 4) + 0.5f})
        .setJitter(state_->random[i])
        .bindLinearDepthTexture(state_->hbao2DepthArray, i)
        .draw(state_->triangle);
    }
  }

  if(state_->configuration.flags() & HbaoFlag::Blur) {
    state_->hbaoCalc
      .mapForDraw(Mn::GL::Framebuffer::ColorAttachment{0})
      .bind();
  } else {
    output.bind();
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
    Mn::GL::Renderer::setBlendFunction(Mn::GL::Renderer::BlendFunction::Zero,
                                       Mn::GL::Renderer::BlendFunction::SourceColor);
  }

  HbaoReinterleaveShader& reinterleaveShader =
    state_->configuration.flags() >= (HbaoFlag::Blur|HbaoFlag::UseAoSpecialBlur) ?
    state_->hbao2ReinterleaveBlurShader : state_->hbao2ReinterleaveShader;

  reinterleaveShader
    .bindResultsTexture(state_->hbao2ResultArray)
    .draw(state_->triangle);

  if(state_->configuration.flags() & HbaoFlag::Blur)
    drawHbaoBlur(output);

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  // TODO reset sample mask if ever used
}

}}
