#ifndef ESP_GFX_BATCH_HBAO_H_
#define ESP_GFX_BATCH_HBAO_H_

#include <Corrade/Containers/Pointer.h>
#include <Magnum/Magnum.h>
#include <Magnum/GL/GL.h>
#include <Magnum/Math/Vector2.h>

namespace esp { namespace gfx_batch {

enum class HbaoFlag {
  Blur = 1 << 0,
  // TODO doesn't work
  UseAoSpecialBlur = 1 << 1,
  /* These two affect only the cache-aware variant. Only one can be set at
     most, not both */
  LayeredImageLoadStore = 1 << 2,
  LayeredGeometryShader = 1 << 3
};
typedef Corrade::Containers::EnumSet<HbaoFlag> HbaoFlags;

CORRADE_ENUMSET_OPERATORS(HbaoFlags)

class HbaoConfiguration {
  public:
    Magnum::Vector2i size() const { return size_; }

    HbaoConfiguration& setSize(const Magnum::Vector2i& size) {
      size_ = size;
      return *this;
    }

    HbaoFlags flags() const { return flags_; }

    HbaoConfiguration& setFlags(HbaoFlags flags) {
      flags_ = flags;
      return *this;
    }

    Magnum::Int samples() const { return samples_; }

    HbaoConfiguration& setSamples(Magnum::Int samples) {
      samples_ = samples;
      return *this;
    }

    Magnum::Float intensity() const { return intensity_; }

    HbaoConfiguration& setIntensity(Magnum::Float intensity) {
      intensity_ = intensity;
      return *this;
    }

    Magnum::Float bias() const { return bias_; }

    HbaoConfiguration& setBias(Magnum::Float bias) {
      bias_ = bias;
      return *this;
    }

    Magnum::Float radius() const { return radius_; }

    HbaoConfiguration& setRadius(Magnum::Float radius) {
      radius_ = radius;
      return *this;
    }

    Magnum::Float blurSharpness() const { return blurSharpness_; }

    HbaoConfiguration& setBlurSharpness(Magnum::Float blurSharpness) {
      blurSharpness_ = blurSharpness;
      return *this;
    }

  private:
    Magnum::Vector2i size_;
    HbaoFlags flags_ = HbaoFlag::Blur;
    Magnum::Int samples_ = 1;
    Magnum::Float intensity_ = 0.732f,
      bias_ = 0.05f,
      radius_ = 1.24f,
      blurSharpness_ = 10.0f;
};

class Hbao {
  public:
    // Use this to construct before a GL context is ready
    explicit Hbao(Magnum::NoCreateT) noexcept;

    explicit Hbao(const HbaoConfiguration& configuration);

    Hbao(const Hbao&) = delete;
    Hbao(Hbao&&) noexcept;

    ~Hbao();

    Hbao& operator=(const Hbao&) = delete;
    Hbao& operator=(Hbao&&) noexcept;

    void drawClassicOrthographic(const Magnum::Matrix4& projection, Magnum::GL::Texture2D& inputDepthStencil, Magnum::GL::AbstractFramebuffer& output);
    void drawClassicPerspective(const Magnum::Matrix4& projection, Magnum::GL::Texture2D& inputDepthStencil, Magnum::GL::AbstractFramebuffer& output);
    void drawCacheAwareOrthographic(const Magnum::Matrix4& projection, Magnum::GL::Texture2D& inputDepthStencil, Magnum::GL::AbstractFramebuffer& output);
    void drawCacheAwarePerspective(const Magnum::Matrix4& projection, Magnum::GL::Texture2D& inputDepthStencil, Magnum::GL::AbstractFramebuffer& output);

  private:
    void drawLinearDepth(const Magnum::Matrix4& projection, bool orthographic, Magnum::GL::Texture2D& inputDepthStencil);
    void drawHbaoBlur(Magnum::GL::AbstractFramebuffer& output);
    void drawClassicInternal(Magnum::GL::AbstractFramebuffer& output);
    void drawCacheAwareInternal(Magnum::GL::AbstractFramebuffer& output);

    struct State;
    Corrade::Containers::Pointer<State> state_;
};

}}

#endif
