// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BATCH_HBAO_H_
#define ESP_GFX_BATCH_HBAO_H_

#include <Corrade/Containers/Pointer.h>
#include <Magnum/GL/GL.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector2.h>

namespace esp {
namespace gfx_batch {

enum class HbaoFlag {
  /**
   * Default should be blur, since without blur there are artifacts. No blur
   * capability given for debugging.
   */
  NoBlur = 1 << 0,
  UseAoSpecialBlur = 1 << 1,
  /* These two affect only the cache-aware variant. Only one can be set at
     most, not both (mutually exclusive) */
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

  // Blur should always be used, otherwise there's nasty grid-like artifacts.
  // NoBlur available for debugging.
  HbaoConfiguration& setNoBlur(bool state) {
    // special blur is dependent on blur being enabled, so both should be
    // disabled if NoBlur is enabled.
    flags_ = (state ? (flags_ | HbaoFlag::NoBlur) & ~HbaoFlag::UseAoSpecialBlur
                    : flags_ & ~HbaoFlag::NoBlur);
    return *this;
  }

  HbaoConfiguration& setUseSpecialBlur(bool state) {
    // special blur is dependent on blur being enabled, so NoBlur should be
    // disabled if special blur is enabled
    flags_ = (state ? (flags_ | HbaoFlag::UseAoSpecialBlur) & ~HbaoFlag::NoBlur
                    : flags_ & ~HbaoFlag::UseAoSpecialBlur);
    return *this;
  }

  HbaoConfiguration& setUseLayeredImageLoadStore(bool state) {
    // LayeredImageLoadStore and LayeredGeometryShader are mutually exclusive
    flags_ = (state ? (flags_ | HbaoFlag::LayeredImageLoadStore) &
                          ~HbaoFlag::LayeredGeometryShader
                    : flags_ & ~HbaoFlag::LayeredImageLoadStore);
    return *this;
  }
  HbaoConfiguration& setUseLayeredGeometryShader(bool state) {
    // LayeredImageLoadStore and LayeredGeometryShader are mutually exclusive
    flags_ = (state ? (flags_ | HbaoFlag::LayeredGeometryShader) &
                          ~HbaoFlag::LayeredImageLoadStore
                    : flags_ & ~HbaoFlag::LayeredGeometryShader);
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
  HbaoConfiguration& scaleIntensity(Magnum::Float mult) {
    intensity_ *= mult;
    return *this;
  }

  Magnum::Float bias() const { return bias_; }

  HbaoConfiguration& setBias(Magnum::Float bias) {
    bias_ = bias;
    return *this;
  }
  HbaoConfiguration& scaleBias(Magnum::Float mult) {
    bias_ *= mult;
    return *this;
  }

  Magnum::Float radius() const { return radius_; }

  HbaoConfiguration& setRadius(Magnum::Float radius) {
    radius_ = radius;
    return *this;
  }
  HbaoConfiguration& scaleRadius(Magnum::Float mult) {
    radius_ *= mult;
    return *this;
  }

  Magnum::Float blurSharpness() const { return blurSharpness_; }

  HbaoConfiguration& setBlurSharpness(Magnum::Float blurSharpness) {
    blurSharpness_ = blurSharpness;
    return *this;
  }
  HbaoConfiguration& scaleBlurSharpness(Magnum::Float mult) {
    blurSharpness_ *= mult;
    return *this;
  }

 private:
  Magnum::Vector2i size_;
  HbaoFlags flags_{};
  Magnum::Int samples_ = 1;
  Magnum::Float intensity_ = 0.732f, bias_ = 0.05f, radius_ = 1.84f,
                blurSharpness_ = 10.0f;
};  // class HbaoConfiguration

enum class HbaoType { Classic, CacheAware };

class Hbao {
 public:
  explicit Hbao(const HbaoConfiguration& configuration);

  Hbao(const Hbao&) = delete;
  Hbao(Hbao&&) noexcept;

  ~Hbao();

  Hbao& operator=(const Hbao&) = delete;
  Hbao& operator=(Hbao&&) noexcept;

  /**
   * @brief Set the configurable quantities of the HBAO algorithm based on user
   * settings and defaults.
   */
  void setConfiguration(const HbaoConfiguration& configuration);

  /**
   * @brief Draw the HBAO effect on top of the current framebuffer.
   * @param projection The current visual sensor's projection matrix.
   * perspective projection matrix.
   * @param algType Either the classic algorithm or the cache-aware algorithm
   * The cache-aware algorithm has performance optimizations.
   * @param inputDepthStencil The owning RenderTarget's depthRenderTexture
   * @param output The owning RenderTarget's framebuffer the effect is to be
   * written to.
   */
  void drawEffect(const Magnum::Matrix4& projection,
                  HbaoType algType,
                  Magnum::GL::Texture2D& inputDepthStencil,
                  Magnum::GL::AbstractFramebuffer& output);

  /**
   * @brief Retrieve the size of the framebuffer used to build the components of
   * the HBAO algorithms.
   */
  Magnum::Vector2i getFrameBufferSize() const;

 private:
  void drawLinearDepth(const Magnum::Matrix4& projection,
                       Magnum::GL::Texture2D& inputDepthStencil);
  void drawHbaoBlur(Magnum::GL::AbstractFramebuffer& output);
  void drawClassicInternal(Magnum::GL::AbstractFramebuffer& output);
  void drawCacheAwareInternal(Magnum::GL::AbstractFramebuffer& output);

  struct State;
  Corrade::Containers::Pointer<State> state_;
};

}  // namespace gfx_batch
}  // namespace esp

#endif
