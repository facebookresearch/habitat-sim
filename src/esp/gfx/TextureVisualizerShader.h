// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_TEXTUREVISUALIZERSHADER_H_
#define ESP_GFX_TEXTUREVISUALIZERSHADER_H_
#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Shaders/GenericGL.h>

namespace esp {
namespace gfx {
/**
@brief A shader to visualize the depth buffer information
*/
class TextureVisualizerShader : public Magnum::GL::AbstractShaderProgram {
 public:
  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,
  };

  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * visualize depth texture
     */
    DepthTexture = 1 << 0,
    /**
     * visualize object-id texture (semantic info)
     */
    ObjectIdTexture = 1 << 1,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /** @brief Constructor */
  explicit TextureVisualizerShader(Flags flags);

  /**
   * @brief Build the colorMapTexture_ based on the passed @p colorMap of
   * colors.
   *
   * @param colorMap The map of colors to use
   * @param sampleWrapHandling Whether queries exceeding the length of @p
   * colorMap will use the final color, or will wrap around on the texture.
   * @param filterType Type of filter to use for when pxl size is incompatible
   * with texture size. Options are @ref Magnum::GL::SamplerFilter::Nearest or
   * @ref Magnum::GL::SamplerFilter::Linear.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& setColorMapTexture(
      Corrade::Containers::ArrayView<const Magnum::Vector3ub> colorMap,
      float offset,
      float scale,
      Magnum::GL::SamplerWrapping sampleWrapHandling,
      Magnum::GL::SamplerFilter filterType);

  /**
   * @brief
   * Offset and scale applied to each input pixel value from depth texture or
   * the ObjectId texture, resulting value is then used to fetch a color from a
   * color map bound with bindColorMapTexture(). Here we use either the Magnum
   * built-in color map: Magnum::DebugTools::ColorMap::turbo(), or a synthesized
   * map based on semantic scene descriptor-specified vertex colors;
   *
   * Default Value (set in the constructor):
   * For depth texture, the initial value is 1.0f/512.0f and 1.0 / 1000.f.
   * For object id texture, the initial value is 1.0f/512.0f and 1.0/256.0f,
   * meaning that for a 256-entry colormap the first 256 values get an exact
   * color from it and the next values will be either clamped to last color or
   * repeated depending on the color map texture wrapping mode (here we use
   * "repeat.")
   *
   * Expects that Flag::DepthTexture or Flag::ObjectIdTexture is enabled.
   *
   * In depth texture visualization, the scale (1 / dis) is to adjust overall
   * intensity of the final visual result. Ideally it is the furthest distance
   * in the scene. In practice, user can set e.g., the "dis" less than or equal
   * to the far plane.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& setColorMapTransformation(float offset, float scale);

  /**
   * @brief Bind depth texture. It can only be called when Flag::DepthTexture is
   * set.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& bindDepthTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind object Id texture. It can only be called when
   * Flag::ObjectIdTexture is set.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& bindObjectIdTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Set the depth unprojection parameters directly. It can only be
   * called when Flag::DepthTexture is set.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& setDepthUnprojection(
      const Magnum::Vector2& depthUnprojection);

  /**
   * @brief rebind internal color map texture.
   * NOTE: this must be called when shaders have been switched.
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& rebindColorMapTexture();

 protected:
  Flags flags_;

  Magnum::GL::Texture2D colorMapTexture_;
  GLint colorMapOffsetScaleUniform_ = -1;
  GLint depthUnprojectionUniform_ = -1;
};
CORRADE_ENUMSET_OPERATORS(TextureVisualizerShader::Flags)
}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_TEXTUREVISUALIZERSHADER_H_
