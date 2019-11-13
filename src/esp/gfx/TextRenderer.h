// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/Containers.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Shaders/Vector.h>
#include <Magnum/Text/AbstractFont.h>
#include <Magnum/Text/Alignment.h>
#include <Magnum/Text/GlyphCache.h>
#include <Magnum/Text/Renderer.h>
#include <string>
#include <vector>

namespace esp {
namespace gfx {

namespace Cr = Corrade;
namespace Mn = Magnum;

const std::string defaultCharacters =
    "abcdefghijklmnopqrstuvwxyz"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "0123456789?!:;,.# ";

class TextRenderer {
 public:
  /**
   * @brief Constructor
   */
  // aspectRatio = width / height
  explicit TextRenderer(float viewportAspectRatio,
                        float fontSize = 20.0f,
                        const std::string& characters = defaultCharacters,
                        int cacheSize = 512);

  /**
   * @brief Create a renderer to render text on screen
   * @return renderer ID
   */
  size_t createRenderer(
      float onScreenCharacterSize = 0.08f,
      size_t glyphCount = 40,
      Mn::Text::Alignment alignment = Mn::Text::Alignment::LineLeft);

  /**
   *  @brief draw the text by a specified renderer (default 0) with specified
   * color
   *  @return Reference to self (for method chaining)
   */
  TextRenderer& draw(const Mn::Color3& color, size_t rendererId = 0);

  /**
   *  @brief update the aspect ratio of the viewport 
   *  @return Reference to self (for method chaining)
   *  @notification DO NOT forget to call it in the viewportEvent()
   */
  TextRenderer& updateAspectRatio(float viewportAspectRatio);

  // ======== texture binding ========
  /**
   * @brief Bind the atlas texture
   * @return Reference to self (for method chaining)
   */
  // TextRenderer& bindAtlasTexture(Magnum::GL::Texture2D& texture);
  /**
   *  @brief Bind the buffer texture containing the adjacent faces
   *  @return Reference to self (for method chaining)
   */
  // TextRenderer& bindAdjFacesBufferTexture(Magnum::GL::BufferTexture&
  // texture);

  // ======== set uniforms ===========
  /**
   *  @brief Set modelview and projection matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  //  TextRenderer& setMVPMatrix(cinst Magnum::Matrix4& matrix);
  /**
   *  @brief Set expsure to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  // TextRenderer& setExposure(float exposure);
  /**
   *  @brief Set gamma to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  // TextRenderer& setGamma(float gamma);
  /**
   *  @brief Set saturation to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  // TextRenderer& setSaturation(float saturation);
  /**
   *  @brief Set the tile size of the atlas texture
   *  @return Reference to self (for method chaining)
   */
  // TextRenderer& setAtlasTextureSize(Magnum::GL::Texture2D& texture,
  // uint32_t tileSize);

  /**
   * @brief Update the text to be rendered
   *
   * Initially no text is rendered.
   * @attention The capacity (glyphCount) must
   * be large enough to contain all glyphs.
   * see @ref createRenderer() for more
   * information.
   */
  void updateText(const std::string& text, size_t rendererId = 0);

 protected:
  Cr::PluginManager::Manager<Mn::Text::AbstractFont> fontManager_;
  // TODO: we can build an array to support
  // multiple fonts
  Cr::Containers::Pointer<Mn::Text::AbstractFont> font_;
  Mn::Text::GlyphCache cache_;
  Mn::Shaders::Vector2D textShader_;
  Mn::Matrix3 textProjection_;
  std::vector<Cr::Containers::Pointer<Mn::Text::Renderer2D>> text_;
};

}  // namespace gfx
}  // namespace esp
