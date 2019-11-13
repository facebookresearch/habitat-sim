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
  explicit TextRenderer(float viewportAspectRatio = 1.33333f,
                        float fontSize = 20.0f,
                        int cacheSize = 512,
                        const std::string& characters = defaultCharacters);

  /**
   * @brief Create a renderer to render text on screen
   * @return renderer ID
   */
  size_t createRenderer(
      float onScreenCharacterSize = 0.08f,
      size_t glyphCount = 60,
      Mn::Text::Alignment alignment = Mn::Text::Alignment::TopRight);

  /**
   *  @brief Draw the text by the specified renderer (default 0) with specified
   * color
   *  @return Reference to self (for method chaining)
   */
  TextRenderer& draw(const Mn::Color3& color, size_t rendererId = 0);

  /**
   *  @brief Update the aspect ratio of the viewport
   *  @return Reference to self (for method chaining)
   *  @notification DO NOT forget to call it in the viewportEvent()
   */
  TextRenderer& updateAspectRatio(float viewportAspectRatio);

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
  // TODO: we can build an array to support multiple fonts
  Cr::Containers::Pointer<Mn::Text::AbstractFont> font_;
  Mn::Text::GlyphCache cache_;
  Mn::Shaders::Vector2D textShader_;
  Mn::Matrix3 textProjection_;
  std::vector<Cr::Containers::Pointer<Mn::Text::Renderer2D>> text_;
};

}  // namespace gfx
}  // namespace esp
