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

class TextRenderer {
 public:
  /**
   * @brief Constructor
   */
  // aspectRatio = width / height
  explicit TextRenderer(float viewportAspectRatio = 1.33333f,
                        float fontSize = 20.0f,
                        int cacheSize = 1024);
  explicit TextRenderer(float viewportAspectRatio,
                        float fontSize,
                        int cacheSize,
                        const std::string& characters);

  // text location relative to the application window
  enum TextLocation {
    TOP_LEFT = 0,
    TOP_RIGHT = 1,
    NUM_LOCATION = 2,
    // TODO: more locations, e.g., BOTTOM_LEFT ...
    // Planning to support 9 locations in total
  };
  /**
   * @brief Create a renderer to render text on screen
   * @return renderer ID
   *
   * Multiple text renderers are supported. Each can have different settings
   * (text length, size, alignment, color etc.)
   */
  size_t createRenderer(float onScreenCharacterSize = 0.04f,
                        size_t glyphCount = 60,
                        TextLocation location = TOP_RIGHT);

  /**
   *  @brief Draw the text by the specified renderer (renderer ID by default: 0)
   * with specified color
   *  @return Reference to self (for method chaining)
   */
  TextRenderer& draw(const Mn::Color3& color, size_t rendererId = 0);

  /**
   *  @brief Update the aspect ratio of the viewport
   *  @return Reference to self (for method chaining)
   *  @note DO NOT forget to call it in the viewportEvent()
   */
  TextRenderer& updateAspectRatio(float viewportAspectRatio);

  /**
   * @brief Update the text to be displayed by a specified renderer (renderer ID
   * by default: 0)
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
  // multiple text renderers are supported
  std::vector<Cr::Containers::Pointer<Mn::Text::Renderer2D>> text_;
  std::vector<TextLocation> locations_;
  static constexpr float locationLookup_[NUM_LOCATION][2] = {
      {-1.0f, 1.0f},  // TOP_LEFT
      {1.0f, 1.0f},   // TOP_RIGHT
  };
};

}  // namespace gfx
}  // namespace esp
