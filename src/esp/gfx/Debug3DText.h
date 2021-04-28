// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DEBUG3DTEXT_H_
#define ESP_GFX_DEBUG3DTEXT_H_

/** @file
 * Utility for text positioned in 3D world-space. You must re-add text every
 * frame; it doesn't persist. This is intended for debugging. The API
 * prioritizes ease-of-use over maximum runtime performance.
 */

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Vector3.h>

#include <string>
#include <vector>

namespace esp {
namespace gfx {

/**
@brief todo
*/
class Debug3DText {
 public:
  // Call this every frame, any time before flushToImGui
  void addText(std::string&& text,
               const Magnum::Vector3& pos,
               const Magnum::Color4& color = Magnum::Color4(1, 1, 1, 1));

  // Call this alongside other calls to ImGui::Begin/End, between
  // imgui_.newFrame() and imgui_.drawFrame()
  void flushToImGui(const Magnum::Matrix4& camProj,
                    const Magnum::Vector2i& displaySize);

 private:
  struct TextRecord {
    const std::string text;
    Magnum::Vector3 pos;
    Magnum::Color4 color;
  };

  std::vector<TextRecord> records_;
};

}  // namespace gfx
}  // namespace esp

#endif
