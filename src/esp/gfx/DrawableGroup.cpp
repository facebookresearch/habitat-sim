// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DrawableGroup.h"

#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {

bool DrawableGroup::prepareForDraw(const RenderCamera& camera) {
  if (!shader_) {
    return false;
  }
  shader_->prepareForDraw(camera);
  return true;
}

}  // namespace gfx
}  // namespace esp
