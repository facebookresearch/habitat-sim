// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DrawableGroup.h"

#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {

DrawableGroup& DrawableGroup::add(esp::gfx::Drawable& drawable) {
  DrawableGroup* prevGroup = DrawableGroupClient::getGroup(drawable);
  if (prevGroup)
    prevGroup->remove(drawable);
  Magnum::SceneGraph::DrawableGroup3D::add(drawable);
  DrawableGroupClient::setGroup(drawable, this);
  return *this;
}

DrawableGroup& DrawableGroup::remove(esp::gfx::Drawable& drawable) {
  CORRADE_ASSERT(DrawableGroupClient::getGroup(drawable) == this,
                 "DrawableGroup::remove: drawable is not part of this group",
                 *this);
  Magnum::SceneGraph::DrawableGroup3D::remove(drawable);
  DrawableGroupClient::setGroup(drawable, nullptr);
  return *this;
}

bool DrawableGroup::prepareForDraw(const RenderCamera& camera) {
  return shader_ && shader_->prepareForDraw(*this, camera);
}

}  // namespace gfx
}  // namespace esp
