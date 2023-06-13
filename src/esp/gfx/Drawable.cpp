// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Drawable.h"
#include <Corrade/Utility/Assert.h>
#include "DrawableGroup.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
uint64_t Drawable::drawableIdCounter = 0;
Drawable::Drawable(scene::SceneNode& node,
                   Magnum::GL::Mesh* mesh,
                   DrawableType type,
                   DrawableGroup* group /* = nullptr */)
    : Magnum::SceneGraph::Drawable3D{node, group},
      type_(type),
      node_(node),
      drawableId_(drawableIdCounter++),
      mesh_(mesh) {
  if (group) {
    group->registerDrawable(*this);
  }
}

Drawable::~Drawable() {
  DrawableGroup* group = drawables();
  if (group) {
    group->unregisterDrawable(*this);
  }
}

DrawableGroup* Drawable::drawables() {
  auto* group = Magnum::SceneGraph::Drawable3D::drawables();
  if (!group) {
    return nullptr;
  }
  CORRADE_ASSERT(dynamic_cast<DrawableGroup*>(group),
                 "Drawable must only be used with esp::gfx::DrawableGroup!",
                 {});
  return static_cast<DrawableGroup*>(group);
}
}  // namespace gfx
}  // namespace esp
