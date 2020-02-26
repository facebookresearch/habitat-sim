// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Drawable.h"

#include <Corrade/Utility/Assert.h>

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

Drawable::Drawable(scene::SceneNode& node,
                   Magnum::GL::AbstractShaderProgram& shader,
                   Magnum::GL::Mesh& mesh,
                   DrawableGroup* group /* = nullptr */)
    : Magnum::SceneGraph::Drawable3D{node, group},
      node_(node),
      shader_(shader),
      mesh_(mesh) {}

DrawableGroup* Drawable::drawables() {
  CORRADE_ASSERT(
      dynamic_cast<DrawableGroup*>(Magnum::SceneGraph::Drawable3D::drawables()),
      "Drawable must only be used with esp::gfx::DrawableGroup!", {});
  return static_cast<DrawableGroup*>(
      Magnum::SceneGraph::Drawable3D::drawables());
}

}  // namespace gfx
}  // namespace esp
