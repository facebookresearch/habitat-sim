// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Drawable.h"

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

Drawable::Drawable(scene::SceneNode& node,
                   Magnum::GL::AbstractShaderProgram& shader,
                   Magnum::GL::Mesh& mesh,
                   Magnum::SceneGraph::DrawableGroup3D* group /* = nullptr */)
    : Magnum::SceneGraph::Drawable3D{node, group},
      node_(node),
      shader_(shader),
      mesh_(mesh) {}

}  // namespace gfx
}  // namespace esp
