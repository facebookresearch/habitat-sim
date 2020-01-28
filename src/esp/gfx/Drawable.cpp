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
                   DrawableGroup* group /* = nullptr */)
    // Note: it is important to NOT pass through group to the Drawable3D
    // constructor, since this will add the drawable to the
    // underlying group twice. This is because we customize group membership
    // semantics using gfx::DrawableGroup
    : Magnum::SceneGraph::Drawable3D{node},
      node_(node),
      shader_(shader),
      mesh_(mesh) {
  if (group)
    group->add(*this);
}

}  // namespace gfx
}  // namespace esp
