// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MeshVisualizerDrawable.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

MeshVisualizerDrawable::MeshVisualizerDrawable(
    scene::SceneNode& node,
    Magnum::Shaders::MeshVisualizer3D* shader,
    Magnum::GL::Mesh& mesh,
    DrawableGroup* group)
    : Drawable{node, mesh, group}, shader_(shader) {}
}  // namespace gfx
}  // namespace esp
