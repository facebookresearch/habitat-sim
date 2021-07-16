// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#include "DepthMapDrawableBase.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

DepthMapDrawableBase::DepthMapDrawableBase(scene::SceneNode& node,
                                           Magnum::GL::Mesh* mesh,
                                           ShaderManager& shaderManager,
                                           DrawableType type,
                                           DrawableGroup* group)
    : Drawable{node, mesh, type, group}, shaderManager_(shaderManager) {}
}  // namespace gfx
}  // namespace esp
