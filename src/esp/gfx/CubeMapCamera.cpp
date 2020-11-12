// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#include "CubeMapCamera.h"
#include <Magnum/EigenIntegration/Integration.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

CubeMapCamera::CubeMapCamera(scene::SceneNode& node) : RenderCamera(node) {}
CubeMapCamera::CubeMapCamera(scene::SceneNode& node,
                             const vec3f& eye,
                             const vec3f& target,
                             const vec3f& up)
    : CubeMapCamera(node,
                    Mn::Vector3{eye},
                    Mn::Vector3{target},
                    Mn::Vector3{up}) {}
CubeMapCamera::CubeMapCamera(scene::SceneNode& node,
                             const Mn::Vector3& eye,
                             const Mn::Vector3& target,
                             const Mn::Vector3& up)
    : RenderCamera(node, eye, target, up) {
  originalViewingMatrix_ = node.transformationMatrix();
}

}  // namespace gfx
}  // namespace esp
