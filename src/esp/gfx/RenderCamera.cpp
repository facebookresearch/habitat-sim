// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderCamera.h"

#include <Magnum/EigenIntegration/Integration.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

RenderCamera::RenderCamera(scene::SceneNode& node) : MagnumCamera{node} {
  node.setType(scene::SceneNodeType::CAMERA);
  setAspectRatioPolicy(Mn::SceneGraph::AspectRatioPolicy::NotPreserved);
}

RenderCamera::RenderCamera(scene::SceneNode& node,
                           const vec3f& eye,
                           const vec3f& target,
                           const vec3f& up)
    : RenderCamera(node) {
  // once it is attached, set the transformation
  node.setTransformation(Mn::Matrix4::lookAt(
      Mn::Vector3{eye}, Mn::Vector3{target}, Mn::Vector3{up}));
}

RenderCamera& RenderCamera::setProjectionMatrix(int width,
                                                int height,
                                                float znear,
                                                float zfar,
                                                float hfov) {
  const float aspectRatio = static_cast<float>(width) / height;
  MagnumCamera::setProjectionMatrix(
      Mn::Matrix4::perspectiveProjection(Mn::Deg{hfov}, aspectRatio, znear,
                                         zfar))
      .setViewport(Magnum::Vector2i(width, height));
  return *this;
}

RenderCamera& RenderCamera::draw(MagnumDrawableGroup& drawables) {
  // TODO: visibility culling
  MagnumCamera::draw(drawables);
  return *this;
}

}  // namespace gfx
}  // namespace esp
