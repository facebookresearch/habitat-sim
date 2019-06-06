// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderCamera.h"

#include <Magnum/EigenIntegration/Integration.h>

using namespace Magnum;

namespace esp {
namespace gfx {

RenderCamera::RenderCamera(scene::SceneNode& node)
    : scene::AttachedObject{node, scene::AttachedObjectType::CAMERA} {
  camera_ = new MagnumCamera(node);
}

RenderCamera::RenderCamera(scene::SceneNode& node,
                           const vec3f& eye,
                           const vec3f& target,
                           const vec3f& up)
    : RenderCamera(node) {
  // once it is attached, set the transformation
  node.setTransformation(
      Matrix4::lookAt(Vector3{eye}, Vector3{target}, Vector3{up}));
}

void RenderCamera::setProjectionMatrix(int width,
                                       int height,
                                       float znear,
                                       float zfar,
                                       float hfov) {
  const float aspectRatio = static_cast<float>(width) / height;
  camera_->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::NotPreserved)
      .setProjectionMatrix(
          Matrix4::perspectiveProjection(Deg{hfov}, aspectRatio, znear, zfar))
      .setViewport(Magnum::Vector2i(width, height));
}

mat4f RenderCamera::getProjectionMatrix() {
  return Eigen::Map<mat4f>(camera_->projectionMatrix().data());
}

mat4f RenderCamera::getCameraMatrix() {
  return Eigen::Map<mat4f>(camera_->cameraMatrix().data());
}

MagnumCamera& RenderCamera::getMagnumCamera() {
  return *camera_;
}

void RenderCamera::draw(MagnumDrawableGroup& drawables) {
  camera_->draw(drawables);
}

}  // namespace gfx
}  // namespace esp
