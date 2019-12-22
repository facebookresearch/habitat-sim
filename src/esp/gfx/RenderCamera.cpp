// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderCamera.h"

#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/Math/Intersection.h>
#include <Magnum/Math/Range.h>
#include <Magnum/SceneGraph/Drawable.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

RenderCamera::RenderCamera(scene::SceneNode& node)
    : Mn::SceneGraph::AbstractFeature3D{node} {
  node.setType(scene::SceneNodeType::CAMERA);
  camera_ = new MagnumCamera(node);
  camera_->setAspectRatioPolicy(
      Mn::SceneGraph::AspectRatioPolicy::NotPreserved);
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

void RenderCamera::setProjectionMatrix(int width,
                                       int height,
                                       float znear,
                                       float zfar,
                                       float hfov) {
  const float aspectRatio = static_cast<float>(width) / height;
  camera_
      ->setProjectionMatrix(Mn::Matrix4::perspectiveProjection(
          Mn::Deg{hfov}, aspectRatio, znear, zfar))
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

uint32_t RenderCamera::draw(MagnumDrawableGroup& drawables) {
  uint32_t numDrawnObjects = drawables.size();
  if (!frustumCulling) {
    camera_->draw(drawables);
  } else {
    // camera frustum relative to world origin
    const Mn::Math::Frustum<float> frustum =
        Mn::Math::Frustum<float>::fromMatrix(camera_->projectionMatrix() *
                                             camera_->cameraMatrix());

    // erase all items that have absolute aabb but don't pass the frustum check
    std::vector<std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>>
        drawableTransformations = camera_->drawableTransformations(drawables);
    drawableTransformations.erase(
        std::remove_if(
            drawableTransformations.begin(), drawableTransformations.end(),
            [&](const std::pair<
                std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                Mn::Matrix4>& a) {
              // obtain the absolute aabb
              Corrade::Containers::Optional<Mn::Range3D> aabb =
                  dynamic_cast<scene::SceneNode&>(a.first.get().object())
                      .getAbsoluteAABB();
              if (aabb) {
                // if it has an absolute aabb, it is a static mesh
                return !Mn::Math::Intersection::rangeFrustum(*aabb, frustum);
              } else {
                // keep the drawable if its node does not have an absolute AABB
                return true;
              }
            }),
        drawableTransformations.end());

    // draw just the visible part
    camera_->draw(drawableTransformations);
    numDrawnObjects = drawableTransformations.size();
  }

  return numDrawnObjects;
}

}  // namespace gfx
}  // namespace esp
