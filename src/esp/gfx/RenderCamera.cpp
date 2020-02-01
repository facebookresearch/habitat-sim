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

size_t RenderCamera::cull(
    std::vector<std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>>& drawableTransforms) {
  // camera frustum relative to world origin
  const Mn::Frustum frustum =
      Mn::Frustum::fromMatrix(projectionMatrix() * cameraMatrix());

  auto newEndIter = std::remove_if(
      drawableTransforms.begin(), drawableTransforms.end(),
      [&](const std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
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
          return false;
        }
      });

  return (newEndIter - drawableTransforms.begin());
}

uint32_t RenderCamera::draw(MagnumDrawableGroup& drawables,
                            bool frustumCulling) {
  if (!frustumCulling) {
    MagnumCamera::draw(drawables);
    return drawables.size();
  }

  std::vector<std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                        Mn::Matrix4>>
      drawableTransforms = drawableTransformations(drawables);

  // draw just the visible part
  size_t numVisibles = cull(drawableTransforms);
  // erase all items that did not pass the frustum visibility test
  drawableTransforms.erase(drawableTransforms.begin() + numVisibles,
                           drawableTransforms.end());

  MagnumCamera::draw(drawableTransforms);
  return drawableTransforms.size();
}

}  // namespace gfx
}  // namespace esp
