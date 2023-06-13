// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderCamera.h"

#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/Math/Intersection.h>
#include <Magnum/Math/Range.h>
#include <Magnum/SceneGraph/Drawable.h>
#include "esp/scene/SceneGraph.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

/**
 * @brief do frustum culling with temporal coherence
 * @param range, the axis-aligned bounding box
 * @param frustum, the frustum
 * @param frustumPlaneIndex, the frustum plane in last frame that culled the
 * aabb (default: 0)
 * @return NullOpt if aabb intersects the frustum, otherwise the frustum plane
 * that culls the aabb
 */
Cr::Containers::Optional<int> rangeFrustum(const Mn::Range3D& range,
                                           const Mn::Frustum& frustum,
                                           int frustumPlaneIndex = 0) {
  const Mn::Vector3 center = range.min() + range.max();
  const Mn::Vector3 extent = range.max() - range.min();

  for (int iPlane = 0; iPlane < 6; ++iPlane) {
    int index = (iPlane + frustumPlaneIndex) % 6;
    const Mn::Vector4& plane = frustum[index];

    const Mn::Vector3 absPlaneNormal = Mn::Math::abs(plane.xyz());

    const float d = Mn::Math::dot(center, plane.xyz());
    const float r = Mn::Math::dot(extent, absPlaneNormal);
    if (d + r < -2.0f * plane.w())
      return Cr::Containers::Optional<int>{index};
  }

  return Cr::Containers::NullOpt;
}

RenderCamera::RenderCamera(scene::SceneNode& node) : MagnumCamera{node} {
  node.setType(scene::SceneNodeType::CAMERA);
  setAspectRatioPolicy(Mn::SceneGraph::AspectRatioPolicy::NotPreserved);
}

RenderCamera::RenderCamera(scene::SceneNode& node,
                           const Mn::Vector3& eye,
                           const Mn::Vector3& target,
                           const Mn::Vector3& up)

    : RenderCamera(node) {
  // once it is attached, set the transformation
  resetViewingParameters(eye, target, up);
}

RenderCamera::RenderCamera(scene::SceneNode& node,
                           const vec3f& eye,
                           const vec3f& target,
                           const vec3f& up)
    : RenderCamera(node,
                   Mn::Vector3{eye},
                   Mn::Vector3{target},
                   Mn::Vector3{up}) {}

RenderCamera& RenderCamera::resetViewingParameters(const Mn::Vector3& eye,
                                                   const Mn::Vector3& target,
                                                   const Mn::Vector3& up) {
  this->node().setTransformation(Mn::Matrix4::lookAt(eye, target, up));
  return *this;
}

bool RenderCamera::isInSceneGraph(const scene::SceneGraph& sceneGraph) {
  return (this->node().scene() == sceneGraph.getRootNode().parent());
}

RenderCamera& RenderCamera::setProjectionMatrix(int width,
                                                int height,
                                                float znear,
                                                float zfar,
                                                Mn::Deg hfov) {
  const float aspectRatio = static_cast<float>(width) / height;
  auto projMat =
      Mn::Matrix4::perspectiveProjection(hfov, aspectRatio, znear, zfar);
  return setProjectionMatrix(width, height, projMat);
}

RenderCamera& RenderCamera::setOrthoProjectionMatrix(int width,
                                                     int height,
                                                     float znear,
                                                     float zfar,
                                                     float scale) {
  auto size = Mn::Vector2{width / (1.0f * height), 1.0f};
  size /= scale;
  auto orthoMat = Mn::Matrix4::orthographicProjection(size, znear, zfar);

  return setProjectionMatrix(width, height, orthoMat);
}

size_t RenderCamera::cull(DrawableTransforms& drawableTransforms) {
  // camera frustum relative to world origin
  const Mn::Frustum frustum =
      Mn::Frustum::fromMatrix(projectionMatrix() * cameraMatrix());

  auto newEndIter = std::remove_if(
      drawableTransforms.begin(), drawableTransforms.end(),
      [&](const std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>& a) {
        // obtain the absolute aabb
        auto& node = static_cast<scene::SceneNode&>(a.first.get().object());
        // This updates the AABB for dynamic objects if needed
        node.setClean();
        const Mn::Range3D& aabb = node.getAbsoluteAABB();

        Cr::Containers::Optional<int> culledPlane =
            rangeFrustum(aabb, frustum, node.getFrustumPlaneIndex());
        if (culledPlane) {
          node.setFrustumPlaneIndex(*culledPlane);
        }
        // if it has value, it means the aabb is culled
        return (culledPlane != Cr::Containers::NullOpt);
      });

  return (newEndIter - drawableTransforms.begin());
}

size_t RenderCamera::removeNonObjects(DrawableTransforms& drawableTransforms) {
  auto newEndIter = std::remove_if(
      drawableTransforms.begin(), drawableTransforms.end(),
      [&](const std::pair<std::reference_wrapper<Mn::SceneGraph::Drawable3D>,
                          Mn::Matrix4>& a) {
        auto& node = static_cast<scene::SceneNode&>(a.first.get().object());
        // don't remove OBJECT types
        return (node.getType() != scene::SceneNodeType::OBJECT);
      });
  return (newEndIter - drawableTransforms.begin());
}

uint32_t RenderCamera::draw(DrawableTransforms& drawableTransforms,
                            Flags flags) {
  previousNumVisibleDrawables_ = drawableTransforms.size();

  if (flags & Flag::UseDrawableIdAsObjectId) {
    useDrawableIds_ = true;
  }

  MagnumCamera::draw(drawableTransforms);

  if (useDrawableIds_) {
    useDrawableIds_ = false;
  }

  return drawableTransforms.size();
}

uint32_t RenderCamera::draw(MagnumDrawableGroup& drawables, Flags flags) {
  auto drawableTransforms = drawableTransformations(drawables);
  filterTransforms(drawableTransforms, flags);
  return draw(drawableTransforms, flags);
}

size_t RenderCamera::filterTransforms(DrawableTransforms& drawableTransforms,
                                      Flags flags) {
  if (flags & Flag::UseDrawableIdAsObjectId) {
    useDrawableIds_ = true;
  }

  if (flags & Flag::ObjectsOnly) {
    // draw just the OBJECTS
    size_t numObjects = removeNonObjects(drawableTransforms);
    drawableTransforms.erase(drawableTransforms.begin() + numObjects,
                             drawableTransforms.end());
  }

  if (flags & Flag::FrustumCulling) {
    // draw just the visible part
    previousNumVisibleDrawables_ = cull(drawableTransforms);
    // erase all items that did not pass the frustum visibility test
    drawableTransforms.erase(
        drawableTransforms.begin() + previousNumVisibleDrawables_,
        drawableTransforms.end());
  }

  if (useDrawableIds_) {
    useDrawableIds_ = false;
  }
  return drawableTransforms.size();
}

esp::geo::Ray RenderCamera::unproject(const Mn::Vector2i& viewportPosition,
                                      bool normalized) {
  esp::geo::Ray ray;
  ray.origin = object().absoluteTranslation();

  const Magnum::Vector2i viewPos{viewportPosition.x(),
                                 viewport().y() - viewportPosition.y() - 1};

  const Magnum::Vector3 normalizedPos{
      2 * Magnum::Vector2{viewPos} / Magnum::Vector2{viewport()} -
          Magnum::Vector2{1.0f},
      1.0};

  // compute the far plane distance
  auto farDistance =
      projectionMatrix()[3][2] / (1.0f + projectionMatrix()[2][2]);

  ray.direction =
      ((object().absoluteTransformationMatrix() * invertedProjectionMatrix)
           .transformPoint(normalizedPos) -
       ray.origin) /
      farDistance;

  if (normalized) {
    ray.direction = ray.direction.normalized();
  }
  return ray;
}

}  // namespace gfx
}  // namespace esp
