// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#include "CubeMapCamera.h"
#include <Magnum/EigenIntegration/Integration.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

CubeMapCamera::CubeMapCamera(scene::SceneNode& node) : RenderCamera(node) {
  updateOriginalViewingMatrix();
}
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
  updateOriginalViewingMatrix();
}
CubeMapCamera& CubeMapCamera::updateOriginalViewingMatrix() {
  originalViewingMatrix_ = this->node().transformation();
  return *this;
}

CubeMapCamera& CubeMapCamera::restoreTransformation() {
  this->node().setTransformation(originalViewingMatrix_);
  return *this;
}

CubeMapCamera& CubeMapCamera::switchToFace(unsigned int cubeSideIndex) {
  CORRADE_ASSERT(cubeSideIndex < 6,
                 "CubeMapCamera::switchToFace(): the index of the cube side,"
                     << cubeSideIndex << "is illegal.",
                 *this);
  switchToFace(CubeMapCamera::cubeMapCoordinate(cubeSideIndex));
  return *this;
}

Mn::Matrix4 CubeMapCamera::getCameraLocalTransform(
    Mn::GL::CubeMapCoordinate cubeSideIndex) {
  Mn::Vector3 eye{0.0, 0.0, 0.0};
  Mn::Vector3 yUp{0.0, 1.0, 0.0};
  Mn::Vector3 zUp{0.0, 0.0, 1.0};
  // Careful: the coordinate system for cubemaps is left-handed.
  // The following implementation is based on:
  // https://www.khronos.org/opengl/wiki/Cubemap_Texture
  // check the diagram which shows how each face in the cubemap is oriented
  // relative to the cube being defined.
  switch (cubeSideIndex) {
    case Mn::GL::CubeMapCoordinate::PositiveX:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{1.0, 0.0, 0.0}, -yUp);
      break;
    case Mn::GL::CubeMapCoordinate::NegativeX:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{-1.0, 0.0, 0.0}, -yUp);
      break;
    case Mn::GL::CubeMapCoordinate::PositiveY:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 1.0, 0.0}, zUp);
      break;
    case Mn::GL::CubeMapCoordinate::NegativeY:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, -1.0, 0.0}, -zUp);
      break;
    case Mn::GL::CubeMapCoordinate::PositiveZ:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 0.0, 1.0}, -yUp);
      break;
    case Mn::GL::CubeMapCoordinate::NegativeZ:
      return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 0.0, -1.0}, -yUp);
      break;
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
}

CubeMapCamera& CubeMapCamera::switchToFace(Mn::GL::CubeMapCoordinate cubeSide) {
  this->node().setTransformation(
      originalViewingMatrix_ *
      CubeMapCamera::getCameraLocalTransform(cubeSide));
  return *this;
}

CubeMapCamera& CubeMapCamera::setProjectionMatrix(int width,
                                                  float znear,
                                                  float zfar) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;
  MagnumCamera::setProjectionMatrix(
      Mn::Matrix4::perspectiveProjection(
          90.0_degf,  // horizontal field of view angle
          1.0,        // aspect ratio (width/height)
          znear,      // z-near plane
          zfar))      // z-far plane
      .setViewport({width, width});
  return *this;
}

}  // namespace gfx
}  // namespace esp
