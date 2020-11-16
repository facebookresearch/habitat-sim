// Copyright (c) Facebook, Inc. and its affiliates.
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

CubeMapCamera& CubeMapCamera::switchToFace(unsigned int cubeSideIndex) {
  switch (cubeSideIndex) {
    case 0:
      switchToFace(Mn::GL::CubeMapCoordinate::PositiveX);
      break;
    case 1:
      switchToFace(Mn::GL::CubeMapCoordinate::NegativeX);
      break;
    case 2:
      switchToFace(Mn::GL::CubeMapCoordinate::PositiveY);
      break;
    case 3:
      switchToFace(Mn::GL::CubeMapCoordinate::NegativeY);
      break;
    case 4:
      switchToFace(Mn::GL::CubeMapCoordinate::PositiveZ);
      break;
    case 5:
      switchToFace(Mn::GL::CubeMapCoordinate::NegativeZ);
      break;
    default:
      LOG(ERROR)
          << "Warning: CubeMapCamera::switchToFace: the index of cube side "
          << cubeSideIndex
          << " is illegal. Camera orientation stays unchanged.";
      break;
  }
  return *this;
}

CubeMapCamera& CubeMapCamera::switchToFace(Mn::GL::CubeMapCoordinate cubeSide) {
  Mn::Vector3 eye{0.0, 0.0, 0.0};
  Mn::Vector3 yUp{0.0, 1.0, 0.0};
  Mn::Vector3 zUp{0.0, 0.0, 1.0};

  auto cameraLocalTransform = [&]() {
    switch (cubeSide) {
      case Mn::GL::CubeMapCoordinate::PositiveX:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{1.0, 0.0, 0.0}, yUp);
        break;
      case Mn::GL::CubeMapCoordinate::NegativeX:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{-1.0, 0.0, 0.0}, yUp);
        break;
      case Mn::GL::CubeMapCoordinate::PositiveY:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 1.0, 0.0}, -zUp);
        break;
      case Mn::GL::CubeMapCoordinate::NegativeY:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, -1.0, 0.0}, zUp);
        break;
      case Mn::GL::CubeMapCoordinate::PositiveZ:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 0.0, 1.0}, yUp);
        break;
      case Mn::GL::CubeMapCoordinate::NegativeZ:
        return Mn::Matrix4::lookAt(eye, Mn::Vector3{0.0, 0.0, -1.0}, yUp);
        break;
      default:
        return Mn::Matrix4{Magnum::Math::IdentityInit};
        break;
    }
  };
  this->node().setTransformation(originalViewingMatrix_ *
                                 cameraLocalTransform());
  return *this;
}

CubeMapCamera& CubeMapCamera::setProjectionMatrix(int width,
                                                  float znear,
                                                  float zfar) {
  MagnumCamera::setProjectionMatrix(
      Mn::Matrix4::perspectiveProjection(
          Mn::Deg{90.0},  // horizontal field of view angle
          1.0,            // aspect ratio (width/height)
          znear,          // z-near plane
          zfar))          // z-far plane
      .setViewport(Magnum::Vector2i(width, width));
  return *this;
}

}  // namespace gfx
}  // namespace esp
