// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightSetup.h"

namespace esp {
namespace gfx {

bool operator==(const LightInfo& a, const LightInfo& b) {
  return a.position == b.position && a.color == b.color && a.model == b.model;
}

bool operator!=(const LightInfo& a, const LightInfo& b) {
  return !(a == b);
}

Magnum::Vector3 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix) {
  switch (light.model) {
    case LightPositionModel::OBJECT:
      return transformationMatrix.transformPoint(light.position);
    case LightPositionModel::GLOBAL:
      return cameraMatrix.transformPoint(light.position);
    case LightPositionModel::CAMERA:
      return light.position;
  }

  CORRADE_ASSERT_UNREACHABLE();
  return {};
}

LightSetup getLightsAtBoxCorners(const Magnum::Range3D& box,
                                 const Magnum::Color4& lightColor) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Magnum::Math::Literals;

  return LightSetup{
      {box.frontTopLeft(), lightColor},    {box.frontTopRight(), lightColor},
      {box.frontBottomLeft(), lightColor}, {box.frontBottomRight(), lightColor},
      {box.backTopLeft(), lightColor},     {box.backTopRight(), lightColor},
      {box.backBottomLeft(), lightColor},  {box.backBottomRight(), lightColor}};
}

}  // namespace gfx
}  // namespace esp
