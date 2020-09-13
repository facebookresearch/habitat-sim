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

Magnum::Vector4 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix) {
  ASSERT(light.position.w() == 1 || light.position.w() == 0);
  const auto pos = light.position.xyz();

  bool isDirectional = light.position.w() == 0;
  Magnum::Vector4 out{0};

  switch (light.model) {
    case LightPositionModel::OBJECT:
      out.xyz() = isDirectional ? transformationMatrix.transformVector(pos)
                                : transformationMatrix.transformPoint(pos);
      break;
    case LightPositionModel::GLOBAL:
      out.xyz() = isDirectional ? cameraMatrix.transformVector(pos)
                                : cameraMatrix.transformPoint(pos);
      break;
    case LightPositionModel::CAMERA:
      out.xyz() = pos;
      break;
  }

  out.w() = light.position.w();
  return out;
}

LightSetup getLightsAtBoxCorners(const Magnum::Range3D& box,
                                 const Magnum::Color3& lightColor) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Magnum::Math::Literals;

  constexpr float w = 1;
  return LightSetup{{Magnum::Vector4(box.frontTopLeft(), w), lightColor},
                    {Magnum::Vector4(box.frontTopRight(), w), lightColor},
                    {Magnum::Vector4(box.frontBottomLeft(), w), lightColor},
                    {Magnum::Vector4(box.frontBottomRight(), w), lightColor},
                    {Magnum::Vector4(box.backTopLeft(), w), lightColor},
                    {Magnum::Vector4(box.backTopRight(), w), lightColor},
                    {Magnum::Vector4(box.backBottomLeft(), w), lightColor},
                    {Magnum::Vector4(box.backBottomRight(), w), lightColor}};
}

Magnum::Color3 getAmbientLightColor(const LightSetup& lightSetup) {
  if (lightSetup.size() == 0) {
    // We assume an empty light setup means the user wants "flat" shading,
    // meaning object ambient color should be copied directly to pixels as-is.
    // We can achieve this in the Phong shader using an ambient light color of
    // (1,1,1) and no additional light sources.
    return Magnum::Color3(1.0, 1.0, 1.0);
  } else {
    // todo: add up ambient terms from all lights in lightSetup
    // temp: hard-coded ambient light tuned for ReplicaCAD
    float ambientIntensity = 0.4;
    return Magnum::Color3(ambientIntensity, ambientIntensity, ambientIntensity);
  }
}

}  // namespace gfx
}  // namespace esp
