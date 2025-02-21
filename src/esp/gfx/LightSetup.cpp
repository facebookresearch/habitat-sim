// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightSetup.h"

#include "esp/core/Check.h"

namespace esp {
namespace gfx {

bool operator==(const LightInfo& a, const LightInfo& b) {
  return a.vector == b.vector && a.color == b.color && a.model == b.model;
}

bool operator!=(const LightInfo& a, const LightInfo& b) {
  return !(a == b);
}

Magnum::Vector4 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix) {
  ESP_CHECK(
      light.vector.w() == 1 || light.vector.w() == 0,
      "Light vector" << light.vector
                     << "is expected to have w == 0 for a directional light or "
                        "w == 1 for a point light");

  switch (light.model) {
    case LightPositionModel::Object:
      return transformationMatrix * light.vector;
    case LightPositionModel::Global:
      return cameraMatrix * light.vector;
    case LightPositionModel::Camera:
      return light.vector;
  }

  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

Magnum::Vector4 getLightPositionRelativeToWorld(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix) {
  ESP_CHECK(
      light.vector.w() == 1 || light.vector.w() == 0,
      "Light vector" << light.vector
                     << "is expected to have w == 0 for a directional light or "
                        "w == 1 for a point light");

  switch (light.model) {
    case LightPositionModel::Object:
      return cameraMatrix.inverted() * transformationMatrix * light.vector;
    case LightPositionModel::Global:
      return light.vector;
    case LightPositionModel::Camera:
      return cameraMatrix.inverted() * light.vector;
  }

  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

LightSetup getLightsAtBoxCorners(const Magnum::Range3D& box,
                                 const Magnum::Color3& lightColor) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Magnum::Math::Literals;

  constexpr float w = 1;
  return LightSetup{{{box.frontTopLeft(), w}, lightColor},
                    {{box.frontTopRight(), w}, lightColor},
                    {{box.frontBottomLeft(), w}, lightColor},
                    {{box.frontBottomRight(), w}, lightColor},
                    {{box.backTopLeft(), w}, lightColor},
                    {{box.backTopRight(), w}, lightColor},
                    {{box.backBottomLeft(), w}, lightColor},
                    {{box.backBottomRight(), w}, lightColor}};
}

LightSetup getDefaultLights() {
  // Eric U hack: custom lights to improve brightness and white balance in HSSD
  // scenes

  // light intensity; a larger value makes the scene more bright, but a
  // too-large value will give an unrealistic "blown out" appearance.
  constexpr float intensity = 1.5f;
  // a bluish-white hue to counteract a yellow tint currently seen in most
  // scenes
  Magnum::Color3 rgb_hue = {0.7, 0.7, 0.92};
  Magnum::Color3 color = rgb_hue * intensity;

  return LightSetup{
      // four lights pointing in different directions, but all pointing somewhat
      // down (light from above)
      {{0.0, -0.5, -0.4, 0.0}, color, LightPositionModel::Global},  // -z
      {{0.0, -0.5, 0.6, 0.0}, color, LightPositionModel::Global},   // +z
      {{-0.4, -0.5, 0.0, 0.0}, color, LightPositionModel::Global},  // -x
      {{0.6, -0.5, 0.0, 0.0}, color, LightPositionModel::Global},   // +x
      // additional light from below, so that the undersides of objects aren't
      // too dark
      {{0.0, 1.0, 0.0, 0.0}, color, LightPositionModel::Global},
      // feel free to add additional lights here
  };
}

Magnum::Color3 getAmbientLightColor(const LightSetup& lightSetup) {
  if (lightSetup.empty()) {
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
