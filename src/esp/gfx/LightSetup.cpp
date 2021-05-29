// Copyright (c) Facebook, Inc. and its affiliates.
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
  // XXX debug purpose
  return LightSetup{
      // position or direction
      {{10.0f, 10.0f, 10.0f, 0.0},
       // color with intensity
       // {5.0, 0.0, 0.0},
       {2.5, 2.5, 2.5},
       // position model
       LightPositionModel::Camera},  // Key light
      // LightPositionModel::Object},  // Key light
      // LightPositionModel::Global},  // Key light
      {{-5.0f, -5.0f, 10.0f, 0.0},
       //{0.0, 5.0, 0.0},
       {1.6, 1.6, 1.6},
       LightPositionModel::Camera},  // Fill light
      // LightPositionModel::Object},  // Fill light
      // LightPositionModel::Global},  // Fill light
      {{0.0f, 10.0f, -10.0f, 0.0},
       {0.2, 0.2, 0.2},
       //{0.0, 0.0, 5.0},
       LightPositionModel::Camera},  // Rim light
      // LightPositionModel::Object},  // Rim light
      // LightPositionModel::Global},  // Rim light
  };
  /*
  return LightSetup{
      // position or direction
      {{10.0f, 10.0f, 10.0f, 0.0},
       // color with intensity
       {1.25, 1.25, 1.25},
       // position model
       LightPositionModel::Camera},  // Key light
      {{-5.0f, -5.0f, 10.0f, 0.0},
       {0.8, 0.8, 0.8},
       LightPositionModel::Camera},  // Fill light
      {{0.0f, 10.0f, -10.0f, 0.0},
       {0.1, 0.1, 0.1},
       LightPositionModel::Camera},  // Rim light
  };
  */
  return LightSetup{{{1.0, 1.0, 0.0, 0.0}, {0.75, 0.75, 0.75}},
                    {{-0.5, 0.0, 1.0, 0.0}, {0.4, 0.4, 0.4}}};
}  // namespace gfx

LightSetup getDefaultThreePointLights() {
  return LightSetup{
      // position or direction
      {{10.0f, 10.0f, 10.0f, 0.0},
       // color with intensity
       {2.5, 2.5, 2.5},
       // position model
       LightPositionModel::Camera},  // Key light
      {{-5.0f, -5.0f, 10.0f, 0.0},
       {1.6, 1.6, 1.6},
       LightPositionModel::Camera},  // Fill light
      {{0.0f, 10.0f, -10.0f, 0.0},
       {0.2, 0.2, 0.2},
       LightPositionModel::Camera},  // Rim light
  };
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

void printOutLightSetup(const LightSetup& lightSetup) {
  LOG(INFO) << "light number: " << lightSetup.size();
  for (size_t iLight = 0; iLight < lightSetup.size(); ++iLight) {
    const auto& c = lightSetup[iLight].color;
    LOG(INFO) << "light " << iLight << ": ";
    LOG(INFO) << "color = " << c.r() << ", " << c.g() << ", " << c.b();
    const auto& v = lightSetup[iLight].vector;
    LOG(INFO) << "light vector = " << v.r() << ", " << v.g() << ", " << v.b()
              << ", " << v.a();
  }
}

}  // namespace gfx
}  // namespace esp
