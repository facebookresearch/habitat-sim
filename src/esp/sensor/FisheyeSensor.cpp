// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "FisheyeSensor.h"
#include "esp/core/Check.h"
#include "esp/gfx/DoubleSphereCameraShader.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace sensor {

FisheyeSensorSpec::FisheyeSensorSpec() : CubeMapSensorBaseSpec() {
  uuid = "fisheye";
  sensorSubType = SensorSubType::Fisheye;
}

void FisheyeSensorSpec::sanityCheck() const {
  CubeMapSensorBaseSpec::sanityCheck();
  ESP_CHECK(sensorSubType == SensorSubType::Fisheye,
            "FisheyeSensorSpec::sanityCheck(): sensor sub-type is not Fisheye");

  ESP_CHECK(focalLength[0] > 0 && focalLength[1] > 0,
            "FisheyeSensorSpec::sanityCheck(): focal length," << focalLength
                                                              << "is illegal.");
}

void FisheyeSensorDoubleSphereSpec::sanityCheck() const {
  FisheyeSensorSpec::sanityCheck();
  ESP_CHECK(fisheyeModelType == FisheyeSensorModelType::DoubleSphere,
            "FisheyeSensorDoubleSphereSpec::sanityCheck(): fisheye model "
            "type is wrong. It should be DoubleSphere.");
  ESP_CHECK(alpha >= 0 && alpha <= 1.0,
            "FisheyeSensorDoubleSphereSpec::sanityCheck(): alpha"
                << alpha << "is illegal.");
}

template <typename T>
void specSanityCheck(FisheyeSensorSpec* spec) {
  auto* actualSpec = dynamic_cast<T*>(spec);
  ESP_CHECK(actualSpec,
            "FisheyeSensor::FisheyeSensor(): the spec cannot be converted "
            "to any known fiesheye sensor spec.");
  actualSpec->sanityCheck();
}

Magnum::Vector2 computePrincipalPointOffset(const FisheyeSensorSpec& spec) {
  if (bool(spec.principalPointOffset)) {
    return *spec.principalPointOffset;
  }
  auto res = spec.resolution.cast<float>();
  return Mn::Vector2(res[0], res[1]) * 0.5f;
}

FisheyeSensor::FisheyeSensor(scene::SceneNode& cameraNode,
                             const FisheyeSensorSpec::ptr& spec)
    : CubeMapSensorBase(cameraNode, spec) {
  switch (fisheyeSensorSpec_->fisheyeModelType) {
    case FisheyeSensorModelType::DoubleSphere: {
      specSanityCheck<FisheyeSensorDoubleSphereSpec>(fisheyeSensorSpec_.get());
    } break;
  };
}

bool FisheyeSensorSpec::operator==(const FisheyeSensorSpec& a) const {
  return CubeMapSensorBaseSpec::operator==(a) &&
         fisheyeModelType == a.fisheyeModelType &&
         focalLength == a.focalLength &&
         principalPointOffset == a.principalPointOffset;
}

Mn::ResourceKey FisheyeSensor::getShaderKey() {
  return Cr::Utility::formatString(
      FISH_EYE_SHADER_KEY_TEMPLATE,
      static_cast<Mn::UnsignedInt>(fisheyeSensorSpec_->fisheyeModelType),
      static_cast<gfx::CubeMapShaderBase::Flags::UnderlyingType>(
          cubeMapShaderBaseFlags_));
}

bool FisheyeSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  renderToCubemapTexture(sim);

  switch (fisheyeSensorSpec_->fisheyeModelType) {
    case FisheyeSensorModelType::DoubleSphere: {
      Magnum::Resource<gfx::CubeMapShaderBase, gfx::DoubleSphereCameraShader>
          shader = getShader<gfx::DoubleSphereCameraShader>();
      auto& actualSpec =
          static_cast<FisheyeSensorDoubleSphereSpec&>(*fisheyeSensorSpec_);
      (*shader)
          .setFocalLength(actualSpec.focalLength)
          .setPrincipalPointOffset(computePrincipalPointOffset(actualSpec))
          .setAlpha(actualSpec.alpha)
          .setXi(actualSpec.xi);
      drawWith(*shader);
    } break;

      // TODO:
      // The other FisheyeSensorModelType

    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }

  return true;
}

}  // namespace sensor
}  // namespace esp
