// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "EquirectangularSensor.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/gfx/EquirectangularShader.h"
#include "esp/sim/Simulator.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace sensor {

EquirectangularSensorSpec::EquirectangularSensorSpec() {
  uuid = "equirectangular";
  sensorSubType = SensorSubType::Equirectangular;
}

void EquirectangularSensorSpec::sanityCheck() const {
  CubeMapSensorBaseSpec::sanityCheck();
  CORRADE_ASSERT(
      sensorSubType == SensorSubType::Equirectangular,
      "EquirectangularSensorSpec::sanityCheck(): sensor sub-type is not "
      "Equirectangular", );
}

EquirectangularSensor::EquirectangularSensor(
    scene::SceneNode& cameraNode,
    const EquirectangularSensorSpec::ptr& spec)
    : CubeMapSensorBase(cameraNode, spec) {
  equirectangularSensorSpec_->sanityCheck();
}

bool EquirectangularSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }
  renderToCubemapTexture(sim);
  Magnum::Resource<gfx::CubeMapShaderBase, gfx::EquirectangularShader> shader =
      getShader<gfx::EquirectangularShader>();

  (*shader).setViewportSize(equirectangularSensorSpec_->resolution);
  drawWith(*shader);

  return true;
}

Mn::ResourceKey EquirectangularSensor::getShaderKey() {
  return Cr::Utility::formatString(
      EQUIRECTANGULAR_SHADER_KEY_TEMPLATE,
      static_cast<gfx::CubeMapShaderBase::Flags::UnderlyingType>(
          cubeMapShaderBaseFlags_));
}

}  // namespace sensor
}  // namespace esp
