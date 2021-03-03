// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "esp/gfx/DepthVisualizerShader.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/SensorInfoVisualizer.h"

namespace esp {
namespace sensor {

VisualSensorSpec::VisualSensorSpec() : SensorSpec() {
  sensorType = SensorType::Color;
}

void VisualSensorSpec::sanityCheck() {
  SensorSpec::sanityCheck();
  bool isVisualSensor =
      (sensorType == SensorType::Color || sensorType == SensorType::Depth ||
       sensorType == SensorType::Normal || sensorType == SensorType::Semantic);
  CORRADE_ASSERT(
      isVisualSensor,
      "VisualSensorSpec::sanityCheck(): sensorType must be Color, Depth, "
      "Normal, or Semantic", );
  if (noiseModel == "Gaussian" || noiseModel == "Poisson" ||
      noiseModel == "SaltAndPepper" || noiseModel == "Speckle") {
    CORRADE_ASSERT(
        sensorType == SensorType::Color,
        "VisualSensorSpec::sanityCheck(): sensorType must be Color if "
        "noiseModel is Gaussian, Poisson, SaltAndPepper, or Speckle", );
  }
  if (noiseModel == "Redwood") {
    CORRADE_ASSERT(
        sensorType == SensorType::Depth,
        "VisualSensorSpec::sanityCheck(): sensorType must be Depth if "
        "noiseModel is Redwood", );
  }
  CORRADE_ASSERT(
      ortho_scale > 0,
      "VisualSensorSpec::sanityCheck(): ortho_scale must be greater than 0", );
  CORRADE_ASSERT(resolution[0] > 0 && resolution[1] > 0,
                 "VisualSensorSpec::sanityCheck(): resolution height and "
                 "width must be greater than 0", );
  CORRADE_ASSERT(!encoding.empty(),
                 "VisualSensorSpec::sanityCheck(): encoding is unitialized", );
}

bool VisualSensorSpec::operator==(const VisualSensorSpec& a) const {
  return SensorSpec::operator==(a) && ortho_scale == a.ortho_scale &&
         resolution == a.resolution && encoding == a.encoding &&
         gpu2gpuTransfer == a.gpu2gpuTransfer;
}

VisualSensor::VisualSensor(scene::SceneNode& node, VisualSensorSpec::ptr spec)
    : Sensor{node, std::move(spec)}, tgt_{nullptr} {
  CORRADE_ASSERT(
      visualSensorSpec_,
      "VisualSensor::VisualSensor(): The input sensorSpec is illegal", );
  visualSensorSpec_->sanityCheck();
}

VisualSensor::~VisualSensor() {
  LOG(INFO) << "Deconstructing VisualSensor";
}

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

bool VisualSensor::displayObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }
  drawObservation(sim);
  renderTarget().blitRgbaToDefault();
  return true;
}

void VisualSensor::visualizeObservation(gfx::SensorInfoVisualizer& visualizer) {
  auto sensorType = this->spec_->sensorType;
  CORRADE_ASSERT(
      sensorType == SensorType::Depth,
      "CameraSensor::visualizeObservation: sensor type is not supported.", );

  // prepare: setup and clear framebuffer
  visualizer.prepareToDraw(framebufferSize());
  switch (sensorType) {
    case SensorType::Depth: {
      // setup the shader
      Mn::Resource<Mn::GL::AbstractShaderProgram, gfx::DepthVisualizerShader>
          shader = visualizer.getShader<gfx::DepthVisualizerShader>(
              gfx::SensorInfoType::Depth);
      shader->bindDepthTexture(renderTarget().getDepthTexture())
          .setDepthUnprojection(*depthUnprojection())
          .setDepthScaling(50.0);
      // draw to the framebuffer
      visualizer.draw(shader.key());
    } break;

    default:
      // Have to had this default, otherwise the clang-tidy will be pissed off,
      // and will not let me pass
      break;
  }
}

}  // namespace sensor
}  // namespace esp
