// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"

#include <utility>

#include "esp/gfx/DepthVisualizerShader.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/SensorInfoVisualizer.h"

namespace esp {
namespace sensor {
VisualSensor::VisualSensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Sensor{node, std::move(spec)}, tgt_{nullptr} {}

VisualSensor::~VisualSensor() = default;

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

void VisualSensor::visualizeObservation(gfx::SensorInfoVisualizer& visualizer,
                                        float depthScaling) {
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
          .setDepthScaling(depthScaling);
      // draw to the framebuffer
      visualizer.draw(shader.key());
    } break;

    default:
      // Have to had this default, otherwise the clang-tidy will be pissed off,
      // and will not let me pass
      break;
  }
}

bool VisualSensor::displayObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  drawObservation(sim);
  renderTarget().blitRgbaToDefault();

  return true;
}

}  // namespace sensor
}  // namespace esp
