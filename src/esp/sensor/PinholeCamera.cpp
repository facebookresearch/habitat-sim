// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PinholeCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/Simulator.h"

namespace esp {
namespace sensor {

PinholeCamera::PinholeCamera(scene::SceneNode& pinholeCameraNode,
                             sensor::SensorSpec::ptr spec)
    : sensor::Sensor(pinholeCameraNode, spec) {
  setProjectionParameters(spec);
}

void PinholeCamera::setProjectionParameters(SensorSpec::ptr spec) {
  ASSERT(spec != nullptr);
  width_ = spec_->resolution[1];
  height_ = spec_->resolution[0];
  near_ = std::atof(spec_->parameters.at("near").c_str());
  far_ = std::atof(spec_->parameters.at("far").c_str());
  hfov_ = std::atof(spec_->parameters.at("hfov").c_str());
}

void PinholeCamera::setProjectionMatrix(gfx::RenderCamera& targetCamera) {
  targetCamera.setProjectionMatrix(width_, height_, near_, far_, hfov_);
}

bool PinholeCamera::getObservationSpace(ObservationSpace& space) {
  space.spaceType = ObservationSpaceType::TENSOR;
  space.shape = {static_cast<size_t>(spec_->resolution[0]),
                 static_cast<size_t>(spec_->resolution[1]),
                 static_cast<size_t>(spec_->channels)};
  space.dataType = core::DataType::DT_UINT8;
  if (spec_->sensorType == SensorType::SEMANTIC) {
    space.dataType = core::DataType::DT_UINT32;
  } else if (spec_->sensorType == SensorType::DEPTH) {
    space.dataType = core::DataType::DT_FLOAT;
  }
  return true;
}

bool PinholeCamera::getObservation(gfx::Simulator& sim, Observation& obs) {
  // TODO: check if sensor is valid?
  // TODO: have different classes for the different types of sensors
  //
  if (!hasRenderTarget())
    return false;

  renderTarget()->renderEnter();

  // Make sure we have memory
  if (buffer_ == nullptr) {
    // TODO: check if our sensor was resized and resize our buffer if needed
    ObservationSpace space;
    getObservationSpace(space);
    buffer_ = core::Buffer::create(space.shape, space.dataType);
  }
  obs.buffer = buffer_;

  // TODO: Get appropriate render with correct resolution
  std::shared_ptr<gfx::Renderer> renderer = sim.getRenderer();
  if (spec_->sensorType == SensorType::SEMANTIC) {
    // TODO: check sim has semantic scene graph
    renderer->draw(*this, sim.getActiveSemanticSceneGraph());
  } else {
    // SensorType is DEPTH or any other type
    renderer->draw(*this, sim.getActiveSceneGraph());
  }

  // TODO: have different classes for the different types of sensors
  // TODO: do we need to flip axis?
  if (spec_->sensorType == SensorType::SEMANTIC) {
    renderer->readFrameObjectId((uint32_t*)buffer_->data);
  } else if (spec_->sensorType == SensorType::DEPTH) {
    renderer->readFrameDepth((float*)buffer_->data);
  } else {
    renderer->readFrameRgba((uint8_t*)buffer_->data);
  }

  renderTarget()->renderExit();

  return true;
}

}  // namespace sensor
}  // namespace esp
