// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include <utility>

#include "esp/gfx/RenderTarget.h"

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
  CORRADE_ASSERT(resolution[0] > 0 && resolution[1] > 0,
                 "VisualSensorSpec::sanityCheck(): resolution height and "
                 "width must be greater than 0", );
  CORRADE_ASSERT(
      channels > 0,
      "VisualSensorSpec::sanityCheck(): the value of the channels which is"
          << channels << "is illegal", );
  CORRADE_ASSERT(
      near > 0.0 && far > near,
      "VisualSensorSpec::sanityCheck(): the near or far plane is illegal.", );
}

bool VisualSensorSpec::operator==(const VisualSensorSpec& a) const {
  return SensorSpec::operator==(a) && resolution == a.resolution &&
         channels == a.channels && gpu2gpuTransfer == a.gpu2gpuTransfer &&
         far == a.far && near == a.near;
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

bool VisualSensor::getObservationSpace(ObservationSpace& space) {
  space.spaceType = ObservationSpaceType::Tensor;
  space.shape = {static_cast<size_t>(visualSensorSpec_->resolution[0]),
                 static_cast<size_t>(visualSensorSpec_->resolution[1]),
                 static_cast<size_t>(visualSensorSpec_->channels)};
  space.dataType = core::DataType::DT_UINT8;
  if (visualSensorSpec_->sensorType == SensorType::Semantic) {
    space.dataType = core::DataType::DT_UINT32;
  } else if (visualSensorSpec_->sensorType == SensorType::Depth) {
    space.dataType = core::DataType::DT_FLOAT;
  }
  return true;
}

void VisualSensor::readObservation(Observation& obs) {
  // Make sure we have memory
  if (buffer_ == nullptr) {
    // TODO: check if our sensor was resized and resize our buffer if needed
    ObservationSpace space;
    getObservationSpace(space);
    buffer_ = core::Buffer::create(space.shape, space.dataType);
  }
  obs.buffer = buffer_;

  // TODO: have different classes for the different types of sensors
  // TODO: do we need to flip axis?
  if (visualSensorSpec_->sensorType == SensorType::Semantic) {
    renderTarget().readFrameObjectId(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32UI, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else if (visualSensorSpec_->sensorType == SensorType::Depth) {
    renderTarget().readFrameDepth(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32F, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else {
    renderTarget().readFrameRgba(Magnum::MutableImageView2D{
        Magnum::PixelFormat::RGBA8Unorm, renderTarget().framebufferSize(),
        obs.buffer->data});
  }
}

bool VisualSensor::getObservation(sim::Simulator& sim, Observation& obs) {
  // TODO: check if sensor is valid?
  // TODO: have different classes for the different types of sensors
  //
  if (!hasRenderTarget())
    return false;

  drawObservation(sim);
  readObservation(obs);

  return true;
}

}  // namespace sensor
}  // namespace esp
