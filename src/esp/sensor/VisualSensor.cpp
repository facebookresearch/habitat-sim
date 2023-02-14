// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include <utility>

#include "esp/core/Utility.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

VisualSensorSpec::VisualSensorSpec() : SensorSpec() {
  sensorType = SensorType::Color;
}

void VisualSensorSpec::sanityCheck() const {
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

Cr::Containers::Optional<Mn::Vector2> VisualSensor::depthUnprojection() const {
  float f = visualSensorSpec_->far;
  float n = visualSensorSpec_->near;
  float d = f - n;
  // in projection matrix, two entries related to the depth are:
  // -(f+n)/(f-n), -2fn/(f-n), where f is the far plane, and n is the near
  // plane. depth parameters = 0.5 * vector(proj[2][2] - 1.0f, proj[3][2])
  return {0.5 * Mn::Vector2{-(f + n) / d - 1.0f, -2.0f * f * n / d}};
}

VisualSensor::MoveSemanticSensorNodeHelper::MoveSemanticSensorNodeHelper(
    VisualSensor& visualSensor,
    sim::Simulator& sim)
    : visualSensor_(visualSensor), sim_(sim) {
  CORRADE_INTERNAL_ASSERT(visualSensor_.specification()->sensorType ==
                          SensorType::Semantic);
  scene::SceneNode& node = visualSensor_.node();
  CORRADE_ASSERT(
      !scene::SceneGraph::isRootNode(node),
      "VisualSensor::moveSemanticSensorToSemanticSceneGraph(): the semantic "
      "sensor is attached to the root node, and thus cannot be moved.", );

  // check if the sensor is already in this semantic scene graph
  CORRADE_ASSERT(
      node.scene() != sim_.getActiveSemanticSceneGraph().getRootNode().scene(),
      "VisualSensor::MoveSemanticSensorNodeHelper::"
      "MoveSemanticSensorNodeHelper(): Cannot move the semantic sensor since "
      "it is already in the "
      "semantic scene graph. Make sure the semantic sensor is in the regular "
      "rgb scene graph to begin with.", );

  // no backup exists
  CORRADE_INTERNAL_ASSERT(semanticSensorParentNodeBackup_ == nullptr);
  CORRADE_INTERNAL_ASSERT(relativeTransformBackup_ == Cr::Containers::NullOpt);

  // back up the data
  relativeTransformBackup_ =
      core::orthonormalizeRotationShear(node.transformation());
  Mn::Matrix4 absTransform =
      core::orthonormalizeRotationShear(node.absoluteTransformation());
  semanticSensorParentNodeBackup_ =
      static_cast<scene::SceneNode*>(node.parent());

  // now, take the sensor from the current scene graph and connect it to
  // the root node of semantic scene graph, set the *correct* transformation
  node.setParent(&sim_.getActiveSemanticSceneGraph().getRootNode());
  node.setTransformation(absTransform);
}

VisualSensor::MoveSemanticSensorNodeHelper::~MoveSemanticSensorNodeHelper() {
  CORRADE_INTERNAL_ASSERT(visualSensor_.specification()->sensorType ==
                          SensorType::Semantic);

  scene::SceneNode& node = visualSensor_.node();
  CORRADE_ASSERT(
      node.scene() != sim_.getActiveSceneGraph().getRootNode().scene(),
      "VisualSensor::MoveSemanticSensorNodeHelper::"
      "~MoveSemanticSensorNodeHelper(): Cannot move the semantic sensor since "
      "it is already in the regular rgb scene graph. Did you move it manually "
      "by yourself?", );

  CORRADE_INTERNAL_ASSERT(semanticSensorParentNodeBackup_);
  CORRADE_INTERNAL_ASSERT(relativeTransformBackup_ != Cr::Containers::NullOpt);

  node.setParent(semanticSensorParentNodeBackup_);
  node.setTransformation(*relativeTransformBackup_);

  semanticSensorParentNodeBackup_ = nullptr;
  relativeTransformBackup_ = Cr::Containers::NullOpt;
}

}  // namespace sensor
}  // namespace esp
