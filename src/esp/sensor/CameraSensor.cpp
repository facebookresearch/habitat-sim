// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/ImageView.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/PixelFormat.h>

#include "CameraSensor.h"
#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/Renderer.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

void CameraSensorSpec::sanityCheck() {
  VisualSensorSpec::sanityCheck();
  CORRADE_ASSERT(sensorSubType == SensorSubType::Pinhole ||
                     sensorSubType == SensorSubType::Orthographic,
                 "CameraSensor::CameraSensorSpec(): sensorSpec does not have "
                 "SensorSubType "
                 "Pinhole or Orthographic", );
}

CameraSensorSpec::CameraSensorSpec()
    : VisualSensorSpec(), channels(4), observationSpace("") {
  uuid = "rgba_camera";
  sensorSubType = SensorSubType::Pinhole;
}

CameraSensor::CameraSensor(scene::SceneNode& cameraNode,
                           const CameraSensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec),
      baseProjMatrix_(Magnum::Math::IdentityInit),
      zoomMatrix_(Magnum::Math::IdentityInit) {
  // Sanity check
  cameraSensorSpec_->sanityCheck();
  CORRADE_ASSERT(
      spec->sensorSubType == SensorSubType::Pinhole ||
          spec->sensorSubType == SensorSubType::Orthographic,
      "CameraSensor::CameraSensor(): The sensor sub-type is unknown", );
  // Initialize renderCamera_ first to avoid segfaults
  renderCamera_ = new gfx::RenderCamera(cameraNode);
  setProjectionParameters(*spec);
  renderCamera_->setAspectRatioPolicy(
      Mn::SceneGraph::AspectRatioPolicy::Extend);
  recomputeProjectionMatrix();
  renderCamera_->setViewport(this->framebufferSize());
}  // ctor

void CameraSensor::setProjectionParameters(const CameraSensorSpec& spec) {
  // update this sensor's sensor spec to reflect the passed new values
  cameraSensorSpec_->resolution = spec.resolution;
  setCameraType(spec.sensorSubType);

}  // setProjectionParameters

void CameraSensor::recomputeProjectionMatrix() {
  projectionMatrix_ = zoomMatrix_ * baseProjMatrix_;
  // update renderCamera_'s projectionMatrix every time the sensor's
  // projectionMatrix changes
  renderCamera_->setProjectionMatrix(cameraSensorSpec_->resolution[1],
                                     cameraSensorSpec_->resolution[0],
                                     projectionMatrix_);
}

void CameraSensor::recomputeBaseProjectionMatrix() {
  // refresh size after relevant parameters have changed
  Mn::Vector2 nearPlaneSize_ =
      Mn::Vector2{1.0f, static_cast<float>(cameraSensorSpec_->resolution[0]) /
                            cameraSensorSpec_->resolution[1]};
  float scale;
  if (cameraSensorSpec_->sensorSubType == SensorSubType::Orthographic) {
    scale = cameraSensorSpec_->ortho_scale;
    nearPlaneSize_ /= scale;
    baseProjMatrix_ =
        Mn::Matrix4::orthographicProjection(nearPlaneSize_, near_, far_);
  } else {
    if (cameraSensorSpec_->sensorSubType != SensorSubType::Pinhole) {
      LOG(INFO) << "CameraSensor::setCameraType : Unsupported Camera type val :"
                << static_cast<int>(cameraSensorSpec_->sensorSubType)
                << " so defaulting to Pinhole.";
      cameraSensorSpec_->sensorSubType = SensorSubType::Pinhole;
    }
    Magnum::Deg halfHFovRad{Magnum::Deg(.5 * hfov_)};
    scale = 1.0f / (2.0f * near_ * Magnum::Math::tan(halfHFovRad));
    nearPlaneSize_ /= scale;
    baseProjMatrix_ =
        Mn::Matrix4::perspectiveProjection(nearPlaneSize_, near_, far_);
  }
  // build projection matrix
  recomputeProjectionMatrix();
}  // CameraSensor::recomputeNearPlaneSize

bool CameraSensor::getObservationSpace(ObservationSpace& space) {
  space.spaceType = ObservationSpaceType::Tensor;
  space.shape = {static_cast<size_t>(cameraSensorSpec_->resolution[0]),
                 static_cast<size_t>(cameraSensorSpec_->resolution[1]),
                 static_cast<size_t>(cameraSensorSpec_->channels)};
  space.dataType = core::DataType::DT_UINT8;
  if (cameraSensorSpec_->sensorType == SensorType::Semantic) {
    space.dataType = core::DataType::DT_UINT32;
  } else if (cameraSensorSpec_->sensorType == SensorType::Depth) {
    space.dataType = core::DataType::DT_FLOAT;
  }
  return true;
}

gfx::RenderCamera* CameraSensor::getRenderCamera() {
  return renderCamera_;
}

bool CameraSensor::getObservation(sim::Simulator& sim, Observation& obs) {
  // TODO: check if sensor is valid?
  // TODO: have different classes for the different types of sensors
  //
  if (!hasRenderTarget())
    return false;

  drawObservation(sim);
  readObservation(obs);

  return true;
}

bool CameraSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  renderTarget().renderEnter();

  gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled())
    flags |= gfx::RenderCamera::Flag::FrustumCulling;

  gfx::Renderer::ptr renderer = sim.getRenderer();
  if (cameraSensorSpec_->sensorType == SensorType::Semantic) {
    // TODO: check sim has semantic scene graph
    renderer->draw(*this, sim.getActiveSemanticSceneGraph(), flags);
    if (&sim.getActiveSemanticSceneGraph() != &sim.getActiveSceneGraph()) {
      flags |= gfx::RenderCamera::Flag::ObjectsOnly;
      renderer->draw(*this, sim.getActiveSceneGraph(), flags);
    }
  } else {
    // SensorType is Depth or any other type
    renderer->draw(*this, sim.getActiveSceneGraph(), flags);
  }

  renderTarget().renderExit();

  return true;
}

void CameraSensor::readObservation(Observation& obs) {
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
  if (cameraSensorSpec_->sensorType == SensorType::Semantic) {
    renderTarget().readFrameObjectId(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32UI, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else if (cameraSensorSpec_->sensorType == SensorType::Depth) {
    renderTarget().readFrameDepth(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32F, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else {
    renderTarget().readFrameRgba(Magnum::MutableImageView2D{
        Magnum::PixelFormat::RGBA8Unorm, renderTarget().framebufferSize(),
        obs.buffer->data});
  }
}

bool CameraSensor::displayObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  drawObservation(sim);
  renderTarget().blitRgbaToDefault();

  return true;
}

bool operator==(const CameraSensorSpec& a, const CameraSensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubType == b.sensorSubType && a.ortho_scale == b.ortho_scale &&
         a.position == b.position && a.orientation == b.orientation &&
         a.resolution == b.resolution && a.channels == b.channels &&
         a.encoding == b.encoding && a.observationSpace == b.observationSpace &&
         a.noiseModel == b.noiseModel && a.gpu2gpuTransfer == b.gpu2gpuTransfer;
}

Corrade::Containers::Optional<Magnum::Vector2> CameraSensor::depthUnprojection()
    const {
  // projectionMatrix_ is managed by implementation class and is set whenever
  // quantities change.
  return {gfx::calculateDepthUnprojection(projectionMatrix_)};
}  // CameraSensor::depthUnprojection

}  // namespace sensor
}  // namespace esp
