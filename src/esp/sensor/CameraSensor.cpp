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

CameraSensor::CameraSensor(scene::SceneNode& cameraNode,
                           const SensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec),
      baseProjMatrix_(Magnum::Math::IdentityInit),
      zoomMatrix_(Magnum::Math::IdentityInit) {
  setProjectionParameters(spec);
}  // ctor

void CameraSensor::setProjectionParameters(const SensorSpec::ptr& spec) {
  ASSERT(spec != nullptr);
  // update this sensor's sensor spec to reflect the passed new values
  spec_->resolution = spec->resolution;
  for (const auto& elem : spec->parameters) {
    spec_->parameters.at(elem.first) = elem.second;
  }

  width_ = spec_->resolution[1];
  height_ = spec_->resolution[0];
  near_ = std::atof(spec_->parameters.at("near").c_str());
  far_ = std::atof(spec_->parameters.at("far").c_str());
  setCameraType(spec->sensorSubType);

}  // setProjectionParameters

void CameraSensor::recomputeBaseProjectionMatrix() {
  // refresh size after relevant parameters have changed
  Mn::Vector2 nearPlaneSize_ =
      Mn::Vector2{1.0f, static_cast<float>(height_) / width_};
  float scale;
  if (spec_->sensorSubType == SensorSubType::Orthographic) {
    scale = std::atof(spec_->parameters.at("ortho_scale").c_str());
    nearPlaneSize_ /= scale;
    baseProjMatrix_ =
        Mn::Matrix4::orthographicProjection(nearPlaneSize_, near_, far_);
  } else {
    if (spec_->sensorSubType != SensorSubType::Pinhole) {
      LOG(INFO) << "CameraSensor::setCameraType : Unsupported Camera type val :"
                << static_cast<int>(spec_->sensorSubType)
                << " so defaulting to Pinhole.";
      spec_->sensorSubType = SensorSubType::Pinhole;
    }
    float fov = std::atof(spec_->parameters.at("hfov").c_str());
    Magnum::Deg halfHFovRad{Magnum::Deg(.5 * fov)};
    scale = 1.0f / (2.0f * near_ * Magnum::Math::tan(halfHFovRad));
    nearPlaneSize_ /= scale;
    baseProjMatrix_ =
        Mn::Matrix4::perspectiveProjection(nearPlaneSize_, near_, far_);
  }
  // build projection matrix
  recomputeProjectionMatrix();
}  // CameraSensor::recomputeNearPlaneSize

CameraSensor& CameraSensor::setProjectionMatrix(
    gfx::RenderCamera& targetCamera) {
  targetCamera.setProjectionMatrix(width_, height_, projectionMatrix_);
  return *this;
}

CameraSensor& CameraSensor::setTransformationMatrix(
    gfx::RenderCamera& targetCamera) {
  CORRADE_ASSERT(!scene::SceneGraph::isRootNode(targetCamera.node()),
                 "CameraSensor::setTransformationMatrix: target camera cannot "
                 "be on the root node of the scene graph",
                 *this);
  Magnum::Matrix4 absTransform = this->node().absoluteTransformation();
  Magnum::Matrix3 rotation = absTransform.rotationScaling();
  Magnum::Math::Algorithms::gramSchmidtOrthonormalizeInPlace(rotation);

  VLOG(2) << "||R - GS(R)|| = "
          << Eigen::Map<mat3f>((rotation - absTransform.rotationShear()).data())
                 .norm();

  auto relativeTransform =
      Magnum::Matrix4::from(rotation, absTransform.translation()) *
      Magnum::Matrix4::scaling(absTransform.scaling());

  // set the transformation to the camera
  // so that the camera has the correct modelview matrix for rendering;
  // to do it,
  // obtain the *absolute* transformation from the sensor node,
  // apply it as the *relative* transformation between the camera and
  // its parent
  auto camParent = targetCamera.node().parent();
  // if camera's parent is the root node, skip it!
  if (!scene::SceneGraph::isRootNode(
          *static_cast<scene::SceneNode*>(camParent))) {
    relativeTransform =
        camParent->absoluteTransformation().inverted() * relativeTransform;
  }
  targetCamera.node().setTransformation(relativeTransform);
  return *this;
}

CameraSensor& CameraSensor::setViewport(gfx::RenderCamera& targetCamera) {
  targetCamera.setViewport(this->framebufferSize());
  return *this;
}

bool CameraSensor::getObservationSpace(ObservationSpace& space) {
  space.spaceType = ObservationSpaceType::Tensor;
  space.shape = {static_cast<size_t>(spec_->resolution[0]),
                 static_cast<size_t>(spec_->resolution[1]),
                 static_cast<size_t>(spec_->channels)};
  space.dataType = core::DataType::DT_UINT8;
  if (spec_->sensorType == SensorType::Semantic) {
    space.dataType = core::DataType::DT_UINT32;
  } else if (spec_->sensorType == SensorType::Depth) {
    space.dataType = core::DataType::DT_FLOAT;
  }
  return true;
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
  if (spec_->sensorType == SensorType::Semantic) {
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
  if (spec_->sensorType == SensorType::Semantic) {
    renderTarget().readFrameObjectId(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32UI, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else if (spec_->sensorType == SensorType::Depth) {
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

Corrade::Containers::Optional<Magnum::Vector2> CameraSensor::depthUnprojection()
    const {
  // projectionMatrix_ is managed by implementation class and is set whenever
  // quantities change.
  return {gfx::calculateDepthUnprojection(projectionMatrix_)};
}  // CameraSensor::depthUnprojection

}  // namespace sensor
}  // namespace esp
