// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/ImageView.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/PixelFormat.h>

#include "PinholeCamera.h"
#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/Renderer.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

PinholeCamera::PinholeCamera(scene::SceneNode& pinholeCameraNode,
                             const sensor::SensorSpec::ptr& spec)
    : sensor::VisualSensor(pinholeCameraNode, spec) {
  setProjectionParameters(spec);
}

void PinholeCamera::setProjectionParameters(const SensorSpec::ptr& spec) {
  ASSERT(spec != nullptr);
  width_ = spec_->resolution[1];
  height_ = spec_->resolution[0];
  near_ = std::atof(spec_->parameters.at("near").c_str());
  far_ = std::atof(spec_->parameters.at("far").c_str());
  hfov_ = std::atof(spec_->parameters.at("hfov").c_str());
}

PinholeCamera& PinholeCamera::setProjectionMatrix(
    gfx::RenderCamera& targetCamera) {
  targetCamera.setProjectionMatrix(width_, height_, near_, far_, hfov_);
  return *this;
}

PinholeCamera& PinholeCamera::setTransformationMatrix(
    gfx::RenderCamera& targetCamera) {
  CORRADE_ASSERT(!scene::SceneGraph::isRootNode(targetCamera.node()),
                 "PinholeCamera::setTransformationMatrix: target camera cannot "
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

PinholeCamera& PinholeCamera::setViewport(gfx::RenderCamera& targetCamera) {
  targetCamera.setViewport(this->framebufferSize());
  return *this;
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

bool PinholeCamera::getObservation(sim::Simulator& sim, Observation& obs) {
  // TODO: check if sensor is valid?
  // TODO: have different classes for the different types of sensors
  //
  if (!hasRenderTarget())
    return false;

  drawObservation(sim);
  readObservation(obs);

  return true;
}

bool PinholeCamera::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  renderTarget().renderEnter();

  gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled())
    flags |= gfx::RenderCamera::Flag::FrustumCulling;

  gfx::Renderer::ptr renderer = sim.getRenderer();
  if (spec_->sensorType == SensorType::SEMANTIC) {
    // TODO: check sim has semantic scene graph
    renderer->draw(*this, sim.getActiveSemanticSceneGraph(), flags);
    if (&sim.getActiveSemanticSceneGraph() != &sim.getActiveSceneGraph()) {
      flags |= gfx::RenderCamera::Flag::ObjectsOnly;
      renderer->draw(*this, sim.getActiveSceneGraph(), flags);
    }
  } else {
    // SensorType is DEPTH or any other type
    renderer->draw(*this, sim.getActiveSceneGraph(), flags);
  }

  renderTarget().renderExit();

  return true;
}

void PinholeCamera::readObservation(Observation& obs) {
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
  if (spec_->sensorType == SensorType::SEMANTIC) {
    renderTarget().readFrameObjectId(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32UI, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else if (spec_->sensorType == SensorType::DEPTH) {
    renderTarget().readFrameDepth(Magnum::MutableImageView2D{
        Magnum::PixelFormat::R32F, renderTarget().framebufferSize(),
        obs.buffer->data});
  } else {
    renderTarget().readFrameRgba(Magnum::MutableImageView2D{
        Magnum::PixelFormat::RGBA8Unorm, renderTarget().framebufferSize(),
        obs.buffer->data});
  }
}

bool PinholeCamera::displayObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  drawObservation(sim);
  renderTarget().blitRgbaToDefault();

  return true;
}

Corrade::Containers::Optional<Magnum::Vector2>
PinholeCamera::depthUnprojection() const {
  const Magnum::Matrix4 projection = Magnum::Matrix4::perspectiveProjection(
      Magnum::Deg{hfov_}, static_cast<float>(width_) / height_, near_, far_);

  return {gfx::calculateDepthUnprojection(projection)};
}

}  // namespace sensor
}  // namespace esp
