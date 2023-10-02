// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CameraSensor.h"
#include <Corrade/Utility/Assert.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/PixelFormat.h>
#include <cmath>

#include "CameraSensor.h"
#include "esp/gfx_batch/DepthUnprojection.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

CameraSensorSpec::CameraSensorSpec() : VisualSensorSpec() {
  uuid = "rgba_camera";
  sensorSubType = SensorSubType::Pinhole;
}

void CameraSensorSpec::sanityCheck() const {
  VisualSensorSpec::sanityCheck();
  CORRADE_ASSERT(sensorSubType == SensorSubType::Pinhole ||
                     sensorSubType == SensorSubType::Orthographic,
                 "CameraSensorSpec::sanityCheck(): sensorSpec does not have "
                 "SensorSubType "
                 "Pinhole or Orthographic", );
  CORRADE_ASSERT(
      orthoScale > 0,
      "CameraSensorSpec::sanityCheck(): orthoScale must be greater than 0", );
}

bool CameraSensorSpec::operator==(const CameraSensorSpec& a) const {
  return VisualSensorSpec::operator==(a) && a.orthoScale == orthoScale &&
         a.hfov == hfov;
}

namespace {

/* Needs to be an internal utility because CameraSensor uses everything except
   hFoV from the CameraSensorSpec */
Mn::Matrix4 projectionMatrixInternal(const CameraSensorSpec& spec,
                                     Mn::Rad hfov) {
  const Mn::Vector2 nearPlaneSize{
      1.0f, Mn::Vector2{Mn::Vector2i{spec.resolution}}.aspectRatio()};
  if (spec.sensorSubType == SensorSubType::Orthographic) {
    return Mn::Matrix4::orthographicProjection(nearPlaneSize / spec.orthoScale,
                                               spec.near, spec.far);
  } else if (spec.sensorSubType == SensorSubType::Pinhole) {
    const float scale = 1.0f / (2.0f * spec.near * Mn::Math::tan(0.5f * hfov));
    return Mn::Matrix4::perspectiveProjection(nearPlaneSize / scale, spec.near,
                                              spec.far);
  } else
    CORRADE_ASSERT_UNREACHABLE(
        "CameraSensorSpec::projectionMatrix(): sensorSpec does not have "
        "SensorSubType Pinhole or Orthographic",
        {});
}

}  // namespace

Mn::Matrix4 CameraSensorSpec::projectionMatrix() const {
  return projectionMatrixInternal(*this, hfov);
}

CameraSensor::CameraSensor(scene::SceneNode& cameraNode,
                           const CameraSensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec),
      baseProjMatrix_(Magnum::Math::IdentityInit),
      zoomMatrix_(Magnum::Math::IdentityInit),
      renderCamera_(new gfx::RenderCamera(cameraNode)) {
  // Sanity check
  CORRADE_ASSERT(
      cameraSensorSpec_,
      "CameraSensor::CameraSensor(): The input sensorSpec is illegal", );
  cameraSensorSpec_->sanityCheck();

  // RenderCamera initialized in member list
  setProjectionParameters(*spec);
  renderCamera_->setAspectRatioPolicy(
      Mn::SceneGraph::AspectRatioPolicy::Extend);
  recomputeProjectionMatrix();
  renderCamera_->setViewport(this->framebufferSize());
  // Set initial hFOV
  setFOV(cameraSensorSpec_->hfov);
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
  baseProjMatrix_ = projectionMatrixInternal(*cameraSensorSpec_, hfov_);
  // build projection matrix
  recomputeProjectionMatrix();
}  // CameraSensor::recomputeNearPlaneSize

gfx::RenderCamera* CameraSensor::getRenderCamera() const {
  return renderCamera_;
}

void CameraSensor::draw(scene::SceneGraph& sceneGraph,
                        gfx::RenderCamera::Flags flags) {
  gfx::DrawableGroup& defaultRenderingGroup = sceneGraph.getDrawables();
  defaultRenderingGroup.prepareForDraw(*renderCamera_);
  renderCamera_->draw(defaultRenderingGroup, flags);
}

bool CameraSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  renderTarget().renderEnter();

  gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled()) {
    flags |= gfx::RenderCamera::Flag::FrustumCulling;
  }

  if (cameraSensorSpec_->sensorType == SensorType::Semantic) {
    // TODO: check sim has semantic scene graph
    bool twoSceneGraphs =
        (&sim.getActiveSemanticSceneGraph() != &sim.getActiveSceneGraph());

    if (twoSceneGraphs) {
      VisualSensor::MoveSemanticSensorNodeHelper helper(*this, sim);
      draw(sim.getActiveSemanticSceneGraph(), flags);
    } else {
      draw(sim.getActiveSemanticSceneGraph(), flags);
    }

    if (twoSceneGraphs) {
      flags |= gfx::RenderCamera::Flag::ObjectsOnly;
      draw(sim.getActiveSceneGraph(), flags);
    }
  } else {
    // SensorType is Color, Depth or any other type
    draw(sim.getActiveSceneGraph(), flags);

    if (cameraSensorSpec_->sensorType == SensorType::Color) {
      // include HBAO in Color sensors (only if enabled for render target)
      renderTarget().tryDrawHbao();

      // include DebugLineRender in Color sensors
      const auto debugLineRender = sim.getDebugLineRender();
      if (debugLineRender) {
        debugLineRender->flushLines(renderCamera_->cameraMatrix(),
                                    renderCamera_->projectionMatrix(),
                                    renderCamera_->viewport());
      }
    }
  }

  renderTarget().renderExit();

  return true;
}

Corrade::Containers::Optional<Magnum::Vector2> CameraSensor::depthUnprojection()
    const {
  // projectionMatrix_ is managed by implementation class and is set whenever
  // quantities change.
  return {gfx_batch::calculateDepthUnprojection(projectionMatrix_)};
}  // CameraSensor::depthUnprojection

}  // namespace sensor
}  // namespace esp
