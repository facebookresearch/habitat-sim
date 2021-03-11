// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "EquirectangularSensor.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/gfx/EquirectangularShader.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace sensor {

EquirectangularSensorSpec::EquirectangularSensorSpec() : VisualSensorSpec() {
  uuid = "equirectangular";
  sensorSubType = SensorSubType::Equirectangular;
}

void EquirectangularSensorSpec::sanityCheck() {
  VisualSensorSpec::sanityCheck();
  CORRADE_ASSERT(
      sensorSubType == SensorSubType::Equirectangular,
      "EquirectangularSensorSpec::sanityCheck(): sensorSpec is not Equirectangular", );
}


EquirectangularSensor::EquirectangularSensor(scene::SceneNode& cameraNode,
                             const EquirectangularSensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec) {
  // initialize a cubemap
  auto& res = equirectangularSensorSpec_->resolution;
  int size = res[0] < res[1] ? res[0] : res[1];
  gfx::CubeMap::Flags cubeMapFlags = {};
  switch (equirectangularSensorSpec_->sensorType) {
    case SensorType::Color:
      cubeMapFlags |= gfx::CubeMap::Flag::ColorTexture;
      break;
    case SensorType::Depth:
      cubeMapFlags |= gfx::CubeMap::Flag::DepthTexture;
      break;
    // TODO: Semantic
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  cubeMap_ = std::make_unique<gfx::CubeMap>(size, cubeMapFlags);

  // initialize the cubemap camera, it attaches to the same node as the sensor
  // You do not have to release it in the dtor since magnum scene graph will
  // handle it
  cubeMapCamera_ = new gfx::CubeMapCamera(cameraNode);
  cubeMapCamera_->setProjectionMatrix(size, equirectangularSensorSpec_->near,
                                      equirectangularSensorSpec_->far);

  // setup shader flags
  switch (equirectangularSensorSpec_->sensorType) {
    case SensorType::Color:
      equirectangularShaderFlags_ |= gfx::EquirectangularShader::Flag::ColorTexture;
      break;
    case SensorType::Depth:
      equirectangularShaderFlags_ |= gfx::EquirectangularShader::Flag::DepthTexture;
      break;
    // TODO: Semantic
    // sensor type list is too long, have to use default
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  shader_ = new gfx::EquirectangularShader(equirectangularShaderFlags_);
  // prepare a big triangle mesh to cover the screen
  mesh_ = Mn::GL::Mesh{};
  mesh_.setCount(3);
}

bool EquirectangularSensorSpec::operator==(const EquirectangularSensorSpec& a) const {
  return VisualSensorSpec::operator==(a) &&
         focalLength == a.focalLength &&
         principalPointOffset == a.principalPointOffset;
}


bool EquirectangularSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  // in case the equirectangular sensor resolution changed at runtime
  {
    auto& res = equirectangularSensorSpec_->resolution;
    int size = res[0] < res[1] ? res[0] : res[1];
    bool reset = cubeMap_->reset(size);
    if (reset) {
      cubeMapCamera_->setProjectionMatrix(size, equirectangularSensorSpec_->near,
                                          equirectangularSensorSpec_->far);
      LOG(INFO) << "#### reset!!" << size << " ";
    }
  }

  esp::gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled()) {
    flags |= gfx::RenderCamera::Flag::FrustumCulling;
  }
  // generate the cubemap texture
  cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(), flags);


    if (equirectangularSensorSpec_->sensorType == SensorType::Color) {
        shader_->bindColorTexture(
            cubeMap_->getTexture(gfx::CubeMap::TextureType::Color));
    }
    if (equirectangularSensorSpec_->sensorType == SensorType::Depth) {
        shader_->bindDepthTexture(
            cubeMap_->getTexture(gfx::CubeMap::TextureType::Depth));
    }
    renderTarget().renderEnter();
    shader_->draw(mesh_);
    renderTarget().renderExit();
    return true;
}

Cr::Containers::Optional<Mn::Vector2> EquirectangularSensor::depthUnprojection() const {
  float f = equirectangularSensorSpec_->far;
  float n = equirectangularSensorSpec_->near;
  float d = f - n;
  // in projection matrix, two entries related to the depth are:
  // -(f+n)/(f-n), -2fn/(f-n), where f is the far plane, and n is the near
  // plane. depth parameters = 0.5 * vector(proj[2][2] - 1.0f, proj[3][2])
  return {0.5 * Mn::Vector2{-(f + n) / d - 1.0f, -2.0f * f * n / d}};
}

}  // namespace sensor
}  // namespace esp
