// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FisheyeSensor.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/gfx/DoubleSphereCameraShader.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace sensor {

FisheyeSensorSpec::FisheyeSensorSpec() : VisualSensorSpec() {
  uuid = "fisheye";
  sensorSubType = SensorSubType::Fisheye;
}

void FisheyeSensorSpec::sanityCheck() {
  VisualSensorSpec::sanityCheck();
  CORRADE_ASSERT(
      sensorSubType == SensorSubType::Fisheye,
      "FisheyeSensorSpec::sanityCheck(): sensorSpec is not Fisheye", );

  CORRADE_ASSERT(focalLength[0] > 0 && focalLength[1] > 0,
                 "FisheyeSensorSpec::sanityCheck(): focal length,"
                     << focalLength << "is illegal.", );
}

void FisheyeSensorDoubleSphereSpec::sanityCheck() {
  FisheyeSensorSpec::sanityCheck();
  CORRADE_ASSERT(fisheyeModelType == FisheyeSensorModelType::DoubleSphere,
                 "FisheyeSensorDoubleSphereSpec::sanityCheck(): fisheye model "
                 "type is wrong. It should be DoubleSphere.", );
  CORRADE_ASSERT(alpha >= 0 && alpha <= 1.0,
                 "FisheyeSensorDoubleSphereSpec::sanityCheck(): alpha"
                     << alpha << "is illegal.", );
}

template <typename T>
void specSanityCheck(FisheyeSensorSpec* spec) {
  auto actualSpec = dynamic_cast<T*>(spec);
  CORRADE_ASSERT(actualSpec,
                 "FisheyeSensor::FisheyeSensor(): the spec cannot be converted "
                 "to any known fiesheye sensor spec.", );
  actualSpec->sanityCheck();
}

FisheyeSensor::FisheyeSensor(scene::SceneNode& cameraNode,
                             const FisheyeSensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec) {
  switch (fisheyeSensorSpec_->fisheyeModelType) {
    case FisheyeSensorModelType::DoubleSphere: {
      specSanityCheck<FisheyeSensorDoubleSphereSpec>(fisheyeSensorSpec_.get());
    } break;
  };

  // initialize a cubemap
  auto& res = fisheyeSensorSpec_->resolution;
  int size = res[0] < res[1] ? res[0] : res[1];
  gfx::CubeMap::Flags cubeMapFlags = {};
  switch (fisheyeSensorSpec_->sensorType) {
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
  cubeMapCamera_->setProjectionMatrix(size, fisheyeSensorSpec_->near,
                                      fisheyeSensorSpec_->far);

  // setup shader flags
  switch (fisheyeSensorSpec_->sensorType) {
    case SensorType::Color:
      fisheyeShaderFlags_ |= gfx::FisheyeShader::Flag::ColorTexture;
      break;
    case SensorType::Depth:
      fisheyeShaderFlags_ |= gfx::FisheyeShader::Flag::DepthTexture;
      break;
    // TODO: Semantic
    // sensor type list is too long, have to use default
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  // prepare a big triangle mesh to cover the screen
  mesh_ = Mn::GL::Mesh{};
  mesh_.setCount(3);
}

bool FisheyeSensorSpec::operator==(const FisheyeSensorSpec& a) const {
  return VisualSensorSpec::operator==(a) &&
         fisheyeModelType == a.fisheyeModelType &&
         focalLength == a.focalLength &&
         principalPointOffset == a.principalPointOffset;
}

Mn::ResourceKey FisheyeSensor::getShaderKey() {
  return Cr::Utility::formatString(
      FISH_EYE_SHADER_KEY_TEMPLATE,
      static_cast<Mn::UnsignedInt>(fisheyeSensorSpec_->fisheyeModelType),
      static_cast<gfx::FisheyeShader::Flags::UnderlyingType>(
          fisheyeShaderFlags_));
}

bool FisheyeSensor::drawObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }

  // in case the fisheye sensor resolution changed at runtime
  {
    auto& res = fisheyeSensorSpec_->resolution;
    int size = res[0] < res[1] ? res[0] : res[1];
    bool reset = cubeMap_->reset(size);
    if (reset) {
      cubeMapCamera_->setProjectionMatrix(size, fisheyeSensorSpec_->near,
                                          fisheyeSensorSpec_->far);
    }
  }

  esp::gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled()) {
    flags |= gfx::RenderCamera::Flag::FrustumCulling;
  }
  // generate the cubemap texture
  cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(), flags);
  // XXX debug
  /*
  if (fisheyeSensorSpec_->sensorType == SensorType::Color) {
    cubeMap_->saveTexture(esp::gfx::CubeMap::TextureType::Color, "cubemap");
    cubeMap_->loadTexture(gfx::CubeMap::TextureType::Color, "cubemap", "png");
  }
  */
  // XXX debug
  /*
  if (fisheyeSensorSpec_->sensorType == SensorType::Depth) {
    cubeMap_->saveTexture(esp::gfx::CubeMap::TextureType::Depth, "cubemap");
    cubeMap_->loadTexture(gfx::CubeMap::TextureType::Depth, "cubemap", "hdr");
    exit(0);
  }
  */

  auto drawWith = [&](auto& shader) {
    if (fisheyeSensorSpec_->sensorType == SensorType::Color) {
      shader.bindColorTexture(
          cubeMap_->getTexture(gfx::CubeMap::TextureType::Color));
    }
    if (fisheyeSensorSpec_->sensorType == SensorType::Depth) {
      shader.bindDepthTexture(
          cubeMap_->getTexture(gfx::CubeMap::TextureType::Depth));
    }
    renderTarget().renderEnter();
    shader.draw(mesh_);
    renderTarget().renderExit();
  };

  switch (fisheyeSensorSpec_->fisheyeModelType) {
    case FisheyeSensorModelType::DoubleSphere: {
      Magnum::Resource<gfx::FisheyeShader, gfx::DoubleSphereCameraShader>
          shader = getShader<gfx::DoubleSphereCameraShader>();
      auto& actualSpec =
          static_cast<FisheyeSensorDoubleSphereSpec&>(*fisheyeSensorSpec_);
      (*shader)
          .setFocalLength(actualSpec.focalLength)
          .setPrincipalPointOffset(actualSpec.principalPointOffset)
          .setAlpha(actualSpec.alpha)
          .setXi(actualSpec.xi);
      drawWith(*shader);
    } break;

      // TODO:
      // The other FisheyeSensorModelType
  }

  return true;
}

Cr::Containers::Optional<Mn::Vector2> FisheyeSensor::depthUnprojection() const {
  float f = fisheyeSensorSpec_->far;
  float n = fisheyeSensorSpec_->near;
  float d = f - n;
  // in projection matrix, two entries related to the depth are:
  // -(f+n)/(f-n), -2fn/(f-n), where f is the far plane, and n is the near
  // plane. depth parameters = 0.5 * vector(proj[2][2] - 1.0f, proj[3][2])
  return {0.5 * Mn::Vector2{-(f + n) / d - 1.0f, -2.0f * f * n / d}};
}

}  // namespace sensor
}  // namespace esp
