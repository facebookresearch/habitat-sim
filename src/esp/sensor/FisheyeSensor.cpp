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

void FisheyeSensorSpec::sanityCheck() {
  // TODO:
  // call base class sanity check first
  CORRADE_ASSERT(focalLength[0] > 0 && focalLength[1] > 0,
                 "FisheyeSensorSpec::sanityCheck(): focal length,"
                     << focalLength << "is illegal.", );

  CORRADE_ASSERT(
      cubeMapCameraNear > 0.0 && cubeMapCameraFar > cubeMapCameraNear,
      "FisheyeSensorSpec::sanityCheck(): clipping planes for the cubemap "
      "camera are illegal.", );
}

void FisheyeSensorDoubleSphereSpec::sanityCheck() {
  FisheyeSensorSpec::sanityCheck();
  CORRADE_ASSERT(alpha >= 0 && alpha <= 1.0,
                 "FisheyeSensorDoubleSphereSpec::sanityCheck(): alpha"
                     << alpha << "is illegal.", );
}

FisheyeSensor::FisheyeSensor(scene::SceneNode& cameraNode,
                             const SensorSpec::ptr& spec)
    : VisualSensor(cameraNode, spec) {
  fisheyeSensorSpec_ = std::static_pointer_cast<FisheyeSensorSpec>(spec_);
  auto convertSpec = [&]() {
    switch (fisheyeSensorSpec_->fisheyeModelType) {
      case FisheyeSensorModelType::DoubleSphere:
        return static_cast<FisheyeSensorDoubleSphereSpec*>(
            fisheyeSensorSpec_.get());
        break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
  };
  auto actualSpec = convertSpec();
  actualSpec->sanityCheck();

  // initialize a cubemap
  auto& res = actualSpec->resolution;
  int size = res[0] < res[1] ? res[0] : res[1];
  gfx::CubeMap::Flags cubeMapFlags = {};
  if (fisheyeSensorSpec_->sensorType == SensorType::Color) {
    cubeMapFlags |= gfx::CubeMap::Flag::ColorTexture;
  }
  if (fisheyeSensorSpec_->sensorType == SensorType::Depth) {
    cubeMapFlags |= gfx::CubeMap::Flag::DepthTexture;
  }
  // TODO: Semantic
  cubeMap_ = std::make_unique<gfx::CubeMap>(size, cubeMapFlags);

  // initialize the cubemap camera, it attaches to the same node as the sensor
  cubeMapCamera_ = new gfx::CubeMapCamera(cameraNode);
  cubeMapCamera_->setProjectionMatrix(size,
                                      fisheyeSensorSpec_->cubeMapCameraNear,
                                      fisheyeSensorSpec_->cubeMapCameraFar);

  if (fisheyeSensorSpec_->sensorType == SensorType::Color) {
    fisheyeShaderFlags_ |= gfx::FisheyeShader::Flag::ColorTexture;
  }

  if (fisheyeSensorSpec_->sensorType == SensorType::Depth) {
    fisheyeShaderFlags_ |= gfx::FisheyeShader::Flag::DepthTexture;
  }

  // compute the depth unprojection parameters
  {
    float f = fisheyeSensorSpec_->cubeMapCameraFar;
    float n = fisheyeSensorSpec_->cubeMapCameraNear;
    float d = f - n;
    // in projection matrix, two entries related to the depth are:
    // -(f+n)/(f-n), -2fn/(f-n), where f is the far plane, and n is the near
    // plane. depth parameters = 0.5 * vector(proj[2][2] - 1.0f, proj[3][2])
    depthUnprojectionParameters_ =
        0.5 * Mn::Vector2{-(f + n) / d - 1.0f, -2.0f * f * n / d};
  }

  // prepare a big triangle mesh to cover the screen
  mesh_ = Mn::GL::Mesh{};
  mesh_.setCount(3);
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
    auto& res = spec_->resolution;
    int size = res[0] < res[1] ? res[0] : res[1];
    bool reset = cubeMap_->reset(size);
    if (reset) {
      cubeMapCamera_->setProjectionMatrix(size,
                                          fisheyeSensorSpec_->cubeMapCameraNear,
                                          fisheyeSensorSpec_->cubeMapCameraFar);
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

  // obtain shader based on fisheye model type
  shader_ = fisheyeShaderManager_.get<gfx::FisheyeShader>(getShaderKey());

  // if no shader with flags exists, create one
  if (!shader_) {
    switch (fisheyeSensorSpec_->fisheyeModelType) {
      case FisheyeSensorModelType::DoubleSphere:
        fisheyeShaderManager_.set<gfx::FisheyeShader>(
            shader_.key(),
            new gfx::DoubleSphereCameraShader{fisheyeShaderFlags_},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
        break;

        // TODO:
        // The other FisheyeSensorModelType

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
  }
  CORRADE_INTERNAL_ASSERT(shader_ && shader_->flags() == fisheyeShaderFlags_);
  // draw the observation to the render target

  switch (fisheyeSensorSpec_->fisheyeModelType) {
    case FisheyeSensorModelType::DoubleSphere: {
      auto& actualShader =
          static_cast<gfx::DoubleSphereCameraShader&>(*shader_);
      auto& actualSpec =
          static_cast<FisheyeSensorDoubleSphereSpec&>(*fisheyeSensorSpec_);
      actualShader.setFocalLength(actualSpec.focalLength)
          .setPrincipalPointOffset(actualSpec.principalPointOffset)
          .setAlpha(actualSpec.alpha)
          .setXi(actualSpec.xi);
    } break;

      // TODO:
      // The other FisheyeSensorModelType

    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  if (fisheyeSensorSpec_->sensorType == SensorType::Color) {
    shader_->bindColorTexture(
        cubeMap_->getTexture(gfx::CubeMap::TextureType::Color));
  }
  if (fisheyeSensorSpec_->sensorType == SensorType::Depth) {
    shader_->bindDepthTexture(
        cubeMap_->getTexture(gfx::CubeMap::TextureType::Depth));
  }
  renderTarget().renderEnter();
  shader_->draw(mesh_);
  renderTarget().renderExit();

  return true;
}

/**
 * @brief Returns the parameters needed to unproject depth for the sensor.
 *
 * Will always be @ref Corrade::Containers::NullOpt for the base sensor class
 * as it has no projection parameters
 */
Cr::Containers::Optional<Mn::Vector2> FisheyeSensor::depthUnprojection() const {
  return {depthUnprojectionParameters_};
};
}  // namespace sensor
}  // namespace esp
