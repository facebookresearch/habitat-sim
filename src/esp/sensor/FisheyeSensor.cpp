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
  CORRADE_ASSERT(resolution[0] == resolution[1],
                 "FisheyeSensorSpec::sanityCheck(): the image width and "
                 "height should be identical.", );
  CORRADE_ASSERT(focalLength[0] > 0 && focalLength[1] > 0,
                 "FisheyeSensorSpec::sanityCheck(): focal length,"
                     << focalLength << "is illegal.", );
}

void FisheyeSensorDoubleSphereSpec::sanityCheck() {
  FisheyeSensorSpec::sanityCheck();
  CORRADE_ASSERT(alpha >= 0 && alpha <= 1.0,
                 "FisheyeSensorDoubleSphereSpec::sanityCheck(): alpha"
                     << alpha << "is illegal.", );
}

FisheyeSensor::FisheyeSensor(scene::SceneNode& cameraNode,
                             const FisheyeSensorSpec::ptr& spec)
    : CameraSensor(cameraNode, std::static_pointer_cast<SensorSpec>(spec)) {
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

  // initialize the cubemap camera, it attaches to the same node as the sensor
  cubeMapCamera_ = new gfx::CubeMapCamera(cameraNode);

  // initialize a cubemap
  cubeMap_ = std::make_unique<esp::gfx::CubeMap>(actualSpec->resolution[0]);

  // TODO: assign flag based on sensor type (color, depth, semantic)
  fisheyeShaderFlags_ |= gfx::FisheyeShader::Flag::ColorTexture;
}

Mn::ResourceKey FisheyeSensor::getShaderKey() {
  return Cr::Utility::formatString(
      FISH_EYE_SHADER_KEY_TEMPLATE,
      static_cast<Mn::UnsignedInt>(fisheyeSensorSpec_->fisheyeModelType),
      static_cast<gfx::FisheyeShader::Flags::UnderlyingType>(
          fisheyeShaderFlags_));
}

bool FisheyeSensor::drawObservation(sim::Simulator& sim) {
  esp::gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled()) {
    flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
  }
  // generate the cubemap texture
  cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(), flags);

  // obtain shader based on fisheye model type
  Mn::ResourceKey key = getShaderKey();
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

  return true;
}
}  // namespace sensor
}  // namespace esp
