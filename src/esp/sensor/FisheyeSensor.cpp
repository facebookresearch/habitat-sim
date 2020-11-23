// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FisheyeSensor.h"

#include <Corrade/Utility/Assert.h>

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
  CORRADE_ASSERT(
      fov > 0.0 && fov < 360.0,
      "FisheyeSensorSpec::sanityCheck(): fov" << fov << "is illegal.", );
}

void FisheyeSensorDoubleSphereSpec::sanityCheck() {
  FisheyeSensorSpec::sanityCheck();
  CORRADE_ASSERT(alpha >= 0 && alpha <= 1.0,
                 "FisheyeSensorDoubleSphereSpec::sanityCheck(): alpha"
                     << alpha << "is illegal.", );
}

FisheyeSensor::FisheyeSensor(scene::SceneNode& cameraNode,
                             const FisheyeSensorSpec::ptr& spec)
    : CameraSensor(cameraNode, std::static_pointer_cast<SensorSpec>(spec)),
      type_(spec->fisheyeModelType) {
  auto convertSpec = [&]() {
    switch (spec->fisheyeModelType) {
      case FisheyeSensorModelType::DoubleSphere:
        return std::static_pointer_cast<FisheyeSensorDoubleSphereSpec>(spec);
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
}

bool FisheyeSensor::drawObservation(sim::Simulator& sim) {
  esp::gfx::RenderCamera::Flags flags;
  if (sim.isFrustumCullingEnabled()) {
    flags |= esp::gfx::RenderCamera::Flag::FrustumCulling;
  }
  // generate the cubemap texture
  cubeMap_->renderToTexture(*cubeMapCamera_, sim.getActiveSceneGraph(), flags);
}
}  // namespace sensor
}  // namespace esp
