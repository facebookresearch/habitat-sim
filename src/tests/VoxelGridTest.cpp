// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/Magnum.h>

#include "esp/geo/VoxelWrapper.h"
#include "esp/sim/Simulator.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

// TODO: Add tests for different Sensors
struct VoxelGrid : Cr::TestSuite::Tester {
  explicit VoxelGrid();

  void testVoxelGrid();
};

VoxelGrid::VoxelGrid() {
#ifdef ESP_BUILD_WITH_VHACD
  addTests({&VoxelGrid::testVoxelGrid});
#endif
}

#ifdef ESP_BUILD_WITH_VHACD
void VoxelGrid::testVoxelGrid() {
  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.glb");
  simConfig.enablePhysics = true;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;

  auto simulator_ = esp::sim::Simulator::create_unique(simConfig);

  // Voxelize the scene with resolution = 1,000,000 and make asserts
  const int resolution = 1000000;
  simulator_->createSceneVoxelization(resolution);
  auto voxelWrapper = simulator_->getSceneVoxelization().get();

  // Verify coordinate conversion works in both directions
  Mn::Vector3i voxelIndex(2, 1, 7);
  Mn::Vector3 globalCoords =
      voxelWrapper->getGlobalCoordsFromVoxelIndex(voxelIndex);
  Mn::Vector3i deconvertedVoxelIndex =
      voxelWrapper->getVoxelIndexFromGlobalCoords(globalCoords);
  CORRADE_VERIFY(voxelIndex == deconvertedVoxelIndex);
}
#endif

CORRADE_TEST_MAIN(VoxelGrid)
