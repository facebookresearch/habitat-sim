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

struct VoxelGrid : Cr::TestSuite::Tester {
  explicit VoxelGrid();

  void testVoxelGridWithVHACD();
};

VoxelGrid::VoxelGrid() {
#ifdef ESP_BUILD_WITH_VHACD
  addTests({&VoxelGrid::testVoxelGridWithVHACD});
#endif
}

#ifdef ESP_BUILD_WITH_VHACD
void VoxelGrid::testVoxelGridWithVHACD() {
  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.glb");
  simConfig.enablePhysics = true;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;

  auto simulator_ = esp::sim::Simulator::create_unique(simConfig);

  // Voxelize the stage with resolution = 1,000,000 and make asserts
  const int resolution = 1000000;
  simulator_->createStageVoxelization(resolution);
  auto voxelization = simulator_->getStageVoxelization().get();

  // Verify coordinate conversion works in both directions
  Mn::Vector3i voxelIndex(2, 1, 7);
  Mn::Vector3 globalCoords =
      voxelization->getGlobalCoordsFromVoxelIndex(voxelIndex);
  Mn::Vector3i deconvertedVoxelIndex =
      voxelization->getVoxelIndexFromGlobalCoords(globalCoords);
  CORRADE_VERIFY(voxelIndex == deconvertedVoxelIndex);

  // "Golden Value Tests" - Verify that certain values return the correct
  // coordinates
  // TODO Add a few more values
  std::vector<Mn::Vector3i> voxel_indices = std::vector<Mn::Vector3i>{
      Mn::Vector3i(0, 0, 0),
      voxelization->getVoxelGridDimensions() - Mn::Vector3i(1, 1, 1)};
  // "Hardcoded" values for the global coordinates corresponding to the
  // positions of the voxel indices.
  std::vector<Mn::Vector3> global_coords =
      std::vector<Mn::Vector3>{Mn::Vector3(-9.75916, -0.390074, 0.973851),
                               Mn::Vector3(8.89573, 7.07188, 25.5983)};
  for (int i = 0; i < voxel_indices.size(); i++) {
    CORRADE_VERIFY(voxelization->getGlobalCoordsFromVoxelIndex(
                       voxel_indices[i]) == global_coords[i]);
    CORRADE_VERIFY(voxelization->getVoxelIndexFromGlobalCoords(
                       global_coords[i]) == voxel_indices[i]);
  }

  // Ensure voxel grid setters and getters work, specifically direct grid
  // manipulation using a strided array view
  voxelization->addGrid<int>("intGrid");
  auto intGrid = voxelization->getGrid<int>("intGrid");

  Mn::Vector3i dims = voxelization->getVoxelGridDimensions();
  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        intGrid[i][j][k] = 10;
      }
    }
  }
  bool settersWorked = true;

  for (int i = 0; i < dims[0]; i++) {
    for (int j = 0; j < dims[1]; j++) {
      for (int k = 0; k < dims[2]; k++) {
        if (voxelization->getVoxel<int>(Mn::Vector3i(i, j, k), "intGrid") !=
            10) {
          settersWorked = false;
        }
      }
    }
  }

  CORRADE_VERIFY(settersWorked);

  // Ensure mesh generation & mesh visualization doesn't crash simulator
  voxelization->generateMesh("Boundary");

  // Only one mesh can be visualized at a time
  simulator_->setStageVoxelizationDraw(true, "Boundary");

  // Turn off visualization
  simulator_->setStageVoxelizationDraw(false, "Boundary");
}
#endif

CORRADE_TEST_MAIN(VoxelGrid)
