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

  // Voxelize the scene with resolution = 1,000,000 and make asserts
  const int resolution = 1000000;
  simulator_->createSceneVoxelization(resolution);
  auto voxelization = simulator_->getSceneVoxelization().get();

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
  // Generate SDFs and Distance Gradient Fields. Ensures nothing is blatantly
  // broken.
  // TODO validate with pre-computed values
  voxelization->generateEuclideanDistanceSDF("EuclideanSDF");
  voxelization->generateManhattanDistanceSDF("ManhattanSDF");
  voxelization->generateDistanceGradientField("GradientField");
  auto sdf_grid = voxelization->getGrid<float>("EuclideanSDF");

  // "Hardcoded" values for the Euclidean SDF values corresponding to the voxel
  // indices.
  std::vector<float> distances = std::vector<float>{7, 1.73205};
  for (int i = 0; i < voxel_indices.size(); i++) {
    Mn::Vector3i coord = voxel_indices[i];
    CORRADE_VERIFY(abs(sdf_grid[coord[0]][coord[1]][coord[2]] - distances[i]) <
                   0.1);
    // Test setting voxel values
    sdf_grid[coord[0]][coord[1]][coord[2]] = 0;
    CORRADE_VERIFY(abs(sdf_grid[coord[0]][coord[1]][coord[2]] - 0) < 0.1);
  }

  // test voxel set getters
  std::vector<Mn::Vector3i> voxelSet =
      voxelization->getVoxelSetFromFloatGrid("EuclideanSDF", -5, -4);
  bool voxelsAreInValidRange = true;
  for (auto ind : voxelSet) {
    if (!(sdf_grid[ind[0]][ind[1]][ind[2]] <= -4 &&
          sdf_grid[ind[0]][ind[1]][ind[2]] >= -5)) {
      voxelsAreInValidRange = false;
    }
  }
  CORRADE_VERIFY(voxelSet.size() == 18291);
  CORRADE_VERIFY(voxelsAreInValidRange);

  // Ensure mesh generation & mesh visualization doesn't crash simulator
  voxelization->generateMesh("Boundary");
  voxelization->generateMesh("GradientField");
  voxelization->generateSliceMesh("EuclideanSDF", 0, -10, 0);

  // Only one mesh can be visualized at a time
  simulator_->setSceneVoxelizationDraw(true, "Boundary");
  simulator_->setSceneVoxelizationDraw(true, "GradientField");
  simulator_->setSceneVoxelizationDraw(true, "EuclideanSDF");

  // Turn off visualization
  simulator_->setSceneVoxelizationDraw(false, "EuclideanSDF");
}
#endif

CORRADE_TEST_MAIN(VoxelGrid)
