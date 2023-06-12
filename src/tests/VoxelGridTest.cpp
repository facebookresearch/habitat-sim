// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/Magnum.h>

#include "esp/geo/VoxelUtils.h"
#include "esp/geo/VoxelWrapper.h"
#include "esp/sim/Simulator.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

struct VoxelGridTest : Cr::TestSuite::Tester {
  explicit VoxelGridTest();

  void testVoxelUtilityFunctions();

  esp::logging::LoggingContext loggingContext_;
};

VoxelGridTest::VoxelGridTest() {
  addTests({&VoxelGridTest::testVoxelUtilityFunctions});
}

void VoxelGridTest::testVoxelUtilityFunctions() {
  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.glb");
  simConfig.enablePhysics = true;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;

  auto simulator_ = esp::sim::Simulator::create_unique(simConfig);

  // Voxelize the scene with resolution = 1,000,000 and make asserts
  const int resolution = 1000000;
  simulator_->createStageVoxelization(resolution);
  auto voxelization = simulator_->getStageVoxelization();

  // Generate Euclidean and Manhattan SDF for the voxelization
  esp::geo::generateEuclideanDistanceSDF(voxelization, "ESDF");
  esp::geo::generateManhattanDistanceSDF(voxelization, "MSDF");

  // "Golden Value" tests to ensure the SDF functions work correctly
  std::vector<Mn::Vector3i> voxel_indices = std::vector<Mn::Vector3i>{
      Mn::Vector3i(0, 0, 0),
      voxelization->getVoxelGridDimensions() - Mn::Vector3i(1, 1, 1),
      Mn::Vector3i(10, 5, 4),
      Mn::Vector3i(4, 10, 5),
      Mn::Vector3i(5, 4, 10),
      Mn::Vector3i(30, 30, 30),
      Mn::Vector3i(22, 14, 30),
      Mn::Vector3i(20, 12, 23)};

  auto esdf_grid = voxelization->getGrid<float>("ESDF");
  auto msdf_grid = voxelization->getGrid<int>("MSDF");

  std::vector<float> correct_esdf_values =
      std::vector<float>{7, 1.73205, 3, 2, -3, 1, -12, -6.16441};
  std::vector<int> correct_msdf_values =
      std::vector<int>{7, 3, 3, 2, -3, 1, -12, -8};

  bool esdf_values_are_correct = true;
  bool msdf_values_are_correct = true;
  // tolerance for comparing ESDF values
  float tolerance = 0.00001;

  for (int i = 0; i < voxel_indices.size(); ++i) {
    auto& ind = voxel_indices[i];
    if (abs(esdf_grid[ind[0]][ind[1]][ind[2]] - correct_esdf_values[i]) >
        tolerance) {
      esdf_values_are_correct = false;
    }
    if (msdf_grid[ind[0]][ind[1]][ind[2]] != correct_msdf_values[i]) {
      msdf_values_are_correct = false;
    }
  }

  CORRADE_VERIFY(esdf_values_are_correct);
  CORRADE_VERIFY(msdf_values_are_correct);

  // Get a vector of indices who's SDF value lies between -13 and -12
  std::vector<Mn::Vector3i> indices =
      esp::geo::getVoxelSetFromFloatGrid(voxelization, "ESDF", -13, -12);

  // ensure that the values were correctly retrieved.

  bool valuesAreInRange = true;
  for (auto& ind : indices) {
    float val = voxelization->getVoxel<float>(ind, "ESDF");
    if (val > -12 || val < -13) {
      valuesAreInRange = false;
    }
  }
  CORRADE_VERIFY(valuesAreInRange);
}
}  // namespace

CORRADE_TEST_MAIN(VoxelGridTest)
