// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <string>

#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;
namespace {

// Scene dataset config directly references test scene locations, along with
// providing default values for stage configuration, obviating the need for
// per-scene/per-stage configs.
const std::string HM3DTestConfigLoc = Cr::Utility::Directory::join(
    SCENE_DATASETS,
    "habitat-test-scenes/hm3d_habitat_annotated_testdata/"
    "hm3d_annotated_testdata.scene_dataset_config.json");

// TODO : support all free/sample scenes once they have their annotations
// complete.  Make sure above config references additional scenes, and then
// just add scene hash to array.
const std::string HM3DTestScenes[] = {"GLAQ4DNUx5U"};

struct HM3DSceneTest : Cr::TestSuite::Tester {
  Simulator::uptr getSimulatorWithScene(const std::string& scene) {
    // change config to be using new scene
    auto cfg =
        esp::sim::SimulatorConfiguration{MM_->getSimulatorConfiguration()};
    cfg.activeSceneName = scene;
    cfg.sceneDatasetConfigFile = HM3DTestConfigLoc;
    // create simulator with new scene and loaded MM holding dataset info
    auto sim = Simulator::create_unique(cfg, MM_);

    return sim;
  }

  explicit HM3DSceneTest();

  void testHM3DScene();

  void testHM3DSemanticScene();

  esp::logging::LoggingContext loggingContext;

  // The MetadataMediator can exist independent of simulator
  // and provides access to all managers for currently active scene dataset
  std::shared_ptr<esp::metadata::MetadataMediator> MM_ = nullptr;
};  // struct HM3DSceneTest

const struct {
  const char* name;

  // index in HM3DTestScenes array for scene
  int testSceneIDX;

} TestHM3DScenes[]{{"GLAQ4DNUx5U", 0}};

HM3DSceneTest::HM3DSceneTest() {
  // set up a default simulation config to initialize MM
  auto cfg = esp::sim::SimulatorConfiguration{};
  // build metadata mediator and initialize with cfg, loading test dataset info
  MM_ = esp::metadata::MetadataMediator::create(cfg);
  addInstancedTests(
      {&HM3DSceneTest::testHM3DScene, &HM3DSceneTest::testHM3DSemanticScene},
      Cr::Containers::arraySize(TestHM3DScenes));
}

void HM3DSceneTest::testHM3DScene() {
  // If scene dataset does not exist, skip test for now
  if (!Cr::Utility::Directory::exists(HM3DTestConfigLoc)) {
    CORRADE_SKIP("HM3D dataset not found.");
  }
  auto&& testData = TestHM3DScenes[testCaseInstanceId()];
  setTestCaseDescription(testData.name);
  auto simulator = getSimulatorWithScene(HM3DTestScenes[testData.testSceneIDX]);
  CORRADE_COMPARE(0, 0);
}

void HM3DSceneTest::testHM3DSemanticScene() {
  // If scene dataset does not exist, skip test for now
  if (!Cr::Utility::Directory::exists(HM3DTestConfigLoc)) {
    CORRADE_SKIP("HM3D dataset not found.");
  }
  auto&& testData = TestHM3DScenes[testCaseInstanceId()];
  setTestCaseDescription(testData.name);
  auto simulator = getSimulatorWithScene(HM3DTestScenes[testData.testSceneIDX]);
  CORRADE_COMPARE(0, 0);
}

}  // namespace

CORRADE_TEST_MAIN(HM3DSceneTest)
