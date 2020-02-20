// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <string>

#include "esp/sim/SimulatorWithAgents.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::agent::AgentConfiguration;
using esp::gfx::LightInfo;
using esp::gfx::LightPositionModel;
using esp::gfx::LightSetup;
using esp::nav::PathFinder;
using esp::scene::SceneConfiguration;
using esp::sensor::Observation;
using esp::sensor::ObservationSpace;
using esp::sensor::ObservationSpaceType;
using esp::sensor::SensorSpec;
using esp::sensor::SensorType;
using esp::sim::SimulatorConfiguration;
using esp::sim::SimulatorWithAgents;

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace Magnum::Math::Literals;

const std::string vangogh =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/van-gogh-room.glb");
const std::string skokloster =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "habitat-test-scenes/skokloster-castle.glb");
const std::string planeScene =
    Cr::Utility::Directory::join(TEST_ASSETS, "scenes/plane.glb");
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(TEST_ASSETS, "testing.phys_scene_config.json");
const std::string screenshotDir =
    Cr::Utility::Directory::join(TEST_ASSETS, "screenshots/");

namespace {

struct SimTest : Cr::TestSuite::Tester {
  explicit SimTest();
  void checkPinholeCameraRGBAObservation(
      SimulatorWithAgents& sim,
      const std::string& groundTruthImageFile);

  SimulatorWithAgents::uptr getSimulator(const std::string& scene) {
    SimulatorConfiguration simConfig{};
    simConfig.scene.id = scene;
    simConfig.enablePhysics = true;
    simConfig.physicsConfigFile = physicsConfigFile;

    auto sim = SimulatorWithAgents::create_unique(simConfig);
    sim->setLightSetup(lightSetup1, "custom_lighting_1");
    sim->setLightSetup(lightSetup2, "custom_lighting_2");
    return sim;
  }

  void basic();
  void reconfigure();
  void reset();
  void getSceneRGBAObservation();
  void getDefaultLightingRGBAObservation();
  void getCustomLightingRGBAObservation();
  void updateLightSetupRGBAObservation();
  void updateObjectLightSetupRGBAObservation();
  void multipleLightingSetupsRGBAObservation();

  LightSetup lightSetup1{{Magnum::Vector3{0.0f, 1.5f, -0.2f}, 0xffffff_rgbf,
                          LightPositionModel::CAMERA}};
  LightSetup lightSetup2{{Magnum::Vector3{0.0f, 0.5f, 1.0f}, 0xffffff_rgbf,
                          LightPositionModel::CAMERA}};
};

SimTest::SimTest() {
  // clang-format off
  addTests({&SimTest::basic,
            &SimTest::reconfigure,
            &SimTest::reset,
            &SimTest::getSceneRGBAObservation,
            &SimTest::getDefaultLightingRGBAObservation,
            &SimTest::getCustomLightingRGBAObservation,
            &SimTest::updateLightSetupRGBAObservation,
            &SimTest::updateObjectLightSetupRGBAObservation,
            &SimTest::multipleLightingSetupsRGBAObservation});
  // clang-format on
}

void SimTest::basic() {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  CORRADE_VERIFY(pathfinder);
}

void SimTest::reconfigure() {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  simulator.reconfigure(cfg);
  CORRADE_VERIFY(pathfinder == simulator.getPathFinder());
  SimulatorConfiguration cfg2;
  cfg2.scene.id = skokloster;
  simulator.reconfigure(cfg2);
  CORRADE_VERIFY(pathfinder != simulator.getPathFinder());
}

void SimTest::reset() {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  SimulatorWithAgents simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  simulator.reset();
  CORRADE_VERIFY(pathfinder == simulator.getPathFinder());
}

void SimTest::checkPinholeCameraRGBAObservation(
    SimulatorWithAgents& simulator,
    const std::string& groundTruthImageFile) {
  // do not rely on default SensorSpec default constructor to remain constant
  auto pinholeCameraSpec = SensorSpec::create();
  pinholeCameraSpec->sensorSubtype = "pinhole";
  pinholeCameraSpec->sensorType = SensorType::COLOR;
  pinholeCameraSpec->position = {1.0f, 1.5f, 1.0f};
  pinholeCameraSpec->resolution = {256, 256};

  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  simulator.addAgent(agentConfig);

  Observation observation;
  ObservationSpace obsSpace;
  CORRADE_VERIFY(
      simulator.getAgentObservation(0, pinholeCameraSpec->uuid, observation));
  CORRADE_VERIFY(
      simulator.getAgentObservationSpace(0, pinholeCameraSpec->uuid, obsSpace));

  std::vector<size_t> expectedShape{
      {static_cast<size_t>(pinholeCameraSpec->resolution[0]),
       static_cast<size_t>(pinholeCameraSpec->resolution[1]), 4}};

  CORRADE_VERIFY(obsSpace.spaceType == ObservationSpaceType::TENSOR);
  CORRADE_VERIFY(obsSpace.dataType == esp::core::DataType::DT_UINT8);
  CORRADE_COMPARE(obsSpace.shape, expectedShape);
  CORRADE_COMPARE(observation.buffer->shape, expectedShape);

  // Compare with previously rendered ground truth
  CORRADE_COMPARE_WITH(
      (Mn::ImageView2D{
          Mn::PixelFormat::RGBA8Unorm,
          {pinholeCameraSpec->resolution[0], pinholeCameraSpec->resolution[1]},
          observation.buffer->data}),
      Cr::Utility::Directory::join(screenshotDir, groundTruthImageFile),
      (Mn::DebugTools::CompareImageToFile{30.f, 0.5f}));
}

void SimTest::getSceneRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);
  checkPinholeCameraRGBAObservation(*simulator, "SimTestExpectedScene.png");
}

void SimTest::getDefaultLightingRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedDefaultLighting.png");
}

void SimTest::getCustomLightingRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting.png");
}

void SimTest::updateLightSetupRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);

  // update default lighting
  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedDefaultLighting.png");

  simulator->setLightSetup(lightSetup1);
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting.png");
  simulator->removeObject(objectID);

  // update custom lighting
  objectID = simulator->addObject(0, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting.png");

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting2.png");
}

void SimTest::updateObjectLightSetupRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedDefaultLighting.png");

  // change from default lighting to custom
  simulator->setObjectLightSetup(objectID, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting.png");

  // change from one custom lighting to another
  simulator->setObjectLightSetup(objectID, "custom_lighting_2");
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedCustomLighting2.png");
}

void SimTest::multipleLightingSetupsRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(planeScene);

  // make sure updates apply to all objects using the light setup
  int objectID = simulator->addObject(0, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({0.0f, 0.5f, -0.5f}, objectID);

  int otherObjectID = simulator->addObject(0, "custom_lighting_1");
  CORRADE_VERIFY(otherObjectID != esp::ID_UNDEFINED);
  simulator->setTranslation({2.0f, 0.5f, -0.5f}, otherObjectID);

  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedSameLighting.png");

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedSameLighting2.png");
  simulator->setLightSetup(lightSetup1, "custom_lighting_1");

  // make sure we can move a single object to another group
  simulator->setObjectLightSetup(objectID, "custom_lighting_2");
  checkPinholeCameraRGBAObservation(*simulator,
                                    "SimTestExpectedDifferentLighting.png");
}

}  // namespace

CORRADE_TEST_MAIN(SimTest)
