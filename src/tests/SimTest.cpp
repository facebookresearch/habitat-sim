// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/physics/RigidObject.h"
#include "esp/sim/Simulator.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::agent::Agent;
using esp::agent::AgentConfiguration;
using esp::agent::AgentState;
using esp::assets::ResourceManager;
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
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

namespace {

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

struct SimTest : Cr::TestSuite::Tester {
  explicit SimTest();

  Simulator::uptr getSimulator(
      const std::string& scene,
      const std::string& sceneLightingKey = ResourceManager::NO_LIGHT_KEY) {
    SimulatorConfiguration simConfig{};
    simConfig.scene.id = scene;
    simConfig.enablePhysics = true;
    simConfig.physicsConfigFile = physicsConfigFile;
    simConfig.sceneLightSetup = sceneLightingKey;

    auto sim = Simulator::create_unique(simConfig);
    sim->setLightSetup(lightSetup1, "custom_lighting_1");
    sim->setLightSetup(lightSetup2, "custom_lighting_2");
    return sim;
  }

  void checkPinholeCameraRGBAObservation(
      Simulator& sim,
      const std::string& groundTruthImageFile,
      Magnum::Float maxThreshold,
      Magnum::Float meanThreshold);

  void basic();
  void reconfigure();
  void reset();
  void getSceneRGBAObservation();
  void getSceneWithLightingRGBAObservation();
  void getDefaultLightingRGBAObservation();
  void getCustomLightingRGBAObservation();
  void updateLightSetupRGBAObservation();
  void updateObjectLightSetupRGBAObservation();
  void multipleLightingSetupsRGBAObservation();
  void recomputeNavmeshWithStaticObjects();
  void loadingObjectTemplates();

  // TODO: remove outlier pixels from image and lower maxThreshold
  const Magnum::Float maxThreshold = 255.f;

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
            &SimTest::getSceneWithLightingRGBAObservation,
            &SimTest::getDefaultLightingRGBAObservation,
            &SimTest::getCustomLightingRGBAObservation,
            &SimTest::updateLightSetupRGBAObservation,
            &SimTest::updateObjectLightSetupRGBAObservation,
            &SimTest::multipleLightingSetupsRGBAObservation,
            &SimTest::recomputeNavmeshWithStaticObjects,
            &SimTest::loadingObjectTemplates});
  // clang-format on
}

void SimTest::basic() {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  Simulator simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  CORRADE_VERIFY(pathfinder);
}

void SimTest::reconfigure() {
  SimulatorConfiguration cfg;
  cfg.scene.id = vangogh;
  Simulator simulator(cfg);
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
  Simulator simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();

  auto pinholeCameraSpec = SensorSpec::create();
  pinholeCameraSpec->sensorSubtype = "pinhole";
  pinholeCameraSpec->sensorType = SensorType::COLOR;
  pinholeCameraSpec->position = {0.0f, 1.5f, 5.0f};
  pinholeCameraSpec->resolution = {100, 100};
  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  auto agent = simulator.addAgent(agentConfig);

  auto stateOrig = AgentState::create();
  agent->getState(stateOrig);

  simulator.reset();

  auto stateFinal = AgentState::create();
  agent->getState(stateFinal);
  CORRADE_VERIFY(stateOrig->position == stateFinal->position);
  CORRADE_VERIFY(stateOrig->rotation == stateFinal->rotation);
  CORRADE_VERIFY(pathfinder == simulator.getPathFinder());
}

void SimTest::checkPinholeCameraRGBAObservation(
    Simulator& simulator,
    const std::string& groundTruthImageFile,
    Magnum::Float maxThreshold,
    Magnum::Float meanThreshold) {
  // do not rely on default SensorSpec default constructor to remain constant
  auto pinholeCameraSpec = SensorSpec::create();
  pinholeCameraSpec->sensorSubtype = "pinhole";
  pinholeCameraSpec->sensorType = SensorType::COLOR;
  pinholeCameraSpec->position = {1.0f, 1.5f, 1.0f};
  pinholeCameraSpec->resolution = {128, 128};

  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  Agent::ptr agent = simulator.addAgent(agentConfig);
  agent->setInitialState(AgentState{});

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
      (Mn::DebugTools::CompareImageToFile{maxThreshold, meanThreshold}));
}

void SimTest::getSceneRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh);
  checkPinholeCameraRGBAObservation(*simulator, "SimTestExpectedScene.png",
                                    maxThreshold, 0.75f);
}

void SimTest::getSceneWithLightingRGBAObservation() {
  setTestCaseName(CORRADE_FUNCTION);
  auto simulator = getSimulator(vangogh, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSceneWithLighting.png", maxThreshold, 0.75f);
}

void SimTest::getDefaultLightingRGBAObservation() {
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.65f);
}

void SimTest::getCustomLightingRGBAObservation() {
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0, nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.65f);
}

void SimTest::updateLightSetupRGBAObservation() {
  auto simulator = getSimulator(vangogh);

  // update default lighting
  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.65f);

  simulator->setLightSetup(lightSetup1);
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.65f);
  simulator->removeObject(objectID);

  // update custom lighting
  objectID = simulator->addObject(0, nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.65f);

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.65f);
}

void SimTest::updateObjectLightSetupRGBAObservation() {
  auto simulator = getSimulator(vangogh);

  int objectID = simulator->addObject(0);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.65f);

  // change from default lighting to custom
  simulator->setObjectLightSetup(objectID, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.65f);

  // change from one custom lighting to another
  simulator->setObjectLightSetup(objectID, "custom_lighting_2");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.65f);
}

void SimTest::multipleLightingSetupsRGBAObservation() {
  auto simulator = getSimulator(planeScene);

  // make sure updates apply to all objects using the light setup
  int objectID = simulator->addObject(0, nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({0.0f, 0.5f, -0.5f}, objectID);

  int otherObjectID = simulator->addObject(0, nullptr, "custom_lighting_1");
  CORRADE_VERIFY(otherObjectID != esp::ID_UNDEFINED);
  simulator->setTranslation({2.0f, 0.5f, -0.5f}, otherObjectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSameLighting.png", maxThreshold, 0.01f);

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSameLighting2.png", maxThreshold, 0.01f);
  simulator->setLightSetup(lightSetup1, "custom_lighting_1");

  // make sure we can move a single object to another group
  simulator->setObjectLightSetup(objectID, "custom_lighting_2");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDifferentLighting.png", maxThreshold, 0.01f);
}

void SimTest::recomputeNavmeshWithStaticObjects() {
  auto simulator = getSimulator(skokloster);

  // compute the initial navmesh
  esp::nav::NavMeshSettings navMeshSettings;
  navMeshSettings.setDefaults();
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings);

  esp::vec3f randomNavPoint =
      simulator->getPathFinder()->getRandomNavigablePoint();
  while (simulator->getPathFinder()->distanceToClosestObstacle(randomNavPoint) <
             1.0 ||
         randomNavPoint[1] > 1.0) {
    randomNavPoint = simulator->getPathFinder()->getRandomNavigablePoint();
  }

  // add static object at a known navigable point
  int objectID = simulator->addObject(0);
  simulator->setTranslation(Magnum::Vector3{randomNavPoint}, objectID);
  simulator->setObjectMotionType(esp::physics::MotionType::STATIC, objectID);
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable({randomNavPoint}, 0.1));

  // recompute with object
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings, true);
  CORRADE_VERIFY(!simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));

  // recompute without again
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings, false);
  CORRADE_VERIFY(simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));

  simulator->removeObject(objectID);

  // test scaling
  esp::assets::PhysicsObjectAttributes& objectTemplate =
      simulator->getObjectTemplate(0);
  objectTemplate.setScale({0.5, 0.5, 0.5});
  objectID = simulator->addObject(0);
  simulator->setTranslation(Magnum::Vector3{randomNavPoint}, objectID);
  simulator->setTranslation(
      simulator->getTranslation(objectID) + Magnum::Vector3{0, 0.5, 0},
      objectID);
  simulator->setObjectMotionType(esp::physics::MotionType::STATIC, objectID);
  esp::vec3f offset(0.75, 0, 0);
  CORRADE_VERIFY(simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable(randomNavPoint + offset, 0.2));
  // recompute with object
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings, true);
  CORRADE_VERIFY(!simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable(randomNavPoint + offset, 0.2));
}

void SimTest::loadingObjectTemplates() {
  auto simulator = getSimulator(planeScene);

  // test directory of templates
  std::vector<int> templateIndices = simulator->loadObjectConfigs(
      Cr::Utility::Directory::join(TEST_ASSETS, "objects"));
  CORRADE_VERIFY(!templateIndices.empty());
  for (auto index : templateIndices) {
    CORRADE_VERIFY(index != esp::ID_UNDEFINED);
  }

  // reload again and ensure that old loaded indices are returned
  std::vector<int> templateIndices2 = simulator->loadObjectConfigs(
      Cr::Utility::Directory::join(TEST_ASSETS, "objects"));
  CORRADE_VERIFY(templateIndices2 == templateIndices);

  // test fresh template
  esp::assets::PhysicsObjectAttributes newTemplate;
  std::string boxPath =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/transform_box.glb");
  newTemplate.setRenderMeshHandle(boxPath);
  int templateIndex = simulator->loadObjectTemplate(newTemplate, boxPath);
  CORRADE_VERIFY(templateIndex != esp::ID_UNDEFINED);

  // test double load
  templateIndex = simulator->loadObjectTemplate(newTemplate, boxPath);
  CORRADE_VERIFY(templateIndex == esp::ID_UNDEFINED);
}

}  // namespace

CORRADE_TEST_MAIN(SimTest)
