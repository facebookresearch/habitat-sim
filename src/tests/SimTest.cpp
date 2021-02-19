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
#include "esp/sensor/CameraSensor.h"
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
using esp::metadata::MetadataMediator;
using esp::metadata::attributes::AbstractPrimitiveAttributes;
using esp::metadata::attributes::ObjectAttributes;
using esp::nav::PathFinder;
using esp::sensor::CameraSensor;
using esp::sensor::CameraSensorSpec;
using esp::sensor::Observation;
using esp::sensor::ObservationSpace;
using esp::sensor::ObservationSpaceType;
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
const std::string planeStage =
    Cr::Utility::Directory::join(TEST_ASSETS, "scenes/plane.glb");
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(TEST_ASSETS, "testing.physics_config.json");
const std::string screenshotDir =
    Cr::Utility::Directory::join(TEST_ASSETS, "screenshots/");

struct SimTest : Cr::TestSuite::Tester {
  explicit SimTest();

  static Simulator::uptr getSimulator(
      SimTest& self,
      const std::string& scene,
      const std::string& sceneLightingKey = esp::NO_LIGHT_KEY) {
    SimulatorConfiguration simConfig{};
    simConfig.activeSceneName = scene;
    simConfig.enablePhysics = true;
    simConfig.physicsConfigFile = physicsConfigFile;
    simConfig.overrideSceneLightDefaults = true;
    simConfig.sceneLightSetup = sceneLightingKey;

    auto sim = Simulator::create_unique(simConfig);
    auto objAttrMgr = sim->getObjectAttributesManager();
    objAttrMgr->loadAllConfigsFromPath(
        Cr::Utility::Directory::join(TEST_ASSETS, "objects/nested_box"), true);

    sim->setLightSetup(self.lightSetup1, "custom_lighting_1");
    sim->setLightSetup(self.lightSetup2, "custom_lighting_2");
    return sim;
  }

  //! build a simulator via an instanced Metadata Mediator
  static Simulator::uptr getSimulatorMM(
      SimTest& self,
      const std::string& scene,
      const std::string& sceneLightingKey = esp::NO_LIGHT_KEY) {
    SimulatorConfiguration simConfig{};
    simConfig.activeSceneName = scene;
    simConfig.enablePhysics = true;
    simConfig.physicsConfigFile = physicsConfigFile;
    simConfig.overrideSceneLightDefaults = true;
    simConfig.sceneLightSetup = sceneLightingKey;

    MetadataMediator::ptr MM = MetadataMediator::create(simConfig);
    auto sim = Simulator::create_unique(simConfig, MM);
    auto objAttrMgr = sim->getObjectAttributesManager();
    objAttrMgr->loadAllConfigsFromPath(
        Cr::Utility::Directory::join(TEST_ASSETS, "objects/nested_box"), true);

    sim->setLightSetup(self.lightSetup1, "custom_lighting_1");
    sim->setLightSetup(self.lightSetup2, "custom_lighting_2");
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
  void buildingPrimAssetObjectTemplates();
  void addSensorToObject();

  // TODO: remove outlier pixels from image and lower maxThreshold
  const Magnum::Float maxThreshold = 255.f;

  LightSetup lightSetup1{{Magnum::Vector4{1.0f, 1.5f, 0.5f, 0.0f},
                          {5.0, 5.0, 0.0},
                          LightPositionModel::CAMERA}};
  LightSetup lightSetup2{{Magnum::Vector4{0.0f, 0.5f, 1.0f, 0.0f},
                          {0.0, 5.0, 5.0},
                          LightPositionModel::CAMERA}};
};
struct {
  // display name for sim being tested
  const char* name;
  // function pointer to constructor to simulator
  Simulator::uptr (*creator)(SimTest& self,
                             const std::string& scene,
                             const std::string& sceneLightingKey);

} SimulatorBuilder[]{{"built with SimConfig", &SimTest::getSimulator},
                     {"built with MetadataMediator", &SimTest::getSimulatorMM}};
SimTest::SimTest() {
  // clang-format off
  addTests({&SimTest::basic,
            &SimTest::reconfigure,
            &SimTest::reset});
            //test instances test both mechanisms for constructing simulator
  addInstancedTests({
            &SimTest::getSceneRGBAObservation,
            &SimTest::getSceneWithLightingRGBAObservation,
            &SimTest::getDefaultLightingRGBAObservation,
            &SimTest::getCustomLightingRGBAObservation,
            &SimTest::updateLightSetupRGBAObservation,
            &SimTest::updateObjectLightSetupRGBAObservation,
            &SimTest::multipleLightingSetupsRGBAObservation,
            &SimTest::recomputeNavmeshWithStaticObjects,
            &SimTest::loadingObjectTemplates,
            &SimTest::buildingPrimAssetObjectTemplates,
            &SimTest::addSensorToObject}, Cr::Containers::arraySize(SimulatorBuilder) );
  // clang-format on
}

void SimTest::basic() {
  SimulatorConfiguration cfg;
  cfg.activeSceneName = vangogh;
  Simulator simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  CORRADE_VERIFY(pathfinder);

  // test for MM ctor
  SimulatorConfiguration cfg_mm;
  cfg_mm.activeSceneName = vangogh;
  MetadataMediator::ptr MM = MetadataMediator::create(cfg_mm);
  Simulator simulator_mm(cfg_mm, MM);
  PathFinder::ptr pathfinder_mm = simulator_mm.getPathFinder();
  CORRADE_VERIFY(pathfinder_mm);
}

void SimTest::reconfigure() {
  SimulatorConfiguration cfg;
  cfg.activeSceneName = vangogh;
  Simulator simulator(cfg);
  PathFinder::ptr pathfinder = simulator.getPathFinder();
  simulator.reconfigure(cfg);
  CORRADE_VERIFY(pathfinder == simulator.getPathFinder());
  SimulatorConfiguration cfg2;
  cfg2.activeSceneName = skokloster;
  simulator.reconfigure(cfg2);
  CORRADE_VERIFY(pathfinder != simulator.getPathFinder());

  // test using MM ctor
  SimulatorConfiguration cfg_mm;
  cfg_mm.activeSceneName = vangogh;
  MetadataMediator::ptr MM = MetadataMediator::create(cfg_mm);
  Simulator simulator_mm(cfg_mm, MM);
  PathFinder::ptr pathfinder_mm = simulator_mm.getPathFinder();
  simulator_mm.reconfigure(cfg_mm);
  CORRADE_VERIFY(pathfinder_mm == simulator_mm.getPathFinder());
  SimulatorConfiguration cfg2_mm;
  cfg2_mm.activeSceneName = skokloster;
  simulator_mm.reconfigure(cfg2_mm);
  CORRADE_VERIFY(pathfinder_mm != simulator_mm.getPathFinder());
}

void SimTest::reset() {
  CORRADE_VERIFY(true);
  auto testReset = [&](Simulator& simulator) {
    PathFinder::ptr pathfinder = simulator.getPathFinder();
    auto pinholeCameraSpec = CameraSensorSpec::create();
    pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
    pinholeCameraSpec->sensorType = SensorType::Color;
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
  };

  SimulatorConfiguration cfg;
  cfg.activeSceneName = vangogh;
  Simulator simulator(cfg);
  testReset(simulator);
  // build simulator with MM

  SimulatorConfiguration cfg_mm;
  cfg_mm.activeSceneName = vangogh;
  MetadataMediator::ptr MM = MetadataMediator::create(cfg_mm);
  Simulator simulator_mm(cfg_mm, MM);
  testReset(simulator_mm);
}

void SimTest::checkPinholeCameraRGBAObservation(
    Simulator& simulator,
    const std::string& groundTruthImageFile,
    Magnum::Float maxThreshold,
    Magnum::Float meanThreshold) {
  // do not rely on default SensorSpec default constructor to remain constant
  auto pinholeCameraSpec = CameraSensorSpec::create();
  pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
  pinholeCameraSpec->sensorType = SensorType::Color;
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

  CORRADE_VERIFY(obsSpace.spaceType == ObservationSpaceType::Tensor);
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
  Corrade::Utility::Debug() << "Starting Test : getSceneRGBAObservation ";
  setTestCaseName(CORRADE_FUNCTION);
  Corrade::Utility::Debug() << "About to build simulator";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  Corrade::Utility::Debug() << "Built simulator";
  checkPinholeCameraRGBAObservation(*simulator, "SimTestExpectedScene.png",
                                    maxThreshold, 0.75f);
}

void SimTest::getSceneWithLightingRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : getSceneWithLightingRGBAObservation ";
  setTestCaseName(CORRADE_FUNCTION);
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSceneWithLighting.png", maxThreshold, 0.75f);
}

void SimTest::getDefaultLightingRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : getDefaultLightingRGBAObservation ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID = simulator->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);
}

void SimTest::getCustomLightingRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : getCustomLightingRGBAObservation ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID =
      simulator->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);
}

void SimTest::updateLightSetupRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : updateLightSetupRGBAObservation ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  // update default lighting
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID = simulator->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);

  simulator->setLightSetup(lightSetup1);
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);
  simulator->removeObject(objectID);

  // update custom lighting
  objectID =
      simulator->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.71f);
}

void SimTest::updateObjectLightSetupRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : updateObjectLightSetupRGBAObservation ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID = simulator->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID);
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);

  // change from default lighting to custom
  simulator->setObjectLightSetup(objectID, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);

  // change from one custom lighting to another
  simulator->setObjectLightSetup(objectID, "custom_lighting_2");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.71f);
}

void SimTest::multipleLightingSetupsRGBAObservation() {
  Corrade::Utility::Debug()
      << "Starting Test : multipleLightingSetupsRGBAObservation ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  // make sure updates apply to all objects using the light setup
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID =
      simulator->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  simulator->setTranslation({0.0f, 0.5f, -0.5f}, objectID);

  int otherObjectID =
      simulator->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
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
  Corrade::Utility::Debug()
      << "Starting Test : recomputeNavmeshWithStaticObjects ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, skokloster, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();

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
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID = simulator->addObjectByHandle(objs[0]);
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
  ObjectAttributes::ptr objectTemplate = objectAttribsMgr->getObjectCopyByID(0);
  objectTemplate->setScale({0.5, 0.5, 0.5});
  int tmplateID = objectAttribsMgr->registerObject(objectTemplate);

  objectID = simulator->addObjectByHandle(objs[0]);
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
  Corrade::Utility::Debug() << "Starting Test : loadingObjectTemplates ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();

  // test directory of templates
  std::vector<int> templateIndices = objectAttribsMgr->loadAllConfigsFromPath(
      Cr::Utility::Directory::join(TEST_ASSETS, "objects"));
  CORRADE_VERIFY(!templateIndices.empty());
  for (auto index : templateIndices) {
    CORRADE_VERIFY(index != esp::ID_UNDEFINED);
  }

  // reload again and ensure that old loaded indices are returned
  std::vector<int> templateIndices2 = objectAttribsMgr->loadAllConfigsFromPath(
      Cr::Utility::Directory::join(TEST_ASSETS, "objects"));
  CORRADE_VERIFY(templateIndices2 == templateIndices);

  // test the loaded assets and accessing them by name
  // verify that getting the template handles with empty string returns all
  int numLoadedTemplates = templateIndices2.size();
  std::vector<std::string> templateHandles =
      objectAttribsMgr->getFileTemplateHandlesBySubstring();
  CORRADE_VERIFY(numLoadedTemplates == templateHandles.size());

  // verify that querying with sub string returns template handle corresponding
  // to that substring
  // get full handle of an existing template
  std::string fullTmpHndl = templateHandles[templateHandles.size() - 1];
  // build substring
  std::div_t len = std::div(fullTmpHndl.length(), 2);
  // get 2nd half of handle
  std::string tmpHndl = fullTmpHndl.substr(len.quot);
  // get all handles that match 2nd half of known handle
  std::vector<std::string> matchTmpltHandles =
      objectAttribsMgr->getObjectHandlesBySubstring(tmpHndl);
  CORRADE_VERIFY(matchTmpltHandles[0] == fullTmpHndl);

  // test fresh template as smart pointer
  ObjectAttributes::ptr newTemplate =
      objectAttribsMgr->createObject("new template", false);
  std::string boxPath =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/transform_box.glb");
  newTemplate->setRenderAssetHandle(boxPath);
  int templateIndex = objectAttribsMgr->registerObject(newTemplate, boxPath);

  CORRADE_VERIFY(templateIndex != esp::ID_UNDEFINED);
  // change render asset for object template named boxPath
  std::string chairPath =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/chair.glb");
  newTemplate->setRenderAssetHandle(chairPath);
  int templateIndex2 = objectAttribsMgr->registerObject(newTemplate, boxPath);

  CORRADE_VERIFY(templateIndex2 != esp::ID_UNDEFINED);
  CORRADE_VERIFY(templateIndex2 == templateIndex);
  ObjectAttributes::ptr newTemplate2 =
      objectAttribsMgr->getObjectCopyByHandle(boxPath);
  CORRADE_VERIFY(newTemplate2->getRenderAssetHandle() == chairPath);
}

void SimTest::buildingPrimAssetObjectTemplates() {
  Corrade::Utility::Debug()
      << "Starting Test : buildingPrimAssetObjectTemplates ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, esp::NO_LIGHT_KEY);

  // test that the correct number of default primitive assets are available as
  // render/collision targets
  // manager of asset attributes
  auto assetAttribsMgr = simulator->getAssetAttributesManager();
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();

  // get all handles of templates for primitive-based render objects
  std::vector<std::string> primObjAssetHandles =
      objectAttribsMgr->getSynthTemplateHandlesBySubstring("");

  // there should be 1 prim template per default primitive asset template
  int numPrimsExpected =
      static_cast<int>(esp::metadata::PrimObjTypes::END_PRIM_OBJ_TYPES);
  // verify the number of primitive templates
  CORRADE_VERIFY(numPrimsExpected == primObjAssetHandles.size());

  AbstractPrimitiveAttributes::ptr primAttr;
  {
    // test that there are existing templates for each key, and that they have
    // valid values to be used to construct magnum primitives
    for (int i = 0; i < numPrimsExpected; ++i) {
      std::string handle = primObjAssetHandles[i];
      CORRADE_VERIFY(handle != "");
      primAttr = assetAttribsMgr->getObjectCopyByHandle(handle);
      CORRADE_VERIFY(primAttr != nullptr);
      CORRADE_VERIFY(primAttr->isValidTemplate());
      // verify that the attributes contains the handle, and the handle contains
      // the expected class name
      std::string className =
          esp::metadata::managers::AssetAttributesManager::PrimitiveNames3DMap
              .at(static_cast<esp::metadata::PrimObjTypes>(i));
      CORRADE_VERIFY((primAttr->getHandle() == handle) &&
                     (handle.find(className) != std::string::npos));
    }
  }
  // empty vector of handles
  primObjAssetHandles.clear();
  {
    // test that existing template handles can be accessed via name string.
    // This access is case insensitive
    primObjAssetHandles =
        objectAttribsMgr->getSynthTemplateHandlesBySubstring("CONESOLID");
    // should only be one handle in this vector
    CORRADE_VERIFY(1 == primObjAssetHandles.size());
    // handle should not be empty and be long enough to hold class name prefix
    CORRADE_VERIFY(9 < primObjAssetHandles[0].length());
    // coneSolid should appear in handle
    std::string checkStr("coneSolid");
    CORRADE_VERIFY(primObjAssetHandles[0].find(checkStr) != std::string::npos);
    // empty vector of handles
    primObjAssetHandles.clear();

    // test that existing template handles can be accessed through exclusion -
    // all but certain string.  This access is case insensitive
    primObjAssetHandles = objectAttribsMgr->getSynthTemplateHandlesBySubstring(
        "CONESOLID", false);
    // should be all handles but coneSolid handle here
    CORRADE_VERIFY((numPrimsExpected - 1) == primObjAssetHandles.size());
    for (auto primObjAssetHandle : primObjAssetHandles) {
      CORRADE_VERIFY(primObjAssetHandle.find(checkStr) == std::string::npos);
    }
  }
  // empty vector of handles
  primObjAssetHandles.clear();

  // test that primitive asset attributes are able to be modified and saved and
  // the changes persist, while the old templates are not removed
  {
    // get existing default cylinder handle
    primObjAssetHandles = assetAttribsMgr->getTemplateHandlesByPrimType(
        esp::metadata::PrimObjTypes::CYLINDER_SOLID);
    // should only be one handle in this vector
    CORRADE_VERIFY(1 == primObjAssetHandles.size());
    // primitive render object uses primitive render asset as handle
    std::string origCylinderHandle = primObjAssetHandles[0];
    primAttr = assetAttribsMgr->getObjectCopyByHandle(origCylinderHandle);
    // verify that the origin handle matches what is expected
    CORRADE_VERIFY(primAttr->getHandle() == origCylinderHandle);
    // get original number of rings for this cylinder
    int origNumRings = primAttr->getNumRings();
    // modify attributes - this will change handle
    primAttr->setNumRings(2 * origNumRings);
    // verify that internal name of attributes has changed due to essential
    // quantity being modified
    std::string newHandle = primAttr->getHandle();
    CORRADE_VERIFY(newHandle != origCylinderHandle);
    // set test label, to validate that copy is reggistered
    primAttr->setString("test", "test0");
    // register new attributes
    int idx = assetAttribsMgr->registerObject(primAttr);
    CORRADE_VERIFY(idx != esp::ID_UNDEFINED);
    // set new test label, to validate against retrieved copy
    primAttr->setString("test", "test1");
    // retrieve registered attributes copy
    AbstractPrimitiveAttributes::ptr primAttr2 =
        assetAttribsMgr->getObjectCopyByHandle(newHandle);
    // verify pre-reg and post-reg are named the same
    CORRADE_VERIFY(primAttr->getHandle() == primAttr2->getHandle());
    // verify retrieved attributes is copy, not original
    CORRADE_VERIFY(primAttr->getString("test") != primAttr2->getString("test"));
    // remove modified attributes
    AbstractPrimitiveAttributes::ptr primAttr3 =
        assetAttribsMgr->removeObjectByHandle(newHandle);
    CORRADE_VERIFY(nullptr != primAttr3);
  }
  // empty vector of handles
  primObjAssetHandles.clear();
  {
    // test creation of new object, using edited attributes
    // get existing default cylinder handle
    primObjAssetHandles = assetAttribsMgr->getTemplateHandlesByPrimType(
        esp::metadata::PrimObjTypes::CYLINDER_SOLID);
    // primitive render object uses primitive render asset as handle
    std::string origCylinderHandle = primObjAssetHandles[0];
    primAttr = assetAttribsMgr->getObjectCopyByHandle(origCylinderHandle);
    // modify attributes - this will change handle
    primAttr->setNumRings(2 * primAttr->getNumRings());
    // verify that internal name of attributes has changed due to essential
    // quantity being modified
    std::string newHandle = primAttr->getHandle();
    // register new attributes
    int idx = assetAttribsMgr->registerObject(primAttr);

    // create object template with modified primitive asset attributes, by
    // passing handle.  defaults to register object template
    auto newCylObjAttr = objectAttribsMgr->createObject(newHandle);
    CORRADE_VERIFY(nullptr != newCylObjAttr);
    // create object with new attributes
    int objectID = simulator->addObjectByHandle(newHandle);
    CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);
  }
  // empty vector of handles
  primObjAssetHandles.clear();

}  // SimTest::buildingPrimAssetObjectTemplates

void SimTest::addSensorToObject() {
  Corrade::Utility::Debug() << "Starting Test : addSensorToObject ";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("sphere");
  int objectID = simulator->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(objectID != esp::ID_UNDEFINED);

  // Add sensor to sphere object
  esp::sensor::SensorSuite sensorSuite;
  auto objectSensorSpec = esp::sensor::CameraSensorSpec::create();
  objectSensorSpec->uuid = std::to_string(objectID);
  objectSensorSpec->position = {0, 0, 0};
  objectSensorSpec->orientation = {0, 0, 0};
  objectSensorSpec->resolution = {128, 128};
  sensorSuite.add(simulator->addSensorToObject(objectID, objectSensorSpec));
  std::string expectedUUID = std::to_string(objectID);
  CORRADE_VERIFY(
      sensorSuite.get(expectedUUID));  // Verify that Sensor exists with uuid
  CameraSensor* cameraSensor =
      dynamic_cast<CameraSensor*>(sensorSuite.get(expectedUUID).get());
  cameraSensor->setTransformationFromSpec();

  simulator->setTranslation({1.0f, 1.5f, 1.0f},
                            objectID);  // Move camera to same place as agent

  auto objs2 = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  int objectID2 = simulator->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(objectID2 != esp::ID_UNDEFINED);
  simulator->setTranslation({1.0f, 0.5f, -0.5f}, objectID2);

  Observation observation;
  ObservationSpace obsSpace;
  simulator->getRenderer()->bindRenderTarget(*cameraSensor);
  CORRADE_VERIFY(cameraSensor->getObservation(*simulator, observation));
  CORRADE_VERIFY(cameraSensor->getObservationSpace(obsSpace));

  esp::vec2i defaultResolution = {128, 128};
  std::vector<size_t> expectedShape{{static_cast<size_t>(defaultResolution[0]),
                                     static_cast<size_t>(defaultResolution[1]),
                                     4}};

  CORRADE_VERIFY(obsSpace.spaceType == ObservationSpaceType::Tensor);
  CORRADE_VERIFY(obsSpace.dataType == esp::core::DataType::DT_UINT8);
  CORRADE_COMPARE(obsSpace.shape, expectedShape);
  CORRADE_COMPARE(observation.buffer->shape, expectedShape);

  // Compare with previously rendered ground truth
  // Object camera at same location as agent camera should render similar image
  CORRADE_COMPARE_WITH(
      (Mn::ImageView2D{Mn::PixelFormat::RGBA8Unorm,
                       {defaultResolution[0], defaultResolution[1]},
                       observation.buffer->data}),
      Cr::Utility::Directory::join(screenshotDir, "SimTestExpectedScene.png"),
      (Mn::DebugTools::CompareImageToFile{maxThreshold, 0.75f}));
}
}  // namespace

CORRADE_TEST_MAIN(SimTest)
