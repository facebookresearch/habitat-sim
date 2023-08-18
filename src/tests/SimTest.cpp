// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/DebugTools/CompareImage.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <string>
#include <vector>

#include "esp/assets/Asset.h"
#include "esp/assets/ResourceManager.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/physics/RigidObject.h"
#include "esp/physics/objectManagers/ArticulatedObjectManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
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
using namespace Magnum::Math::Literals;

const std::string vangogh =
    Cr::Utility::Path::join(SCENE_DATASETS,
                            "habitat-test-scenes/van-gogh-room.glb");
const std::string skokloster =
    Cr::Utility::Path::join(SCENE_DATASETS,
                            "habitat-test-scenes/skokloster-castle.glb");
const std::string planeStage =
    Cr::Utility::Path::join(TEST_ASSETS, "scenes/plane.glb");
const std::string physicsConfigFile =
    Cr::Utility::Path::join(TEST_ASSETS, "testing.physics_config.json");
const std::string screenshotDir =
    Cr::Utility::Path::join(TEST_ASSETS, "screenshots/");

struct SimTest : Cr::TestSuite::Tester {
  explicit SimTest();

  //! build a simulator via a SimulatorConfiguration alone
  static Simulator::uptr getSimulator(
      SimTest& self,
      const std::string& scene,
      bool createRenderer,
      const std::string& sceneLightingKey = esp::NO_LIGHT_KEY) {
    SimulatorConfiguration simConfig{};
    simConfig.activeSceneName = scene;
    simConfig.enablePhysics =
#ifdef ESP_BUILD_WITH_BULLET
        true;
#else
        false;
#endif
    simConfig.physicsConfigFile = physicsConfigFile;
    simConfig.overrideSceneLightDefaults = true;
    simConfig.sceneLightSetupKey = sceneLightingKey;
    simConfig.createRenderer = createRenderer;

    auto sim = Simulator::create_unique(simConfig);
    auto objAttrMgr = sim->getObjectAttributesManager();
    objAttrMgr->loadAllJSONConfigsFromPath(
        Cr::Utility::Path::join(TEST_ASSETS, "objects/nested_box"), true);

    sim->setLightSetup(self.lightSetup1, "custom_lighting_1");
    sim->setLightSetup(self.lightSetup2, "custom_lighting_2");
    return sim;
  }

  //! build a simulator via an instanced Metadata Mediator
  static Simulator::uptr getSimulatorMM(
      SimTest& self,
      const std::string& scene,
      bool createRenderer,
      const std::string& sceneLightingKey = esp::NO_LIGHT_KEY) {
    SimulatorConfiguration simConfig{};
    simConfig.activeSceneName = scene;
    simConfig.enablePhysics =
#ifdef ESP_BUILD_WITH_BULLET
        true;
#else
        false;
#endif
    simConfig.physicsConfigFile = physicsConfigFile;
    simConfig.overrideSceneLightDefaults = true;
    simConfig.sceneLightSetupKey = sceneLightingKey;
    simConfig.createRenderer = createRenderer;

    MetadataMediator::ptr MM = MetadataMediator::create(simConfig);
    auto sim = Simulator::create_unique(simConfig, MM);
    auto objAttrMgr = sim->getObjectAttributesManager();
    objAttrMgr->loadAllJSONConfigsFromPath(
        Cr::Utility::Path::join(TEST_ASSETS, "objects/nested_box"), true);

    sim->setLightSetup(self.lightSetup1, "custom_lighting_1");
    sim->setLightSetup(self.lightSetup2, "custom_lighting_2");
    return sim;
  }
  void checkPinholeCameraRGBAObservation(
      Simulator& sim,
      const std::string& groundTruthImageFile,
      Magnum::Float maxThreshold,
      Magnum::Float meanThreshold);

  void addObjectsAndMakeObservation(Simulator& sim,
                                    esp::sensor::CameraSensorSpec& cameraSpec,
                                    const std::string& objTmpltHandle,
                                    Observation& observation);

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
  void addObjectByHandle();
  void addObjectInvertedScale();
  void addSensorToObject();
  void createMagnumRenderingOff();
  void getRuntimePerfStats();
  void testArticulatedObjectSkinned();

  esp::logging::LoggingContext loggingContext_;
  // TODO: remove outlier pixels from image and lower maxThreshold
  const Magnum::Float maxThreshold = 255.f;

  LightSetup lightSetup1{{Magnum::Vector4{-1.0f, -1.5f, -0.5f, 0.0f},
                          {5.0, 5.0, 0.0},
                          LightPositionModel::Camera}};
  LightSetup lightSetup2{{Magnum::Vector4{0.0f, -0.5f, -1.0f, 0.0f},
                          {0.0, 5.0, 5.0},
                          LightPositionModel::Camera}};
};  // struct SimTest

struct {
  // display name for sim being tested
  const char* name;
  // function pointer to constructor to simulator
  Simulator::uptr (*creator)(SimTest& self,
                             const std::string& scene,
                             bool createRenderer,
                             const std::string& sceneLightingKey);

} SimulatorBuilder[]{{"built with SimConfig", &SimTest::getSimulator},
                     {"built with MetadataMediator", &SimTest::getSimulatorMM}};
SimTest::SimTest() {
  // clang-format off
  //test instances test both mechanisms for constructing simulator
  addInstancedTests({
            &SimTest::basic,
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
            &SimTest::loadingObjectTemplates,
            &SimTest::buildingPrimAssetObjectTemplates,
            &SimTest::addObjectByHandle,
            &SimTest::addObjectInvertedScale,
            &SimTest::addSensorToObject,
            &SimTest::getRuntimePerfStats,
#ifdef ESP_BUILD_WITH_BULLET
            &SimTest::createMagnumRenderingOff,
            &SimTest::testArticulatedObjectSkinned
#endif
            }, Cr::Containers::arraySize(SimulatorBuilder) );
  // clang-format on
}
void SimTest::basic() {
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  PathFinder::ptr pathfinder = simulator->getPathFinder();
  CORRADE_VERIFY(pathfinder);
}

void SimTest::reconfigure() {
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  PathFinder::ptr pathfinder = simulator->getPathFinder();
  SimulatorConfiguration cfg =
      simulator->getMetadataMediator()->getSimulatorConfiguration();
  simulator->reconfigure(cfg);
  CORRADE_COMPARE(pathfinder, simulator->getPathFinder());
  SimulatorConfiguration cfg2;
  cfg2.activeSceneName = skokloster;
  simulator->reconfigure(cfg2);
  CORRADE_VERIFY(pathfinder != simulator->getPathFinder());
}

void SimTest::reset() {
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);

  PathFinder::ptr pathfinder = simulator->getPathFinder();
  auto pinholeCameraSpec = CameraSensorSpec::create();
  pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
  pinholeCameraSpec->sensorType = SensorType::Color;
  pinholeCameraSpec->position = {0.0f, 1.5f, 5.0f};
  pinholeCameraSpec->resolution = {100, 100};
  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  auto agent = simulator->addAgent(agentConfig);

  auto stateOrig = AgentState::create();
  agent->getState(stateOrig);

  simulator->reset();

  auto stateFinal = AgentState::create();
  agent->getState(stateFinal);
  CORRADE_COMPARE(stateOrig->position, stateFinal->position);
  CORRADE_COMPARE(stateOrig->rotation, stateFinal->rotation);
  CORRADE_COMPARE(pathfinder, simulator->getPathFinder());
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
      Cr::Utility::Path::join(screenshotDir, groundTruthImageFile),
      (Mn::DebugTools::CompareImageToFile{maxThreshold, meanThreshold}));
}

void SimTest::getSceneRGBAObservation() {
  ESP_DEBUG() << "Starting Test : getSceneRGBAObservation";
  setTestCaseName(CORRADE_FUNCTION);
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  checkPinholeCameraRGBAObservation(*simulator, "SimTestExpectedScene.png",
                                    maxThreshold, 0.75f);
}

void SimTest::getSceneWithLightingRGBAObservation() {
  ESP_DEBUG() << "Starting Test : getSceneWithLightingRGBAObservation";
  setTestCaseName(CORRADE_FUNCTION);
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSceneWithLighting.png", maxThreshold, 0.75f);
}

void SimTest::getDefaultLightingRGBAObservation() {
  ESP_DEBUG() << "Starting Test : getDefaultLightingRGBAObservation";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({1.0f, 0.5f, -0.5f});
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);
}

void SimTest::getCustomLightingRGBAObservation() {
  ESP_DEBUG() << "Starting Test : getCustomLightingRGBAObservation";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj =
      rigidObjMgr->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({1.0f, 0.5f, -0.5f});

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);
}

void SimTest::updateLightSetupRGBAObservation() {
  ESP_DEBUG() << "Starting Test : updateLightSetupRGBAObservation";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();
  // update default lighting
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({1.0f, 0.5f, -0.5f});

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);

  simulator->setLightSetup(lightSetup1);
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);
  rigidObjMgr->removePhysObjectByHandle(obj->getHandle());

  // update custom lighting
  obj = rigidObjMgr->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({1.0f, 0.5f, -0.5f});

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.71f);
}

void SimTest::updateObjectLightSetupRGBAObservation() {
  ESP_DEBUG() << "Starting Test : updateObjectLightSetupRGBAObservation";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({1.0f, 0.5f, -0.5f});
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDefaultLighting.png", maxThreshold, 0.71f);

  // change from default lighting to custom
  obj->setLightSetup("custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting.png", maxThreshold, 0.71f);

  // change from one custom lighting to another
  obj->setLightSetup("custom_lighting_2");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedCustomLighting2.png", maxThreshold, 0.71f);
}

void SimTest::multipleLightingSetupsRGBAObservation() {
  ESP_DEBUG() << "Starting Test : multipleLightingSetupsRGBAObservation";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();
  // make sure updates apply to all objects using the light setup
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj =
      rigidObjMgr->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({0.0f, 0.5f, -0.5f});

  auto otherObj =
      rigidObjMgr->addObjectByHandle(objs[0], nullptr, "custom_lighting_1");
  CORRADE_VERIFY(otherObj->isAlive());
  CORRADE_VERIFY(otherObj->getID() != esp::ID_UNDEFINED);
  otherObj->setTranslation({2.0f, 0.5f, -0.5f});

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSameLighting.png", maxThreshold, 0.01f);

  simulator->setLightSetup(lightSetup2, "custom_lighting_1");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedSameLighting2.png", maxThreshold, 0.01f);
  simulator->setLightSetup(lightSetup1, "custom_lighting_1");

  // make sure we can move a single object to another group
  obj->setLightSetup("custom_lighting_2");
  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestExpectedDifferentLighting.png", maxThreshold, 0.01f);
}

void SimTest::recomputeNavmeshWithStaticObjects() {
  ESP_DEBUG() << "Starting Test : recomputeNavmeshWithStaticObjects";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, skokloster, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();

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
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  obj->setTranslation(Magnum::Vector3{randomNavPoint});
  obj->setMotionType(esp::physics::MotionType::STATIC);
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable({randomNavPoint}, 0.1));

  // recompute with object
  navMeshSettings.includeStaticObjects = true;
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings);
  CORRADE_VERIFY(!simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));

  // recompute without again
  navMeshSettings.includeStaticObjects = false;
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings);
  CORRADE_VERIFY(simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));

  rigidObjMgr->removePhysObjectByHandle(obj->getHandle());

  // test scaling
  ObjectAttributes::ptr objectTemplate = objectAttribsMgr->getObjectCopyByID(0);
  objectTemplate->setScale({0.5, 0.5, 0.5});
  int tmplateID = objectAttribsMgr->registerObject(objectTemplate);

  obj = rigidObjMgr->addObjectByHandle(objs[0]);
  obj->setTranslation(Magnum::Vector3{randomNavPoint});
  obj->setTranslation(obj->getTranslation() + Magnum::Vector3{0, 0.5, 0});
  obj->setMotionType(esp::physics::MotionType::STATIC);
  esp::vec3f offset(0.75, 0, 0);
  CORRADE_VERIFY(simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable(randomNavPoint + offset, 0.2));
  // recompute with object
  navMeshSettings.includeStaticObjects = true;
  simulator->recomputeNavMesh(*simulator->getPathFinder().get(),
                              navMeshSettings);
  CORRADE_VERIFY(!simulator->getPathFinder()->isNavigable(randomNavPoint, 0.1));
  CORRADE_VERIFY(
      simulator->getPathFinder()->isNavigable(randomNavPoint + offset, 0.2));
}

void SimTest::loadingObjectTemplates() {
  ESP_DEBUG() << "Starting Test : loadingObjectTemplates";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, true, esp::NO_LIGHT_KEY);
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();

  // test directory of templates
  std::vector<int> templateIndices =
      objectAttribsMgr->loadAllJSONConfigsFromPath(
          Cr::Utility::Path::join(TEST_ASSETS, "objects"));
  CORRADE_VERIFY(!templateIndices.empty());
  for (auto index : templateIndices) {
    CORRADE_VERIFY(index != esp::ID_UNDEFINED);
  }

  // reload again and ensure that old loaded indices are returned
  std::vector<int> templateIndices2 =
      objectAttribsMgr->loadAllJSONConfigsFromPath(
          Cr::Utility::Path::join(TEST_ASSETS, "objects"));
  CORRADE_COMPARE(templateIndices2, templateIndices);

  // test the loaded assets and accessing them by name
  // verify that getting the template handles with empty string returns all
  int numLoadedTemplates = templateIndices2.size();
  std::vector<std::string> templateHandles =
      objectAttribsMgr->getFileTemplateHandlesBySubstring();
  CORRADE_COMPARE(numLoadedTemplates, templateHandles.size());

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
  CORRADE_COMPARE(matchTmpltHandles[0], fullTmpHndl);

  // test fresh template as smart pointer
  ObjectAttributes::ptr newTemplate =
      objectAttribsMgr->createObject("new template", false);
  std::string boxPath =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/transform_box.glb");
  newTemplate->setRenderAssetHandle(boxPath);
  int templateIndex = objectAttribsMgr->registerObject(newTemplate, boxPath);

  CORRADE_VERIFY(templateIndex != esp::ID_UNDEFINED);
  // change render asset for object template named boxPath
  std::string chairPath =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/chair.glb");
  newTemplate->setRenderAssetHandle(chairPath);
  int templateIndex2 = objectAttribsMgr->registerObject(newTemplate, boxPath);

  CORRADE_VERIFY(templateIndex2 != esp::ID_UNDEFINED);
  CORRADE_COMPARE(templateIndex2, templateIndex);
  ObjectAttributes::ptr newTemplate2 =
      objectAttribsMgr->getObjectCopyByHandle(boxPath);
  CORRADE_COMPARE(newTemplate2->getRenderAssetHandle(), chairPath);
}

void SimTest::buildingPrimAssetObjectTemplates() {
  ESP_DEBUG() << "Starting Test : buildingPrimAssetObjectTemplates";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, true, esp::NO_LIGHT_KEY);

  // test that the correct number of default primitive assets are available as
  // render/collision targets
  // manager of asset attributes
  auto assetAttribsMgr = simulator->getAssetAttributesManager();
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto rigidObjMgr = simulator->getRigidObjectManager();

  // get all handles of templates for primitive-based render objects
  std::vector<std::string> primObjAssetHandles =
      objectAttribsMgr->getSynthTemplateHandlesBySubstring("");

  // there should be 1 prim template per default primitive asset template
  int numPrimsExpected =
      static_cast<int>(esp::metadata::PrimObjTypes::END_PRIM_OBJ_TYPES);
  // verify the number of primitive templates
  CORRADE_COMPARE(numPrimsExpected, primObjAssetHandles.size());

  AbstractPrimitiveAttributes::ptr primAttr;
  {
    // test that there are existing templates for each key, and that they have
    // valid values to be used to construct magnum primitives
    for (int i = 0; i < numPrimsExpected; ++i) {
      std::string handle = primObjAssetHandles[i];
      CORRADE_VERIFY(!handle.empty());
      primAttr = assetAttribsMgr->getObjectCopyByHandle(handle);
      CORRADE_VERIFY(primAttr);
      CORRADE_VERIFY(primAttr->isValidTemplate());
      // verify that the attributes contains the handle, and the handle contains
      // the expected class name
      std::string className =
          esp::metadata::managers::AssetAttributesManager::PrimitiveNames3DMap
              .at(static_cast<esp::metadata::PrimObjTypes>(
                  primAttr->getPrimObjType()));

      CORRADE_COMPARE(primAttr->getHandle(), handle);
      CORRADE_VERIFY(handle.find(className) != std::string::npos);
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
    CORRADE_COMPARE(primObjAssetHandles.size(), 1);
    // handle should not be empty and be long enough to hold class name prefix
    CORRADE_COMPARE_AS(primObjAssetHandles[0].length(), 9,
                       Cr::TestSuite::Compare::Greater);
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
    CORRADE_COMPARE((numPrimsExpected - 1), primObjAssetHandles.size());
    for (auto primObjAssetHandle : primObjAssetHandles) {
      CORRADE_COMPARE(primObjAssetHandle.find(checkStr), std::string::npos);
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
    CORRADE_COMPARE(primObjAssetHandles.size(), 1);
    // primitive render object uses primitive render asset as handle
    std::string origCylinderHandle = primObjAssetHandles[0];
    primAttr = assetAttribsMgr->getObjectCopyByHandle(origCylinderHandle);
    // verify that the origin handle matches what is expected
    CORRADE_COMPARE(primAttr->getHandle(), origCylinderHandle);
    // get original number of rings for this cylinder
    int origNumRings = primAttr->getNumRings();
    // modify attributes - this will change handle
    primAttr->setNumRings(2 * origNumRings);
    // verify that internal name of attributes has changed due to essential
    // quantity being modified
    std::string newHandle = primAttr->getHandle();

    CORRADE_COMPARE_AS(newHandle, origCylinderHandle,
                       Cr::TestSuite::Compare::NotEqual);
    // set bogus file directory, to validate that copy is registered
    primAttr->setFileDirectory("test0");
    // register new attributes
    int idx = assetAttribsMgr->registerObject(primAttr);

    CORRADE_VERIFY(idx != esp::ID_UNDEFINED);
    // set new test label, to validate against retrieved copy
    primAttr->setFileDirectory("test1");
    // retrieve registered attributes copy
    AbstractPrimitiveAttributes::ptr primAttr2 =
        assetAttribsMgr->getObjectCopyByHandle(newHandle);
    // verify pre-reg and post-reg are named the same
    CORRADE_COMPARE(primAttr->getHandle(), primAttr2->getHandle());
    // verify retrieved attributes is copy, not original

    CORRADE_COMPARE_AS(primAttr->getFileDirectory(),
                       primAttr2->getFileDirectory(),
                       Cr::TestSuite::Compare::NotEqual);
    // remove modified attributes
    AbstractPrimitiveAttributes::ptr primAttr3 =
        assetAttribsMgr->removeObjectByHandle(newHandle);
    CORRADE_VERIFY(primAttr3);
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
    CORRADE_VERIFY(newCylObjAttr);
    // create object with new attributes
    auto obj = rigidObjMgr->addObjectByHandle(newHandle);
    CORRADE_VERIFY(obj->isAlive());
    CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  }
  // empty vector of handles
  primObjAssetHandles.clear();

}  // SimTest::buildingPrimAssetObjectTemplates

void SimTest::addObjectByHandle() {
  ESP_DEBUG() << "Starting Test : addObject";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, true, esp::NO_LIGHT_KEY);
  auto rigidObjMgr = simulator->getRigidObjectManager();

  auto obj = rigidObjMgr->addObjectByHandle("invalid_handle");
  CORRADE_COMPARE(obj, nullptr);

  // pass valid object_config.json filepath as handle to addObjectByHandle
  const auto validHandle = Cr::Utility::Path::join(
      TEST_ASSETS, "objects/nested_box.object_config.json");
  obj = rigidObjMgr->addObjectByHandle(validHandle);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
}

void SimTest::addObjectsAndMakeObservation(
    Simulator& sim,
    esp::sensor::CameraSensorSpec& cameraSpec,
    const std::string& objTmpltHandle,
    Observation& observation) {
  auto rigidObjMgr = sim.getRigidObjectManager();
  // remove any existing objects
  rigidObjMgr->removeAllObjects();
  // add and place first object
  auto obj = rigidObjMgr->addObjectByHandle(objTmpltHandle, nullptr,
                                            "custom_lighting_1");
  obj->setTranslation({-1.0f, 0.5f, -2.5f});

  // add and place second object
  auto otherObj = rigidObjMgr->addObjectByHandle(objTmpltHandle, nullptr,
                                                 "custom_lighting_2");
  otherObj->setTranslation({1.0f, 0.5f, -2.5f});

  // Make Observation of constructed scene
  CORRADE_VERIFY(sim.getAgentObservation(0, cameraSpec.uuid, observation));

}  // SimTest::addObjectsAndMakeObservation

void SimTest::addObjectInvertedScale() {
  ESP_DEBUG() << "Starting Test : addObjectInvertedScale";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, planeStage, true, esp::NO_LIGHT_KEY);
  auto rigidObjMgr = simulator->getRigidObjectManager();
  auto objAttrMgr = simulator->getObjectAttributesManager();
  // Add agent to take image
  auto pinholeCameraSpec = CameraSensorSpec::create();
  pinholeCameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
  pinholeCameraSpec->sensorType = SensorType::Color;
  pinholeCameraSpec->position = {0.0f, 1.5f, 0.0f};
  pinholeCameraSpec->resolution = {128, 128};

  AgentConfiguration agentConfig{};
  agentConfig.sensorSpecifications = {pinholeCameraSpec};
  Agent::ptr agent = simulator->addAgent(agentConfig);
  agent->setInitialState(AgentState{});

  // Add 2 objects and take initial non-negative scaled observation
  const auto objHandle = Cr::Utility::Path::join(
      TEST_ASSETS, "objects/nested_box.object_config.json");

  Observation expectedObservation;
  addObjectsAndMakeObservation(*simulator, *pinholeCameraSpec, objHandle,
                               expectedObservation);

  // Make aa copy of observation buffer so future observations don't overwrite
  // this one
  Cr::Containers::Array<uint8_t> obsCopy{
      Cr::NoInit, expectedObservation.buffer->data.size()};
  Cr::Utility::copy(expectedObservation.buffer->data, obsCopy);

  // File name of expected image for un-inverted and each axis-inverted image
  const auto expectedScreenshotFile = Cr::Utility::Path::join(
      screenshotDir, "SimTestInvertScaleImageExpected.png");
  // Make a ground truth image based on a copy of the observation buffer
  const Mn::ImageView2D expectedImage{
      Mn::PixelFormat::RGBA8Unorm,
      {pinholeCameraSpec->resolution[0], pinholeCameraSpec->resolution[1]},
      obsCopy};

  // Verify non-negative scale scene is as expected
  CORRADE_COMPARE_WITH(
      expectedImage, expectedScreenshotFile,
      (Mn::DebugTools::CompareImageToFile{maxThreshold, 0.01f}));

  // Create and test observations with scale negative in each of x, y and z
  // directions
  Cr::Containers::StringView testAxis[3]{"X_axis", "Y_axis", "Z_axis"};
  for (int i = 0; i < 3; ++i) {
    CORRADE_ITERATION(testAxis[i]);
    ObjectAttributes::ptr newObjAttr =
        objAttrMgr->getObjectCopyByHandle(objHandle);

    Mn::Vector3 scale = newObjAttr->getScale();
    // change x, y, or z scale to be negative
    scale[i] *= -1.0f;

    // Set modified scale
    newObjAttr->setScale(scale);
    // Register new object attributes with negative scale along a single axis
    // using a new name
    const std::string newObjHandle =
        Cr::Utility::formatString("scale_{}_{}", i, objHandle);
    objAttrMgr->registerObject(newObjAttr, newObjHandle);

    // Build object layout and retrieve observation
    Observation newObservation;
    addObjectsAndMakeObservation(*simulator, *pinholeCameraSpec, newObjHandle,
                                 newObservation);

    const Mn::ImageView2D newImage{
        Mn::PixelFormat::RGBA8Unorm,
        {pinholeCameraSpec->resolution[0], pinholeCameraSpec->resolution[1]},
        newObservation.buffer->data};

    // Verify inverted scale scene is as expected compared to file.
    CORRADE_COMPARE_WITH(
        newImage, expectedScreenshotFile,
        (Mn::DebugTools::CompareImageToFile{maxThreshold, 0.01f}));

    // Needed to make a buffer copy into the comparison image
    CORRADE_COMPARE_WITH(newImage, expectedImage,
                         (Mn::DebugTools::CompareImage{maxThreshold, 0.01f}));
  }

}  // SimTest::addObjectInvertedScale

void SimTest::addSensorToObject() {
  ESP_DEBUG() << "Starting Test : addSensorToObject";
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, esp::NO_LIGHT_KEY);
  auto rigidObjMgr = simulator->getRigidObjectManager();
  // manager of object attributes
  auto objectAttribsMgr = simulator->getObjectAttributesManager();
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("icosphereSolid");
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  esp::scene::SceneNode& objectNode = *obj->getSceneNode();

  // Add sensor to sphere object
  auto objectSensorSpec = esp::sensor::CameraSensorSpec::create();
  objectSensorSpec->uuid = std::to_string(obj->getID());
  objectSensorSpec->position = {0, 0, 0};
  objectSensorSpec->orientation = {0, 0, 0};
  objectSensorSpec->resolution = {128, 128};
  simulator->addSensorToObject(obj->getID(), objectSensorSpec);
  std::string expectedUUID = std::to_string(obj->getID());
  CameraSensor& cameraSensor = dynamic_cast<CameraSensor&>(
      objectNode.getNodeSensorSuite().get(expectedUUID));
  cameraSensor.setTransformationFromSpec();

  obj->setTranslation(
      {1.0f, 1.5f, 1.0f});  // Move camera to same place as agent

  auto objs2 = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj2 = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj2->isAlive());
  CORRADE_VERIFY(obj2->getID() != esp::ID_UNDEFINED);
  obj2->setTranslation({1.0f, 0.5f, -0.5f});
  esp::scene::SceneNode& objectNode2 = *obj2->getSceneNode();

  Observation observation;
  ObservationSpace obsSpace;
  simulator->getRenderer()->bindRenderTarget(cameraSensor);
  CORRADE_VERIFY(cameraSensor.getObservation(*simulator, observation));
  CORRADE_VERIFY(cameraSensor.getObservationSpace(obsSpace));

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
      Cr::Utility::Path::join(screenshotDir, "SimTestExpectedScene.png"),
      (Mn::DebugTools::CompareImageToFile{maxThreshold, 0.75f}));
}

void SimTest::createMagnumRenderingOff() {
  ESP_DEBUG() << "Starting Test : createMagnumRenderingOff";

  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, false, "custom_lighting_1");
  auto objectAttribsMgr = simulator->getObjectAttributesManager();

  auto rigidObjMgr = simulator->getRigidObjectManager();

  // check that we can load a glb file
  auto objs = objectAttribsMgr->getObjectHandlesBySubstring("nested_box");
  auto obj = rigidObjMgr->addObjectByHandle(objs[0]);
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({-10.0f, -10.0f, -10.0f});

  // check that adding a primitive object works
  obj = rigidObjMgr->addObjectByHandle("cubeSolid");
  obj->setTranslation({10.0f, 10.0f, 10.0f});
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  esp::scene::SceneNode* objectNode = obj->getSceneNode();

  auto distanceBetween = [](Mn::Vector3 a, Mn::Vector3 b) {
    Mn::Vector3 d = b - a;
    return Mn::Math::pow(dot(d, d), 0.5f);
  };

  auto testRaycast = [&]() {
    // cast a ray at the object to check that the object is actually there
    auto raycastresults = simulator->castRay(
        esp::geo::Ray({10.0, 9.0, 10.0}, {0.0, 1.0, 0.0}), 100.0);
    CORRADE_COMPARE(raycastresults.hits[0].objectId, obj->getID());
    auto point = raycastresults.hits[0].point;
    CORRADE_COMPARE_AS(distanceBetween(point, {10.0, 9.9, 10.0}), 0.001,
                       Cr::TestSuite::Compare::Less);
    raycastresults = simulator->castRay(
        esp::geo::Ray({10.0, 11.0, 10.0}, {0.0, -1.0, 0.0}), 100.0);
    CORRADE_COMPARE(raycastresults.hits[0].objectId, obj->getID());
    point = raycastresults.hits[0].point;
    CORRADE_COMPARE_AS(distanceBetween(point, {10.0, 10.1, 10.0}), 0.001,
                       Cr::TestSuite::Compare::Less);
  };

  auto testBoundingBox = [&]() {
    // check that we can still compute bounding box of the object
    Magnum::Range3D meshbb = objectNode->getCumulativeBB();
    float eps = 0.001;
    CORRADE_COMPARE_WITH(meshbb.left(), -0.1,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH(meshbb.right(), 0.1,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH(meshbb.bottom(), -0.1,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH(meshbb.top(), 0.1,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH(meshbb.back(), -0.1,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH(meshbb.front(), 0.1,
                         Cr::TestSuite::Compare::around(eps));
  };
  // test raycast and bounding box for cubeSolid
  testRaycast();
  testBoundingBox();

  // test raycast and bounding box for cubeWireframe
  rigidObjMgr->removePhysObjectByHandle(obj->getHandle());
  obj = rigidObjMgr->addObjectByHandle("cubeWireframe");
  CORRADE_VERIFY(obj->isAlive());
  CORRADE_VERIFY(obj->getID() != esp::ID_UNDEFINED);
  obj->setTranslation({10.0f, 10.0f, 10.0f});
  objectNode = obj->getSceneNode();
  testRaycast();
  testBoundingBox();

  // do some sensor stuff to check that nothing errors
  auto objectSensorSpec = esp::sensor::CameraSensorSpec::create();
  objectSensorSpec->uuid = std::to_string(obj->getID());
  objectSensorSpec->position = {0, 0, 0};
  objectSensorSpec->orientation = {0, 0, 0};
  objectSensorSpec->resolution = {128, 128};
  simulator->addSensorToObject(obj->getID(), objectSensorSpec);
  std::string expectedUUID = std::to_string(obj->getID());
  CameraSensor& cameraSensor = dynamic_cast<CameraSensor&>(
      objectNode->getNodeSensorSuite().get(expectedUUID));
  cameraSensor.setTransformationFromSpec();
  Observation observation;

  // check that there is no renderer
  CORRADE_VERIFY(!simulator->getRenderer());
  CORRADE_VERIFY(!cameraSensor.getObservation(*simulator, observation));
}

void SimTest::getRuntimePerfStats() {
  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, vangogh, true, "custom_lighting_1");

  auto statNames = simulator->getRuntimePerfStatNames();

  constexpr auto numRigidIdx = 0;
  constexpr auto drawCountIdx = 5;
  constexpr auto drawFacesIdx = 6;
  CORRADE_COMPARE(statNames[numRigidIdx], "num rigid");
  CORRADE_COMPARE(statNames[drawCountIdx], "num drawables");
  CORRADE_COMPARE(statNames[drawFacesIdx], "num faces");

  auto statValues = simulator->getRuntimePerfStatValues();

  CORRADE_COMPARE(statValues[numRigidIdx], 0);
  // magic numbers here correspond to the contents of the vangogh 3D asset
  CORRADE_COMPARE(statValues[drawCountIdx], 15);
  CORRADE_COMPARE(statValues[drawFacesIdx], 11272);

  {
    auto objAttrMgr = simulator->getObjectAttributesManager();
    auto rigidObjMgr = simulator->getRigidObjectManager();
    auto objs = objAttrMgr->getObjectHandlesBySubstring("nested_box");
    rigidObjMgr->addObjectByHandle(objs[0]);
  }

  statNames = simulator->getRuntimePerfStatNames();
  statValues = simulator->getRuntimePerfStatValues();

  CORRADE_COMPARE(statValues[numRigidIdx], 1);
  // magic numbers here correspond to the contents of the vangogh and nested_box
  // 3D assets
  CORRADE_COMPARE(statValues[drawCountIdx], 17);
  CORRADE_COMPARE(statValues[drawFacesIdx], 11296);

  SimulatorConfiguration simConfig =
      simulator->getMetadataMediator()->getSimulatorConfiguration();
  simConfig.activeSceneName = esp::assets::EMPTY_SCENE;
  simulator->reconfigure(simConfig);

  statValues = simulator->getRuntimePerfStatValues();

  CORRADE_COMPARE(statValues[numRigidIdx], 0);
  CORRADE_COMPARE(statValues[drawCountIdx], 0);
  CORRADE_COMPARE(statValues[drawFacesIdx], 0);
}

void SimTest::testArticulatedObjectSkinned() {
  ESP_DEBUG() << "Starting Test : testArticulatedObjectSkinned";

  auto&& data = SimulatorBuilder[testCaseInstanceId()];
  setTestCaseDescription(data.name);
  auto simulator = data.creator(*this, "NONE", true, esp::NO_LIGHT_KEY);

  const std::string urdfFile =
      Cr::Utility::Path::join(TEST_ASSETS, "urdf/skinned_prism.urdf");

  auto aoManager = simulator->getArticulatedObjectManager();

  CORRADE_COMPARE(aoManager->getNumObjects(), 0);
  auto ao = aoManager->addArticulatedObjectFromURDF(urdfFile);
  CORRADE_COMPARE(aoManager->getNumObjects(), 1);

  CORRADE_COMPARE(ao->getSceneNode()->getSemanticId(), 100);

  CORRADE_VERIFY(ao);
  CORRADE_COMPARE(ao->getNumLinks(), 4);

  const auto linkIds = ao->getLinkIdsWithBase();

  auto linkA = ao->getLink(linkIds[0]);
  CORRADE_VERIFY(linkA->linkName == "A");
  auto linkB = ao->getLink(linkIds[1]);
  CORRADE_VERIFY(linkB->linkName == "B");
  auto linkC = ao->getLink(linkIds[2]);
  CORRADE_VERIFY(linkC->linkName == "C");
  auto linkD = ao->getLink(linkIds[3]);
  CORRADE_VERIFY(linkD->linkName == "D");
  auto linkE = ao->getLink(linkIds[4]);
  CORRADE_VERIFY(linkE->linkName == "E");

  ao->setTranslation({1.f, -3.f, -6.f});

  checkPinholeCameraRGBAObservation(
      *simulator, "SimTestSkinnedAOInitialPose.png", maxThreshold, 0.71f);

  const auto rot = Mn::Quaternion::rotation(
      Mn::Deg(25.f), Mn::Vector3(0.f, 1.f, 0.f).normalized());
  std::vector<float> jointPos{};
  for (int i = 0; i < 4; ++i) {
    const auto rotData = rot.data();
    jointPos.push_back(rotData[0]);  // x
    jointPos.push_back(rotData[1]);  // y
    jointPos.push_back(rotData[2]);  // z
    jointPos.push_back(rotData[3]);  // w
  }
  ao->setJointPositions(jointPos);

  checkPinholeCameraRGBAObservation(*simulator, "SimTestSkinnedAOPose.png",
                                    maxThreshold, 0.71f);

  aoManager->removeAllObjects();
  CORRADE_COMPARE(aoManager->getNumObjects(), 0);

}  // SimTest::testArticulatedObjectSkinned

}  // namespace

CORRADE_TEST_MAIN(SimTest)
