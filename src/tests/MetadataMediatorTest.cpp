// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

#include "configure.h"
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using esp::metadata::MetadataMediator;

namespace {

const std::string physicsConfigFile =
    Cr::Utility::Directory::join(DATA_DIR,
                                 "test_assets/testing.physics_config.json");

const std::string datasetTestDirs =
    Cr::Utility::Directory::join(DATA_DIR, "test_assets/dataset_tests/");

const std::string sceneDatasetConfigFile_0 = Cr::Utility::Directory::join(
    datasetTestDirs,
    "dataset_0/test_dataset_0.scene_dataset_config.json");

// test dataset contains glob wildcards in scene config, and articulated object
// refs
const std::string sceneDatasetConfigFile_1 = Cr::Utility::Directory::join(
    datasetTestDirs,
    "dataset_1/test_dataset_1.scene_dataset_config.json");

struct MetadataMediatorTest : Cr::TestSuite::Tester {
  explicit MetadataMediatorTest();

  void reset() {
    // create new MM with blank cfg
    MM_ = MetadataMediator::create();
    // reference alternate dataset
  };

  void initDataset0() {
    auto cfg_0 = esp::sim::SimulatorConfiguration{};
    ESP_WARNING() << "Starting testDataset0 : initDataset0 0";
    cfg_0.sceneDatasetConfigFile = sceneDatasetConfigFile_0;
    cfg_0.physicsConfigFile = physicsConfigFile;
    MM_->setSimulatorConfiguration(cfg_0);
    ESP_WARNING() << "Starting testDataset0 : initDataset0 1";
  }

  void initDataset1() {
    auto cfg_1 = esp::sim::SimulatorConfiguration{};
    ESP_WARNING() << "Starting testDataset0 : initDataset1 0";
    cfg_1.sceneDatasetConfigFile = sceneDatasetConfigFile_1;
    cfg_1.physicsConfigFile = physicsConfigFile;
    MM_->setSimulatorConfiguration(cfg_1);
    ESP_WARNING() << "Starting testDataset0 : initDataset1 1";
  }

  void displayDSReports() {
    // display info report
    std::string dsOverView = MM_->getDatasetsOverview();
    ESP_WARNING() << "\nDataset Overview : \n"
                  << dsOverView << "\nDataset Overview Done";
    // display info report
    std::string dsInfoReport = MM_->createDatasetReport();
    ESP_WARNING() << "\nActive Dataset Details : \n"
                  << dsInfoReport << "\nActive Dataset Done";
  }

  // tests
  void testDataset0();
  void testDataset1();

  void testDatasetDelete();

  esp::logging::LoggingContext loggingContext;
  MetadataMediator::ptr MM_ = nullptr;

};  // struct MetadataMediatorTest

MetadataMediatorTest::MetadataMediatorTest() {
  MM_ = MetadataMediator::create();
  addTests({&MetadataMediatorTest::testDataset0,
            &MetadataMediatorTest::testDataset1,
            &MetadataMediatorTest::testDatasetDelete});

}  // ctor

void MetadataMediatorTest::testDataset0() {
  // setup dataset 0 for test
  initDataset0();

  ESP_WARNING() << "Starting testDataset0 : test LoadStages";

  const auto& stageAttributesMgr = MM_->getStageAttributesManager();
  int numStageHandles = stageAttributesMgr->getNumObjects();
  // should be 7 - one for default NONE stage, one original JSON, one based on
  // original but changed, and one new, and 3 built from .glb files
  CORRADE_COMPARE(numStageHandles, 7);
  // get list of handles matching glb-based attributes
  auto glbBasedStageAttrHandles =
      stageAttributesMgr->getObjectHandlesBySubstring(".glb", true);
  CORRADE_COMPARE(glbBasedStageAttrHandles.size(), 3);
  for (const auto& attrHandle : glbBasedStageAttrHandles) {
    auto glbStageAttr = stageAttributesMgr->getObjectCopyByHandle(attrHandle);
    // verify that all attributes' names are the same as the render handles
    // (which were the original files)
    CORRADE_COMPARE(glbStageAttr->getHandle(),
                    glbStageAttr->getRenderAssetHandle());
  }

  // get list of matching handles for base - should always only be 1
  auto stageAttrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_stage.stage_config.json", true);
  CORRADE_COMPARE(stageAttrHandles.size(), 1);
  // get copy of attributes
  auto stageAttr =
      stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // get render asset handle for later comparison
  auto renderAssetHandle = stageAttr->getRenderAssetHandle();
  // verify existence
  CORRADE_VERIFY(stageAttr);
  // verify set to values in file
  CORRADE_COMPARE(stageAttr->getScale(), Magnum::Vector3(2, 2, 2));
  CORRADE_VERIFY(!stageAttr->getForceFlatShading());
  CORRADE_COMPARE(stageAttr->getMargin(), 0.03);
  CORRADE_COMPARE(stageAttr->getFrictionCoefficient(), 0.3);
  CORRADE_COMPARE(stageAttr->getRestitutionCoefficient(), 0.3);
  CORRADE_COMPARE(stageAttr->getOrigin(), Magnum::Vector3(0, 0, 0));
  CORRADE_COMPARE(stageAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  CORRADE_COMPARE(stageAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));

  // get list of matching handles for modified base - should always only be 1
  stageAttrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_stage", true);
  CORRADE_COMPARE(stageAttrHandles.size(), 1);
  // get copy of attributes
  stageAttr = stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(stageAttr);
  // verify set to override values in dataset_config file
  CORRADE_COMPARE(stageAttr->getScale(), Magnum::Vector3(1, 1, 1));
  CORRADE_COMPARE(stageAttr->getMargin(), 0.041);
  CORRADE_COMPARE(stageAttr->getFrictionCoefficient(), 0.4);
  CORRADE_COMPARE(stageAttr->getRestitutionCoefficient(), 0.5);
  CORRADE_COMPARE(stageAttr->getUnitsToMeters(), 1.0);

  // get list of matching handles for new - should always only be 1
  stageAttrHandles =
      stageAttributesMgr->getObjectHandlesBySubstring("new_test_stage", true);
  CORRADE_COMPARE(stageAttrHandles.size(), 1);
  // get copy of attributes
  stageAttr = stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(stageAttr);
  // verify set to values in dataset_config file
  auto newRenderAssetHandle = stageAttr->getRenderAssetHandle();
  // verify same renderasset handle to loaded stage attributes
  CORRADE_COMPARE(renderAssetHandle, newRenderAssetHandle);
  // verify values set correctly
  CORRADE_COMPARE(stageAttr->getOrientUp(), Magnum::Vector3(0, -1, 0));
  CORRADE_COMPARE(stageAttr->getScale(), Magnum::Vector3(2, 2, 2));
  CORRADE_COMPARE(stageAttr->getGravity(), Magnum::Vector3(0, 9.8, 0));
  CORRADE_COMPARE(stageAttr->getFrictionCoefficient(), 0.35);
  CORRADE_COMPARE(stageAttr->getRestitutionCoefficient(), 0.25);
  CORRADE_COMPARE(stageAttr->getUnitsToMeters(), 2.0);
  CORRADE_VERIFY(stageAttr->getForceFlatShading());

  // get new attributes but don't register
  stageAttr = stageAttributesMgr->createObject("new_default_attributes", false);
  // verify existence
  CORRADE_VERIFY(stageAttr);
  // verify contains default attributes value
  CORRADE_COMPARE(stageAttr->getOrigin(), Magnum::Vector3(1.0, 2.0, 3.0));

  // end test LoadStages

  ESP_WARNING() << "Starting test LoadObjects";

  const auto& objectAttributesMgr = MM_->getObjectAttributesManager();
  int numObjHandles = objectAttributesMgr->getNumFileTemplateObjects();
  // should be 6 file-based templates - 4 original, one based on an original
  // but changed, and one new
  CORRADE_COMPARE(numObjHandles, 6);

  // get list of matching handles for base object - should always only be 1
  auto objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_object1.object_config.json", true);
  CORRADE_COMPARE(objAttrHandles.size(), 1);
  // get copy of attributes
  auto objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(objAttr);
  // verify set to values in file
  CORRADE_COMPARE(objAttr->getScale(), Magnum::Vector3(1, 1, 1));
  CORRADE_VERIFY(!objAttr->getForceFlatShading());
  CORRADE_COMPARE(objAttr->getMargin(), 0.03);
  CORRADE_COMPARE(objAttr->getMass(), 0.038);
  CORRADE_COMPARE(objAttr->getFrictionCoefficient(), 0.5);
  CORRADE_COMPARE(objAttr->getRestitutionCoefficient(), 0.2);
  CORRADE_COMPARE(objAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  CORRADE_COMPARE(objAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));
  CORRADE_COMPARE(objAttr->getUnitsToMeters(), 1.0);
  CORRADE_VERIFY(!objAttr->getBoundingBoxCollisions());
  CORRADE_VERIFY(objAttr->getJoinCollisionMeshes());

  // get list of matching handles for modified base - should always only be 1
  objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_object1_1_slick_heavy", true);
  CORRADE_COMPARE(objAttrHandles.size(), 1);
  // get copy of attributes
  objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(objAttr);
  // verify identical to copied object except for override values in
  // dataset_config file
  CORRADE_COMPARE(objAttr->getScale(), Magnum::Vector3(1, 1, 1));
  CORRADE_VERIFY(!objAttr->getForceFlatShading());
  CORRADE_COMPARE(objAttr->getMargin(), 0.03);
  CORRADE_COMPARE(objAttr->getMass(), 3.5);
  CORRADE_COMPARE(objAttr->getFrictionCoefficient(), 0.2);
  CORRADE_COMPARE(objAttr->getRestitutionCoefficient(), 0.2);
  CORRADE_COMPARE(objAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  CORRADE_COMPARE(objAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));
  CORRADE_COMPARE(objAttr->getUnitsToMeters(), 1.0);
  CORRADE_VERIFY(!objAttr->getBoundingBoxCollisions());
  CORRADE_VERIFY(objAttr->getJoinCollisionMeshes());

  // get list of matching handles for new - should always only be 1
  objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "new_test_object3", true);
  CORRADE_COMPARE(objAttrHandles.size(), 1);
  // get copy of attributes
  objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(objAttr);
  // verify set to values in dataset_config file
  newRenderAssetHandle = objAttr->getRenderAssetHandle();
  // verify same renderasset handle to loaded stage attributes
  CORRADE_VERIFY(newRenderAssetHandle.find("dataset_test_object3.glb") !=
                 std::string::npos);
  // verify values set correctly

  CORRADE_COMPARE(objAttr->getMass(), 1.1);
  CORRADE_COMPARE(objAttr->getFrictionCoefficient(), 0.1);
  CORRADE_COMPARE(objAttr->getInertia(), Magnum::Vector3(3, 2, 1));

  // get new attributes but don't register
  objAttr = objectAttributesMgr->createObject("new_default_attributes", false);
  // verify existence
  CORRADE_VERIFY(objAttr);
  // verify contains default attributes value
  CORRADE_COMPARE(objAttr->getMass(), 10.0);
  CORRADE_COMPARE(objAttr->getInertia(), Magnum::Vector3(3, 2, 1));

  // end test LoadObjects

  ESP_WARNING() << "Starting test LoadLights";

  const auto& lightsLayoutAttributesMgr =
      MM_->getLightLayoutAttributesManager();
  // get # of loaded light layout attributes.
  int numLightAttrHandles = lightsLayoutAttributesMgr->getNumObjects();
  // Should be 3 : file based, copy and new
  CORRADE_COMPARE(numLightAttrHandles, 3);
  // get list of matching handles for base light config - should always only
  // be 1
  auto lightAttrHandles =
      lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
          "dataset_test_lights.lighting_config.json", true);
  CORRADE_COMPARE(lightAttrHandles.size(), 1);
  // get copy of attributes
  auto lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(lightAttr);
  // verify the number of lights within the lighting layout
  CORRADE_COMPARE(12, lightAttr->getNumLightInstances());

  // get list of matching handles for modified light config - should always
  // only be 1
  lightAttrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_lights", true);
  CORRADE_COMPARE(lightAttrHandles.size(), 1);
  // get copy of attributes
  lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(lightAttr);
  // verify the number of lights within the lighting layout
  CORRADE_COMPARE(lightAttr->getNumLightInstances(), 12);

  // get list of matching handles for new light config - should always only be
  // 1
  lightAttrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
      "new_test_lights_0", true);
  CORRADE_COMPARE(lightAttrHandles.size(), 1);
  // get copy of attributes
  lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  CORRADE_VERIFY(lightAttr);
  // verify the number of lights within the lighting layout
  CORRADE_COMPARE(lightAttr->getNumLightInstances(), 3);

  // end test LoadLights

  ESP_WARNING() << "Starting test LoadSceneInstances";
  //
  // SHOULD NOT BE REFERENCED DIRECTLY IN USER CODE, but rather desired scene
  // instance should be acquired through MM.
  //
  const auto& sceneAttributesMgr = MM_->getSceneAttributesManager();
  // get # of loaded scene attributes.
  int numSceneHandles = sceneAttributesMgr->getNumObjects();
  // should be 1
  CORRADE_COMPARE(numSceneHandles, 1);
  // get handle list matching passed handle
  auto sceneAttrHandles = sceneAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_scene", true);
  // make sure there is only 1 matching dataset_test_scene
  CORRADE_COMPARE(sceneAttrHandles.size(), 1);

  const std::string activeSceneName = sceneAttrHandles[0];
  ESP_WARNING() << "testLoadSceneInstances : Scene instance attr handle :"
                << activeSceneName;
  // get scene instance attributes ref
  // metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
  auto sceneAttrs = MM_->getSceneAttributesByName(activeSceneName);
  // this should be a scene instance attributes with specific stage and object
  CORRADE_VERIFY(sceneAttrs);
  // verify default value for translation origin
  CORRADE_COMPARE(static_cast<int>(sceneAttrs->getTranslationOrigin()),
                  static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
  const int assetLocalInt =
      static_cast<int>(Attrs::SceneInstanceTranslationOrigin::AssetLocal);

  //
  // miscellaneous scene instance attribute values
  //
  // default lighting name
  const std::string lightHandle = sceneAttrs->getLightingHandle();
  CORRADE_VERIFY(lightHandle.find("modified_test_lights") != std::string::npos);
  // navmesh
  const std::string navmeshHandle = sceneAttrs->getNavmeshHandle();
  CORRADE_VERIFY(navmeshHandle.find("navmesh_path1") != std::string::npos);
  // ssd
  const std::string ssdHandle = sceneAttrs->getSemanticSceneHandle();
  CORRADE_VERIFY(ssdHandle.find("semantic_descriptor_path1") !=
                 std::string::npos);

  //
  // test stage instance
  //
  const auto stageInstanceAttrs = sceneAttrs->getStageInstance();
  // verify name
  const std::string stageName = stageInstanceAttrs->getHandle();
  CORRADE_VERIFY(stageName.find("modified_test_stage") != std::string::npos);
  // verify translation origin to be asset_local
  CORRADE_COMPARE(static_cast<int>(stageInstanceAttrs->getTranslationOrigin()),
                  assetLocalInt);
  // verify translation amount to be expected amount
  CORRADE_COMPARE(stageInstanceAttrs->getTranslation(),
                  Magnum::Vector3(1.1, 2.2, 3.3));
  //
  // test object instances
  //
  // get all instances
  const std::vector<Attrs::SceneObjectInstanceAttributes::cptr>
      objInstanceAttrs = sceneAttrs->getObjectInstances();
  CORRADE_COMPARE(objInstanceAttrs.size(), 2);

  // first object instance
  const Attrs::SceneObjectInstanceAttributes::cptr& objAttr0 =
      objInstanceAttrs[0];
  // name
  std::string objName = objAttr0->getHandle();
  CORRADE_COMPARE_AS(objName.find("dataset_test_object1"), std::string::npos,
                     Cr::TestSuite::Compare::NotEqual);
  // translation
  CORRADE_COMPARE(objAttr0->getTranslation(), Magnum::Vector3(0.1, 0.2, 0.3));
  // translation origin
  CORRADE_COMPARE(static_cast<int>(objAttr0->getTranslationOrigin()),
                  assetLocalInt);

  // second object instance
  const Attrs::SceneObjectInstanceAttributes::cptr& objAttr1 =
      objInstanceAttrs[1];
  // name
  objName = objAttr1->getHandle();
  CORRADE_COMPARE_AS(objName.find("dataset_test_object2"), std::string::npos,
                     Cr::TestSuite::Compare::NotEqual);
  // translation
  CORRADE_COMPARE(objAttr1->getTranslation(), Magnum::Vector3(0.3, 0.4, 0.5));
  // translation origin
  CORRADE_COMPARE(static_cast<int>(objAttr1->getTranslationOrigin()),
                  assetLocalInt);

  // end test LoadSceneInstances

  ESP_WARNING() << "Starting test LoadNavmesh";
  // get map of navmeshes
  const std::map<std::string, std::string> navmeshMap =
      MM_->getActiveNavmeshMap();
  // should have 2
  CORRADE_COMPARE(navmeshMap.size(), 2);

  // should hold 2 keys
  CORRADE_VERIFY(navmeshMap.count("navmesh_path1") > 0);
  CORRADE_VERIFY(navmeshMap.count("navmesh_path2") > 0);
  // each key should hold specific value
  CORRADE_COMPARE(navmeshMap.at("navmesh_path1"), "test_navmesh_path1");
  CORRADE_COMPARE(navmeshMap.at("navmesh_path2"), "test_navmesh_path2");
  // end test LoadNavmesh

  ESP_WARNING() << "Starting test LoadSemanticScene";
  // get map of semantic scene instances
  const std::map<std::string, std::string> semanticMap =
      MM_->getActiveSemanticSceneDescriptorMap();
  // should have 2
  CORRADE_COMPARE(semanticMap.size(), 2);
  // should hold 2 keys
  CORRADE_VERIFY(semanticMap.count("semantic_descriptor_path1") > 0);
  CORRADE_VERIFY(semanticMap.count("semantic_descriptor_path2") > 0);
  // each key should hold specific value
  CORRADE_COMPARE(semanticMap.at("semantic_descriptor_path1"),
                  "test_semantic_descriptor_path1");
  CORRADE_COMPARE(semanticMap.at("semantic_descriptor_path2"),
                  "test_semantic_descriptor_path2");

  // end test LoadSemanticScene
  displayDSReports();

}  // testDataset0

void MetadataMediatorTest::testDataset1() {
  // primarily testing glob file wildcard loading
  initDataset1();

  ESP_WARNING() << "Starting testDataset1 : test LoadStages";
  const auto& stageAttributesMgr = MM_->getStageAttributesManager();
  int numStageHandles = stageAttributesMgr->getNumObjects();
  // shoudld be 6 : one for default NONE stage, glob lookup yields 2 stages +
  // 2 modified and 1 new stage in scene dataset config
  CORRADE_COMPARE(numStageHandles, 6);
  // end test LoadStages

  ESP_WARNING() << "Starting test LoadObjects";
  const auto& objectAttributesMgr = MM_->getObjectAttributesManager();
  int numObjHandles = objectAttributesMgr->getNumFileTemplateObjects();
  // glob lookup yields 4 files + 2 modified in config
  CORRADE_COMPARE(numObjHandles, 6);
  // end test LoadObjects

  ESP_WARNING() << "Starting test LoadLights";
  const auto& lightsLayoutAttributesMgr =
      MM_->getLightLayoutAttributesManager();
  // get # of loaded light layout attributes.
  int numLightAttrHandles = lightsLayoutAttributesMgr->getNumObjects();
  // Should be 4 : 2 file based, copy and new
  CORRADE_COMPARE(numLightAttrHandles, 4);

  // end test LoadLights

  ESP_WARNING() << "Starting test LoadSceneInstances";
  //
  // SHOULD NOT BE REFERENCED DIRECTLY IN USER CODE, but rather desired scene
  // instance should be acquired through MM.
  //
  const auto& sceneAttributesMgr = MM_->getSceneAttributesManager();
  // get # of loaded scene attributes.
  int numSceneHandles = sceneAttributesMgr->getNumObjects();
  // should be 2 - 2 file based
  CORRADE_COMPARE(numSceneHandles, 2);

  // end test LoadSceneInstances

  ESP_WARNING() << "Starting test LoadArticulatedObjects";

  namespace Dir = Cr::Utility::Directory;
  // verify # of urdf filepaths loaded - should be 6;
  const std::map<std::string, std::string>& urdfTestFilenames =
      MM_->getArticulatedObjectModelFilenames();
  CORRADE_COMPARE(urdfTestFilenames.size(), 6);
  // test that each stub name key corresponds to the actual file name passed
  // through the key making process
  for (std::map<std::string, std::string>::const_iterator iter =
           urdfTestFilenames.begin();
       iter != urdfTestFilenames.end(); ++iter) {
    // TODO replace when model intherits from AbstractManagedObject and
    // instances proper key synth methods.
    const std::string shortHandle =
        Dir::splitExtension(
            Dir::splitExtension(Dir::filename(iter->second)).first)
            .first;
    // test that map key constructed as shortened handle.
    CORRADE_COMPARE(shortHandle, iter->first);
    // test that file name ends in ".urdf"
    CORRADE_COMPARE(Dir::splitExtension(Dir::filename(iter->second))
                        .second.compare(".urdf"),
                    0);
    // test that file actually exists
    const std::string filename = iter->second;
    CORRADE_VERIFY(Dir::exists(filename));
  }
  // end test LoadArticulatedObjects

  ESP_WARNING() << "Starting test LoadNavmesh";
  // get map of navmeshes
  const std::map<std::string, std::string> navmeshMap =
      MM_->getActiveNavmeshMap();
  // should have 3
  CORRADE_COMPARE(navmeshMap.size(), 3);
  // end test LoadNavmesh

  ESP_WARNING() << "Starting test LoadSemanticScene";
  // get map of semantic scene instances
  const std::map<std::string, std::string> semanticMap =
      MM_->getActiveSemanticSceneDescriptorMap();
  // should have 3
  CORRADE_COMPARE(semanticMap.size(), 3);
  // testLoadSemanticScene
  // display info report
  displayDSReports();

}  // testDataset1

void MetadataMediatorTest::testDatasetDelete() {
  // load dataset 1 and make active
  initDataset1();
  const std::string nameDS1 = MM_->getActiveSceneDatasetName();
  // verify the dataset has been added
  CORRADE_VERIFY(MM_->sceneDatasetExists(nameDS1));
  // verify the active dataset is the expected name
  CORRADE_COMPARE(nameDS1, sceneDatasetConfigFile_1);
  // get the stage attributes manager for the scene dataset
  const auto& stageAttrMgr_DS1 = MM_->getStageAttributesManager();
  // verify not nullptr
  CORRADE_VERIFY(stageAttrMgr_DS1);

  // load datsaet 0 and make active
  initDataset0();
  // get new active dataset
  const std::string nameDS0 = MM_->getActiveSceneDatasetName();
  // verify the dataset has been added
  CORRADE_VERIFY(MM_->sceneDatasetExists(nameDS0));
  // verify the active dataset is the expected name
  CORRADE_COMPARE(nameDS0, sceneDatasetConfigFile_0);

  // delete dataset 1 and verify delete
  CORRADE_VERIFY(MM_->removeSceneDataset(nameDS1));
  // verify the dataset does not exist anymore
  CORRADE_VERIFY(!MM_->sceneDatasetExists(nameDS1));

  // verify deleted scene dataset's stage manager is nullptr
  CORRADE_VERIFY(!stageAttrMgr_DS1);

  // attempt to delete dataset 0 and verify fails - cannot delete active dataset
  CORRADE_VERIFY(!MM_->removeSceneDataset(nameDS0));

  // switch to default active dataset
  MM_->setActiveSceneDatasetName("default");
  // attempt to delete dataset 0 and verify delete
  CORRADE_VERIFY(MM_->removeSceneDataset(nameDS0));
  // verify the dataset does not exist anymore
  CORRADE_VERIFY(!MM_->sceneDatasetExists(nameDS0));

  // display info report
  displayDSReports();

}  // testDatasetDelete

}  // namespace

CORRADE_TEST_MAIN(MetadataMediatorTest)
