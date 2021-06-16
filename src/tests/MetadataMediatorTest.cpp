// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

#include "configure.h"

namespace {
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using esp::metadata::MetadataMediator;

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

class MetadataMediatorTest : public testing::Test {
 protected:
  void SetUp() override {
    // create new MM with blank cfg
    MM_ = MetadataMediator::create();

    // reference alternate dataset
  };

  void initDataset0() {
    auto cfg_0 = esp::sim::SimulatorConfiguration{};
    cfg_0.sceneDatasetConfigFile = sceneDatasetConfigFile_0;
    cfg_0.physicsConfigFile = physicsConfigFile;
    MM_->setSimulatorConfiguration(cfg_0);
  }

  void initDataset1() {
    auto cfg_1 = esp::sim::SimulatorConfiguration{};
    cfg_1.sceneDatasetConfigFile = sceneDatasetConfigFile_1;
    cfg_1.physicsConfigFile = physicsConfigFile;
    MM_->setSimulatorConfiguration(cfg_1);
  }

  MetadataMediator::ptr MM_ = nullptr;

};  // class MetadataMediatorTest

TEST_F(MetadataMediatorTest, testDataset0) {
  // setup dataset 0 for test
  initDataset0();

  LOG(INFO) << "Starting testDataset0 : test LoadStages";
  const auto& stageAttributesMgr = MM_->getStageAttributesManager();
  int numStageHandles = stageAttributesMgr->getNumObjects();
  // should be 7 - one for default NONE stage, one original JSON, one based on
  // original but changed, and one new, and 3 built from .glb files
  ASSERT_EQ(numStageHandles, 7);
  // get list of handles matching glb-based attributes
  auto glbBasedStageAttrHandles =
      stageAttributesMgr->getObjectHandlesBySubstring(".glb", true);
  ASSERT_EQ(glbBasedStageAttrHandles.size(), 3);
  for (const auto& attrHandle : glbBasedStageAttrHandles) {
    auto glbStageAttr = stageAttributesMgr->getObjectCopyByHandle(attrHandle);
    // verify that all attributes' names are the same as the render handles
    // (which were the original files)
    ASSERT_EQ(glbStageAttr->getHandle(), glbStageAttr->getRenderAssetHandle());
  }

  // get list of matching handles for base - should always only be 1
  auto stageAttrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_stage.stage_config.json", true);
  ASSERT_EQ(stageAttrHandles.size(), 1);
  // get copy of attributes
  auto stageAttr =
      stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // get render asset handle for later comparison
  auto renderAssetHandle = stageAttr->getRenderAssetHandle();
  // verify existence
  ASSERT_NE(nullptr, stageAttr);
  // verify set to values in file
  ASSERT_EQ(stageAttr->getScale(), Magnum::Vector3(2, 2, 2));
  ASSERT_EQ(stageAttr->getRequiresLighting(), true);
  ASSERT_EQ(stageAttr->getMargin(), 0.03);
  ASSERT_EQ(stageAttr->getFrictionCoefficient(), 0.3);
  ASSERT_EQ(stageAttr->getRestitutionCoefficient(), 0.3);
  ASSERT_EQ(stageAttr->getOrigin(), Magnum::Vector3(0, 0, 0));
  ASSERT_EQ(stageAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  ASSERT_EQ(stageAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));

  // get list of matching handles for modified base - should always only be 1
  stageAttrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_stage", true);
  ASSERT_EQ(stageAttrHandles.size(), 1);
  // get copy of attributes
  stageAttr = stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, stageAttr);
  // verify set to override values in dataset_config file
  ASSERT_EQ(stageAttr->getScale(), Magnum::Vector3(1, 1, 1));
  ASSERT_EQ(stageAttr->getMargin(), 0.041);
  ASSERT_EQ(stageAttr->getFrictionCoefficient(), 0.4);
  ASSERT_EQ(stageAttr->getRestitutionCoefficient(), 0.5);
  ASSERT_EQ(stageAttr->getUnitsToMeters(), 1.0);

  // get list of matching handles for new - should always only be 1
  stageAttrHandles =
      stageAttributesMgr->getObjectHandlesBySubstring("new_test_stage", true);
  ASSERT_EQ(stageAttrHandles.size(), 1);
  // get copy of attributes
  stageAttr = stageAttributesMgr->getObjectCopyByHandle(stageAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, stageAttr);
  // verify set to values in dataset_config file
  auto newRenderAssetHandle = stageAttr->getRenderAssetHandle();
  // verify same renderasset handle to loaded stage attributes
  ASSERT_EQ(renderAssetHandle, newRenderAssetHandle);
  // verify values set correctly
  ASSERT_EQ(stageAttr->getOrientUp(), Magnum::Vector3(0, -1, 0));
  ASSERT_EQ(stageAttr->getScale(), Magnum::Vector3(2, 2, 2));
  ASSERT_EQ(stageAttr->getGravity(), Magnum::Vector3(0, 9.8, 0));
  ASSERT_EQ(stageAttr->getFrictionCoefficient(), 0.35);
  ASSERT_EQ(stageAttr->getRestitutionCoefficient(), 0.25);
  ASSERT_EQ(stageAttr->getUnitsToMeters(), 2.0);
  ASSERT_EQ(stageAttr->getRequiresLighting(), false);

  // get new attributes but don't register
  stageAttr = stageAttributesMgr->createObject("new_default_attributes", false);
  // verify existence
  ASSERT_NE(nullptr, stageAttr);
  // verify contains default attributes value
  ASSERT_EQ(stageAttr->getOrigin(), Magnum::Vector3(1.0, 2.0, 3.0));

  // end test LoadStages

  LOG(INFO) << "Starting test LoadObjects";

  const auto& objectAttributesMgr = MM_->getObjectAttributesManager();
  int numObjHandles = objectAttributesMgr->getNumFileTemplateObjects();
  // should be 6 file-based templates - 4 original, one based on an original
  // but changed, and one new
  ASSERT_EQ(numObjHandles, 6);

  // get list of matching handles for base object - should always only be 1
  auto objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_object1.object_config.json", true);
  ASSERT_EQ(objAttrHandles.size(), 1);
  // get copy of attributes
  auto objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, objAttr);
  // verify set to values in file
  ASSERT_EQ(objAttr->getScale(), Magnum::Vector3(1, 1, 1));
  ASSERT_EQ(objAttr->getRequiresLighting(), true);
  ASSERT_EQ(objAttr->getMargin(), 0.03);
  ASSERT_EQ(objAttr->getMass(), 0.038);
  ASSERT_EQ(objAttr->getFrictionCoefficient(), 0.5);
  ASSERT_EQ(objAttr->getRestitutionCoefficient(), 0.2);
  ASSERT_EQ(objAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  ASSERT_EQ(objAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));
  ASSERT_EQ(objAttr->getUnitsToMeters(), 1.0);
  ASSERT_EQ(objAttr->getBoundingBoxCollisions(), false);
  ASSERT_EQ(objAttr->getJoinCollisionMeshes(), true);

  // get list of matching handles for modified base - should always only be 1
  objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_object1_1_slick_heavy", true);
  ASSERT_EQ(objAttrHandles.size(), 1);
  // get copy of attributes
  objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, objAttr);
  // verify identical to copied object except for override values in
  // dataset_config file
  ASSERT_EQ(objAttr->getScale(), Magnum::Vector3(1, 1, 1));
  ASSERT_EQ(objAttr->getRequiresLighting(), true);
  ASSERT_EQ(objAttr->getMargin(), 0.03);
  ASSERT_EQ(objAttr->getMass(), 3.5);
  ASSERT_EQ(objAttr->getFrictionCoefficient(), 0.2);
  ASSERT_EQ(objAttr->getRestitutionCoefficient(), 0.2);
  ASSERT_EQ(objAttr->getOrientUp(), Magnum::Vector3(0, 1, 0));
  ASSERT_EQ(objAttr->getOrientFront(), Magnum::Vector3(0, 0, -1));
  ASSERT_EQ(objAttr->getUnitsToMeters(), 1.0);
  ASSERT_EQ(objAttr->getBoundingBoxCollisions(), false);
  ASSERT_EQ(objAttr->getJoinCollisionMeshes(), true);

  // get list of matching handles for new - should always only be 1
  objAttrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
      "new_test_object3", true);
  ASSERT_EQ(objAttrHandles.size(), 1);
  // get copy of attributes
  objAttr = objectAttributesMgr->getObjectCopyByHandle(objAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, objAttr);
  // verify set to values in dataset_config file
  newRenderAssetHandle = objAttr->getRenderAssetHandle();
  // verify same renderasset handle to loaded stage attributes
  ASSERT_NE(std::string::npos,
            newRenderAssetHandle.find("dataset_test_object3.glb"));
  // verify values set correctly

  ASSERT_EQ(objAttr->getMass(), 1.1);
  ASSERT_EQ(objAttr->getFrictionCoefficient(), 0.1);
  ASSERT_EQ(objAttr->getInertia(), Magnum::Vector3(3, 2, 1));

  // get new attributes but don't register
  objAttr = objectAttributesMgr->createObject("new_default_attributes", false);
  // verify existence
  ASSERT_NE(nullptr, objAttr);
  // verify contains default attributes value
  ASSERT_EQ(objAttr->getMass(), 10.0);
  ASSERT_EQ(objAttr->getInertia(), Magnum::Vector3(3, 2, 1));

  // end test LoadObjects

  LOG(INFO) << "Starting test LoadLights";

  const auto& lightsLayoutAttributesMgr =
      MM_->getLightLayoutAttributesManager();
  // get # of loaded light layout attributes.
  int numLightAttrHandles = lightsLayoutAttributesMgr->getNumObjects();
  // Should be 3 : file based, copy and new
  ASSERT_EQ(numLightAttrHandles, 3);
  // get list of matching handles for base light config - should always only
  // be 1
  auto lightAttrHandles =
      lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
          "dataset_test_lights.lighting_config.json", true);
  ASSERT_EQ(lightAttrHandles.size(), 1);
  // get copy of attributes
  auto lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, lightAttr);
  // verify the number of lights within the lighting layout
  ASSERT_EQ(12, lightAttr->getNumLightInstances());

  // get list of matching handles for modified light config - should always
  // only be 1
  lightAttrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
      "modified_test_lights", true);
  ASSERT_EQ(lightAttrHandles.size(), 1);
  // get copy of attributes
  lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, lightAttr);
  // verify the number of lights within the lighting layout
  ASSERT_EQ(12, lightAttr->getNumLightInstances());

  // get list of matching handles for new light config - should always only be
  // 1
  lightAttrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
      "new_test_lights_0", true);
  ASSERT_EQ(lightAttrHandles.size(), 1);
  // get copy of attributes
  lightAttr =
      lightsLayoutAttributesMgr->getObjectCopyByHandle(lightAttrHandles[0]);
  // verify existence
  ASSERT_NE(nullptr, lightAttr);
  // verify the number of lights within the lighting layout
  ASSERT_EQ(3, lightAttr->getNumLightInstances());

  // end test LoadLights

  LOG(INFO) << "Starting test LoadSceneInstances";
  //
  // SHOULD NOT BE REFERENCED DIRECTLY IN USER CODE, but rather desired scene
  // instance should be acquired through MM.
  //
  const auto& sceneAttributesMgr = MM_->getSceneAttributesManager();
  // get # of loaded scene attributes.
  int numSceneHandles = sceneAttributesMgr->getNumObjects();
  // should be 1
  ASSERT_EQ(numSceneHandles, 1);
  // get handle list matching passed handle
  auto sceneAttrHandles = sceneAttributesMgr->getObjectHandlesBySubstring(
      "dataset_test_scene", true);
  // make sure there is only 1 matching dataset_test_scene
  ASSERT_EQ(sceneAttrHandles.size(), 1);

  const std::string activeSceneName = sceneAttrHandles[0];
  LOG(WARNING) << "testLoadSceneInstances : Scene instance attr handle : "
               << activeSceneName;
  // get scene instance attributes ref
  // metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
  auto sceneAttrs = MM_->getSceneAttributesByName(activeSceneName);
  // this should be a scene instance attributes with specific stage and object
  ASSERT_NE(sceneAttrs, nullptr);
  // verify default value for translation origin
  ASSERT_EQ(sceneAttrs->getTranslationOrigin(),
            static_cast<int>(AttrMgrs::SceneInstanceTranslationOrigin::COM));
  const int assetLocalInt =
      static_cast<int>(AttrMgrs::SceneInstanceTranslationOrigin::AssetLocal);

  //
  // miscellaneous scene instance attribute values
  //
  // default lighting name
  const std::string lightHandle = sceneAttrs->getLightingHandle();
  ASSERT_NE(lightHandle.find("modified_test_lights"), std::string::npos);
  // navmesh
  const std::string navmeshHandle = sceneAttrs->getNavmeshHandle();
  ASSERT_NE(navmeshHandle.find("navmesh_path1"), std::string::npos);
  // ssd
  const std::string ssdHandle = sceneAttrs->getSemanticSceneHandle();
  ASSERT_NE(ssdHandle.find("semantic_descriptor_path1"), std::string::npos);

  //
  // test stage instance
  //
  const auto stageInstanceAttrs = sceneAttrs->getStageInstance();
  // verify name
  const std::string stageName = stageInstanceAttrs->getHandle();
  ASSERT_NE(stageName.find("modified_test_stage"), std::string::npos);
  // verify translation origin to be asset_local
  ASSERT_EQ(stageInstanceAttrs->getTranslationOrigin(), assetLocalInt);
  // verify translation amount to be expected amount
  ASSERT_EQ(stageInstanceAttrs->getTranslation(),
            Magnum::Vector3(1.1, 2.2, 3.3));
  //
  // test object instances
  //
  // get all instances
  const std::vector<Attrs::SceneObjectInstanceAttributes::ptr>
      objInstanceAttrs = sceneAttrs->getObjectInstances();
  ASSERT_EQ(objInstanceAttrs.size(), 2);

  // first object instance
  const Attrs::SceneObjectInstanceAttributes::ptr& objAttr0 =
      objInstanceAttrs[0];
  // name
  std::string objName = objAttr0->getHandle();
  ASSERT_NE(objName.find("dataset_test_object1"), std::string::npos);
  // translation
  ASSERT_EQ(objAttr0->getTranslation(), Magnum::Vector3(0.1, 0.2, 0.3));
  // translation origin
  ASSERT_EQ(objAttr0->getTranslationOrigin(), assetLocalInt);

  // second object instance
  const Attrs::SceneObjectInstanceAttributes::ptr& objAttr1 =
      objInstanceAttrs[1];
  // name
  objName = objAttr1->getHandle();
  ASSERT_NE(objName.find("dataset_test_object2"), std::string::npos);
  // translation
  ASSERT_EQ(objAttr1->getTranslation(), Magnum::Vector3(0.3, 0.4, 0.5));
  // translation origin
  ASSERT_EQ(objAttr1->getTranslationOrigin(), assetLocalInt);

  // end test LoadSceneInstances

  LOG(INFO) << "Starting test LoadNavmesh";
  // get map of navmeshes
  const std::map<std::string, std::string> navmeshMap =
      MM_->getActiveNavmeshMap();
  // should have 2
  ASSERT_EQ(navmeshMap.size(), 2);

  // should hold 2 keys
  ASSERT_NE(navmeshMap.count("navmesh_path1"), 0);
  ASSERT_NE(navmeshMap.count("navmesh_path2"), 0);
  // each key should hold specific value
  ASSERT_EQ(navmeshMap.at("navmesh_path1"), "test_navmesh_path1");
  ASSERT_EQ(navmeshMap.at("navmesh_path2"), "test_navmesh_path2");
  // end test LoadNavmesh

  LOG(INFO) << "Starting test LoadSemanticScene";
  // get map of semantic scene instances
  const std::map<std::string, std::string> semanticMap =
      MM_->getActiveSemanticSceneDescriptorMap();
  // should have 2
  ASSERT_EQ(semanticMap.size(), 2);
  // should hold 2 keys
  ASSERT_NE(semanticMap.count("semantic_descriptor_path1"), 0);
  ASSERT_NE(semanticMap.count("semantic_descriptor_path2"), 0);
  // each key should hold specific value
  ASSERT_EQ(semanticMap.at("semantic_descriptor_path1"),
            "test_semantic_descriptor_path1");
  ASSERT_EQ(semanticMap.at("semantic_descriptor_path2"),
            "test_semantic_descriptor_path2");
  // end test LoadSemanticScene
}  // testDataset0

TEST_F(MetadataMediatorTest, testDataset1) {
  // primarily testing glob file wildcard loading
  initDataset1();

  LOG(INFO) << "Starting testDataset1 : test LoadStages";
  const auto& stageAttributesMgr = MM_->getStageAttributesManager();
  int numStageHandles = stageAttributesMgr->getNumObjects();
  // shoudld be 6 : one for default NONE stage, glob lookup yields 2 stages + 2
  // modified and 1 new stage in scene dataset config
  ASSERT_EQ(numStageHandles, 6);
  // end test LoadStages

  LOG(INFO) << "Starting test LoadObjects";
  const auto& objectAttributesMgr = MM_->getObjectAttributesManager();
  int numObjHandles = objectAttributesMgr->getNumFileTemplateObjects();
  // glob lookup yields 4 files + 2 modified in config
  ASSERT_EQ(numObjHandles, 6);
  // end test LoadObjects

  LOG(INFO) << "Starting test LoadLights";
  const auto& lightsLayoutAttributesMgr =
      MM_->getLightLayoutAttributesManager();
  // get # of loaded light layout attributes.
  int numLightAttrHandles = lightsLayoutAttributesMgr->getNumObjects();
  // Should be 4 : 2 file based, copy and new
  ASSERT_EQ(numLightAttrHandles, 4);

  // end test LoadLights

  LOG(INFO) << "Starting test LoadSceneInstances";
  //
  // SHOULD NOT BE REFERENCED DIRECTLY IN USER CODE, but rather desired scene
  // instance should be acquired through MM.
  //
  const auto& sceneAttributesMgr = MM_->getSceneAttributesManager();
  // get # of loaded scene attributes.
  int numSceneHandles = sceneAttributesMgr->getNumObjects();
  // should be 2 - 2 file based
  ASSERT_EQ(numSceneHandles, 2);

  // end test LoadSceneInstances

  LOG(INFO) << "Starting test LoadArticulatedObjects";

  namespace Dir = Cr::Utility::Directory;
  // verify # of urdf filepaths loaded - should be 6;
  const std::map<std::string, std::string>& urdfTestFilenames =
      MM_->getArticulatedObjectModelFilenames();
  ASSERT_EQ(urdfTestFilenames.size(), 6);
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
    ASSERT_EQ(shortHandle, iter->first);
    // test that file name ends in ".urdf"
    ASSERT_EQ(Dir::splitExtension(Dir::filename(iter->second))
                  .second.compare(".urdf"),
              0);
  }
  // end test LoadArticulatedObjects

  LOG(INFO) << "Starting test LoadNavmesh";
  // get map of navmeshes
  const std::map<std::string, std::string> navmeshMap =
      MM_->getActiveNavmeshMap();
  // should have 3
  ASSERT_EQ(navmeshMap.size(), 3);
  // end test LoadNavmesh

  LOG(INFO) << "Starting test LoadSemanticScene";
  // get map of semantic scene instances
  const std::map<std::string, std::string> semanticMap =
      MM_->getActiveSemanticSceneDescriptorMap();
  // should have 3
  ASSERT_EQ(semanticMap.size(), 3);
  // testLoadSemanticScene
}  // testDataset1

TEST_F(MetadataMediatorTest, testDatasetDelete) {
  // load dataset 1 and make active
  initDataset1();
  const std::string nameDS1 = MM_->getActiveSceneDatasetName();
  // verify the dataset has been added
  ASSERT_EQ(MM_->sceneDatasetExists(nameDS1), true);
  // verify the active dataset is the expected name
  ASSERT_EQ(nameDS1, sceneDatasetConfigFile_1);
  // get the stage attributes manager for the scene dataset
  const auto& stageAttrMgr_DS1 = MM_->getStageAttributesManager();
  // verify not nullptr
  ASSERT_NE(stageAttrMgr_DS1, nullptr);

  // load datsaet 0 and make active
  initDataset0();
  // get new active dataset
  const std::string nameDS0 = MM_->getActiveSceneDatasetName();
  // verify the dataset has been added
  ASSERT_EQ(MM_->sceneDatasetExists(nameDS0), true);
  // verify the active dataset is the expected name
  ASSERT_EQ(nameDS0, sceneDatasetConfigFile_0);

  // delete dataset 1 and verify delete
  ASSERT_EQ(MM_->removeSceneDataset(nameDS1), true);
  // verify the dataset does not exist anymore
  ASSERT_EQ(MM_->sceneDatasetExists(nameDS1), false);

  // verify deleted scene dataset's stage manager is nullptr
  ASSERT_EQ(stageAttrMgr_DS1, nullptr);

}  // testDatasetDelete

}  // namespace
