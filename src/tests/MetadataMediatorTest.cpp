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

using esp::metadata::MetadataMediator;

namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using esp::metadata::MetadataMediator;

using AttrMgrs::AttributesManager;
using Attrs::AbstractPrimitiveAttributes;
using Attrs::CapsulePrimitiveAttributes;
using Attrs::ConePrimitiveAttributes;
using Attrs::CubePrimitiveAttributes;
using Attrs::CylinderPrimitiveAttributes;
using Attrs::IcospherePrimitiveAttributes;
using Attrs::ObjectAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::SceneAttributes;
using Attrs::StageAttributes;
using Attrs::UVSpherePrimitiveAttributes;
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(DATA_DIR,
                                 "test_assets/testing.physics_config.json");
const std::string sceneDatasetConfigFile = Cr::Utility::Directory::join(
    DATA_DIR,
    "test_assets/dataset_tests/test_dataset.scene_dataset_config.json");

class MetadataMediatorTest : public testing::Test {
 protected:
  void SetUp() override {
    MM = MetadataMediator::create(sceneDatasetConfigFile);
  };
  void testLoadStages() {
    const auto stageAttributesMgr = MM->getStageAttributesManager();
    int numHandles = stageAttributesMgr->getNumObjects();
    // should be 4 - one for default NONE stage, one original, one based on
    // original but changed, and one new
    ASSERT_EQ(numHandles, 4);

    // get list of matching handles for base - should always only be 1
    auto attrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
        "dataset_test_stage.stage_config.json", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    auto attr = stageAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // get render asset handle for later comparison
    auto renderAssetHandle = attr->getRenderAssetHandle();
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify set to values in file
    ASSERT_EQ(attr->getScale(), Magnum::Vector3(2, 2, 2));
    ASSERT_EQ(attr->getRequiresLighting(), true);
    ASSERT_EQ(attr->getMargin(), 0.03);
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.3);
    ASSERT_EQ(attr->getRestitutionCoefficient(), 0.3);
    ASSERT_EQ(attr->getOrigin(), Magnum::Vector3(0, 0, 0));
    ASSERT_EQ(attr->getOrientUp(), Magnum::Vector3(0, 1, 0));
    ASSERT_EQ(attr->getOrientFront(), Magnum::Vector3(0, 0, -1));

    // get list of matching handles for modified base - should always only be 1
    attrHandles = stageAttributesMgr->getObjectHandlesBySubstring(
        "modified_test_stage", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = stageAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify set to override values in dataset_config file
    ASSERT_EQ(attr->getScale(), Magnum::Vector3(1, 1, 1));
    ASSERT_EQ(attr->getMargin(), 0.041);
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.4);
    ASSERT_EQ(attr->getRestitutionCoefficient(), 0.5);
    ASSERT_EQ(attr->getUnitsToMeters(), 1.0);

    // get list of matching handles for new - should always only be 1
    attrHandles =
        stageAttributesMgr->getObjectHandlesBySubstring("new_test_stage", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = stageAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify set to values in dataset_config file
    auto newRenderAssetHandle = attr->getRenderAssetHandle();
    // verify same renderasset handle to loaded stage attributes
    ASSERT_EQ(renderAssetHandle, newRenderAssetHandle);
    // verify values set correctly
    ASSERT_EQ(attr->getOrientUp(), Magnum::Vector3(0, -1, 0));
    ASSERT_EQ(attr->getScale(), Magnum::Vector3(2, 2, 2));
    ASSERT_EQ(attr->getGravity(), Magnum::Vector3(0, 9.8, 0));
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.35);
    ASSERT_EQ(attr->getRestitutionCoefficient(), 0.25);
    ASSERT_EQ(attr->getUnitsToMeters(), 2.0);
    ASSERT_EQ(attr->getRequiresLighting(), false);

    // get new attributes but don't register
    attr = stageAttributesMgr->createObject("new_default_attributes", false);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify contains default attributes value
    ASSERT_EQ(attr->getOrigin(), Magnum::Vector3(1.0, 2.0, 3.0));

  }  // testLoadStages

  void testLoadObjects() {
    const auto objectAttributesMgr = MM->getObjectAttributesManager();
    int numHandles = objectAttributesMgr->getNumFileTemplateObjects();
    // should be 6 file-based templates - 4 original, one based on an original
    // but changed, and one new
    ASSERT_EQ(numHandles, 6);

    // get list of matching handles for base object - should always only be 1
    auto attrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
        "dataset_test_object1.object_config.json", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    auto attr = objectAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify set to values in file
    ASSERT_EQ(attr->getScale(), Magnum::Vector3(1, 1, 1));
    ASSERT_EQ(attr->getRequiresLighting(), true);
    ASSERT_EQ(attr->getMargin(), 0.03);
    ASSERT_EQ(attr->getMass(), 0.038);
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.5);
    ASSERT_EQ(attr->getRestitutionCoefficient(), 0.2);
    ASSERT_EQ(attr->getOrientUp(), Magnum::Vector3(0, 1, 0));
    ASSERT_EQ(attr->getOrientFront(), Magnum::Vector3(0, 0, -1));
    ASSERT_EQ(attr->getUnitsToMeters(), 1.0);
    ASSERT_EQ(attr->getBoundingBoxCollisions(), false);
    ASSERT_EQ(attr->getJoinCollisionMeshes(), true);

    // get list of matching handles for modified base - should always only be 1
    attrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
        "modified_test_object1_1_slick_heavy", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = objectAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify identical to copied object except for override values in
    // dataset_config file
    ASSERT_EQ(attr->getScale(), Magnum::Vector3(1, 1, 1));
    ASSERT_EQ(attr->getRequiresLighting(), true);
    ASSERT_EQ(attr->getMargin(), 0.03);
    ASSERT_EQ(attr->getMass(), 3.5);
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.2);
    ASSERT_EQ(attr->getRestitutionCoefficient(), 0.2);
    ASSERT_EQ(attr->getOrientUp(), Magnum::Vector3(0, 1, 0));
    ASSERT_EQ(attr->getOrientFront(), Magnum::Vector3(0, 0, -1));
    ASSERT_EQ(attr->getUnitsToMeters(), 1.0);
    ASSERT_EQ(attr->getBoundingBoxCollisions(), false);
    ASSERT_EQ(attr->getJoinCollisionMeshes(), true);

    // get list of matching handles for new - should always only be 1
    attrHandles = objectAttributesMgr->getObjectHandlesBySubstring(
        "new_test_object3", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = objectAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify set to values in dataset_config file
    auto newRenderAssetHandle = attr->getRenderAssetHandle();
    // verify same renderasset handle to loaded stage attributes
    ASSERT_NE(std::string::npos,
              newRenderAssetHandle.find("dataset_test_object3.glb"));
    // verify values set correctly

    ASSERT_EQ(attr->getMass(), 1.1);
    ASSERT_EQ(attr->getFrictionCoefficient(), 0.1);
    ASSERT_EQ(attr->getInertia(), Magnum::Vector3(3, 2, 1));

    // get new attributes but don't register
    attr = objectAttributesMgr->createObject("new_default_attributes", false);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify contains default attributes value
    ASSERT_EQ(attr->getMass(), 10.0);
    ASSERT_EQ(attr->getInertia(), Magnum::Vector3(3, 2, 1));
  }  // testLoadObjects

  void testLoadLights() {
    const auto lightsLayoutAttributesMgr =
        MM->getLightLayoutAttributesManager();
    // get # of loaded light layout attributes.
    int numHandles = lightsLayoutAttributesMgr->getNumObjects();
    // Should be 3 : file based, copy and new
    ASSERT_EQ(numHandles, 3);
    // get list of matching handles for base light config - should always only
    // be 1
    auto attrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
        "dataset_test_lights.lighting_config.json", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    auto attr =
        lightsLayoutAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify the number of lights within the lighting layout
    ASSERT_EQ(12, attr->getNumLightInstances());

    // get list of matching handles for modified light config - should always
    // only be 1
    attrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
        "modified_test_lights", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = lightsLayoutAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify the number of lights within the lighting layout
    ASSERT_EQ(12, attr->getNumLightInstances());

    // get list of matching handles for new light config - should always only be
    // 1
    attrHandles = lightsLayoutAttributesMgr->getObjectHandlesBySubstring(
        "new_test_lights_0", true);
    ASSERT_EQ(attrHandles.size(), 1);
    // get copy of attributes
    attr = lightsLayoutAttributesMgr->getObjectCopyByHandle(attrHandles[0]);
    // verify existence
    ASSERT_NE(nullptr, attr);
    // verify the number of lights within the lighting layout
    ASSERT_EQ(3, attr->getNumLightInstances());

  }  // testLoadLights

  void testLoadSceneInstances() {
    const auto sceneAttributesMgr = MM->getSceneAttributesManager();

  }  // testLoadSceneInstances

  void testLoadNavmesh() {
    // get map of navmeshes
    const auto navmeshMap = MM->getActiveNavmeshMap();
    // should have 2
    ASSERT_EQ(navmeshMap.size(), 2);

    // should hold 2 keys
    ASSERT_NE(navmeshMap.count("navmesh_path1"), 0);
    ASSERT_NE(navmeshMap.count("navmesh_path2"), 0);
    // each key should hold specific value
    ASSERT_EQ(navmeshMap.at("navmesh_path1"), "test_navmesh_path1");
    ASSERT_EQ(navmeshMap.at("navmesh_path2"), "test_navmesh_path2");
  }  // testLoadNavmesh

  void testLoadSemanticScene() {
    // get map of semantic scene instances
    const auto semanticMap = MM->getActiveSemanticSceneDescriptorMap();
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
  }  // testLoadSemanticScene

  MetadataMediator::ptr MM = nullptr;

};  // class MetadataMediatorTest

/**
 * @brief This test will verify that the physics attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(MetadataMediatorTest, MetadataMediatorTest_CreateTestDataset) {
  LOG(INFO) << "Starting "
               "MetadataMediatorTest::MetadataMediatorTest_CreateTestDataset : "
               "Dataset name : "
            << sceneDatasetConfigFile;
  // verify dataset name
  ASSERT_EQ(MM->getActiveSceneDatasetName(), sceneDatasetConfigFile);

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadStages";
  testLoadStages();

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadObjects";
  testLoadObjects();

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadLights";
  testLoadLights();

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadSceneInstances";
  testLoadSceneInstances();

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadNavmesh";
  testLoadNavmesh();

  LOG(INFO) << "Starting "
               "MetadataMediatorTest::testLoadSemanticScene";
  testLoadSemanticScene();

}  // MetadataMediatorTest, MetadataMediatorTest_CreateTestDataset
