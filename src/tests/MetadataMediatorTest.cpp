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
const std::string datasetConfigFile = Cr::Utility::Directory::join(
    DATA_DIR,
    "test_assets/dataset_tests/test_dataset.dataset_config.json");

class MetadataMediatorTest : public testing::Test {
 protected:
  void SetUp() override { MM = MetadataMediator::create(datasetConfigFile); };

  const std::string dataset_json = R"(
{
    "stages": {
        "default_attributes": {
            "origin":[1.0,2.0,3.0]
        },
        "paths": ["stages"],
        "configs" : [
        {
            "original_file": "dataset_test_stage.stage_config.json",
            "template_handle": "modified_test_stage",
            "attributes": {
				"scale":[1,1,1],
				"gravity":[0,-9.8,0],
				"margin":0.041,
				"friction_coefficient": 0.4,
				"restitution_coefficient": 0.5,
				"units_to_meters":1.0
             }
        },
        {
            "template_handle": "new_test_stage",
            "asset_source_dir": "stages",
            "attributes": {
				"render_asset": "dataset_test_stage.glb",
				"up":[0,-1,0],
				"scale":[2,2,2],
				"gravity":[0,9.8,0],
				"friction_coefficient": 0.35,
				"restitution_coefficient": 0.25,
				"units_to_meters":2.0,
				"requires_lighting":false
            }
        }
        ]
    },
    "objects":{
        "default_attributes": {
			"mass" : 10.0,
			"inertia": [3,2,1]
        },
        "paths": ["objects"],
        "configs" : [
        {
            "original_file": "dataset_test_object1.object_config.json",
            "template_handle": "modified_test_object1_1_slick_heavy",
            "attributes": {
				"friction_coefficient": 0.2,
				"mass": 3.5
            }
        },
		{
            "template_handle": "new_test_object3",
            "asset_source_dir": "objects",
            "attributes": {
				"render_asset": "dataset_test_object3.glb",
				"friction_coefficient": 0.1,
				"mass": 1.1
            }
        }

        ]
    },
    "light_setups":{
        "default_attributes": {
        },
        "paths": ["lights"],
        "configs" : [
        {
            "original_file":"dataset_test_lights.lighting_config.json",
            "template_handle": "modified_test_lights",
            "attributes": {
  			    "lights":{
					"0": { "position": [1.5,0.1,1.5], "intensity": 2.4, "color": [0.5,1,0.95], "type": "point"},
					"1": { "position": [2.5,-0.1,2.5], "intensity": 2.1, "color": [0.5,1,0.95], "type": "point"},
					"11": { "position": [3.5,-0.7,-3.5], "intensity": -0.5, "color": [1,0.5,1], "type": "point"}
				}
           }
        },
        {
            "template_handle": "new_test_lights_0",
            "attributes": {
  			    "lights":{
					"3": { "position": [11.5,10.1,11.5], "intensity": 1.4, "color": [0.5,1,0.95], "type": "point"},
					"4": { "position": [12.5,-10.1,12.5], "intensity": 1.1, "color": [0.5,1,0.95], "type": "point"},
					"5": { "position": [13.5,-10.7,-13.5], "intensity": -1.5, "color": [1,0.5,1], "type": "point"}
				}
            }
        }
        ]
    },
	"scene_instances":{
		"paths":["scenes"]
	},
    "navmesh_instances":{
        "navmesh_path1":"test_navmesh_path1",
        "navmesh_path2":"test_navmesh_path2"
    },
    "semantic_scene_descriptor_instances": {
        "semantic_descriptor_path1":"test_semantic_descriptor_path1",
        "semantic_descriptor_path2":"test_semantic_descriptor_path2"
    }
}
)";

  void testLoadStages() {
    const auto stageAttributesMgr = MM->getStageAttributesManager();
    int numHandles = stageAttributesMgr->getNumObjects();
    // should be 3 - one original, one based on original but changed, and one
    // new
    ASSERT_EQ(numHandles, 3);

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
    const auto lightsAttributesMgr = MM->getLightAttributesManager();
    // get # of loaded light attributes.

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
            << datasetConfigFile;
  // verify dataset name
  ASSERT_EQ(MM->getActiveDatasetName(), datasetConfigFile);

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
