// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include <string>

#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

#include "esp/physics/RigidBase.h"

#include "configure.h"

namespace {
namespace Cr = Corrade;

using Magnum::Math::Literals::operator""_radf;
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using esp::metadata::MetadataMediator;
using esp::metadata::PrimObjTypes;

using esp::physics::MotionType;

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

class AttributesManagersTest : public testing::Test {
 protected:
  void SetUp() override {
    // set up a default simulation config to initialize MM
    auto cfg = esp::sim::SimulatorConfiguration{};
    auto MM = MetadataMediator::create(cfg);
    // get attributes managers for default dataset
    assetAttributesManager_ = MM->getAssetAttributesManager();
    lightLayoutAttributesManager_ = MM->getLightLayoutAttributesManager();
    objectAttributesManager_ = MM->getObjectAttributesManager();
    physicsAttributesManager_ = MM->getPhysicsAttributesManager();
    sceneAttributesManager_ = MM->getSceneAttributesManager();
    stageAttributesManager_ = MM->getStageAttributesManager();
  };

  /**
   * @brief Test loading from JSON
   * @tparam T Class of attributes manager
   * @tparam U Class of attributes
   * @param mgr the Attributes Manager being tested
   * @return attributes template built from JSON parsed from string
   */
  template <typename T, typename U>
  std::shared_ptr<U> testBuildAttributesFromJSONString(
      std::shared_ptr<T> mgr,
      const std::string& jsonString) {
    // create JSON document
    try {
      esp::io::JsonDocument tmp = esp::io::parseJsonString(jsonString);
      // io::JsonGenericValue :
      const esp::io::JsonGenericValue jsonDoc = tmp.GetObject();
      // create an empty template
      std::shared_ptr<U> attrTemplate1 =
          mgr->buildManagedObjectFromDoc("new_template_from_json", jsonDoc);

      return attrTemplate1;
    } catch (...) {
      LOG(ERROR) << "testBuildAttributesFromJSONString : Failed to parse "
                 << jsonString << " as JSON.";
      return nullptr;
    }

  }  // testBuildAttributesFromJSONString

  /**
   * @brief Test creation, copying and removal of templates for Object, Physics
   * and Stage Attributes Managers
   * @tparam Class of attributes manager
   * @param mgr the Attributes Manager being tested,
   * @param handle the handle of the desired attributes template to work with
   */
  template <typename T>
  void testCreateAndRemove(std::shared_ptr<T> mgr, const std::string& handle) {
    // meaningless key to modify attributes for verifcation of behavior
    std::string keyStr = "tempKey";
    // get starting number of templates
    int orignNumTemplates = mgr->getNumObjects();
    // verify template is not present - should not be
    bool isPresentAlready = mgr->getObjectLibHasHandle(handle);
    ASSERT_NE(isPresentAlready, true);

    // create template from source handle, register it and retrieve it
    // Note: registration of template means this is a copy of registered
    // template
    auto attrTemplate1 = mgr->createObject(handle, true);
    // verify it exists
    ASSERT_NE(nullptr, attrTemplate1);
    // retrieve a copy of the named attributes template
    auto attrTemplate2 = mgr->getObjectOrCopyByHandle(handle);
    // verify copy has same quantities and values as original
    ASSERT_EQ(attrTemplate1->getHandle(), attrTemplate2->getHandle());

    // test changing a user-defined field in each template, verify the templates
    // are not now the same
    attrTemplate1->setString(keyStr, "temp");
    attrTemplate2->setString(keyStr, "temp2");
    ASSERT_NE(attrTemplate1->getString(keyStr),
              attrTemplate2->getString(keyStr));
    // get original template ID
    int oldID = attrTemplate1->getID();

    // register modified template and verify that this is the template now
    // stored
    int newID = mgr->registerObject(attrTemplate2, handle);
    // verify IDs are the same
    ASSERT_EQ(oldID, newID);

    // get another copy
    auto attrTemplate3 = mgr->getObjectOrCopyByHandle(handle);
    // verify added field is present and the same
    ASSERT_EQ(attrTemplate3->getString(keyStr),
              attrTemplate2->getString(keyStr));
    // change field in new copy
    attrTemplate3->setString(keyStr, "temp3");
    // verify that now they are different
    ASSERT_NE(attrTemplate3->getString(keyStr),
              attrTemplate2->getString(keyStr));

    // test removal
    int removeID = attrTemplate2->getID();
    // remove template by ID, acquire copy of removed template
    auto oldTemplate = mgr->removeObjectByID(removeID);
    // verify it exists
    ASSERT_NE(nullptr, oldTemplate);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumObjects());
    // re-add template copy via registration
    int newAddID = mgr->registerObject(attrTemplate2, handle);
    // verify IDs are the same
    ASSERT_EQ(removeID, newAddID);

    // lock template referenced by handle
    bool success = mgr->setLock(handle, true);
    // attempt to remove attributes via handle
    auto oldTemplate2 = mgr->removeObjectByHandle(handle);
    // verify no template was deleted
    ASSERT_EQ(nullptr, oldTemplate2);
    // unlock template
    success = mgr->setLock(handle, false);

    // remove  attributes via handle
    auto oldTemplate3 = mgr->removeObjectByHandle(handle);
    // verify deleted template  exists
    ASSERT_NE(nullptr, oldTemplate3);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumObjects());

  }  // AttributesManagersTest::testCreateAndRemove

  /**
   * @brief Test creation, copying and removal of templates for lights
   * attributes managers.
   * @param mgr the Attributes Manager being tested,
   * @param handle the handle of the desired attributes template to work with
   */

  void testCreateAndRemoveLights(
      AttrMgrs::LightLayoutAttributesManager::ptr mgr,
      const std::string& handle) {
    // meaningless key to modify attributes for verifcation of behavior
    std::string keyStr = "tempKey";
    // get starting number of templates
    int orignNumTemplates = mgr->getNumObjects();

    // Source config for lights holds multiple light configurations.
    // Create a single template for each defined light in configuration and
    // register it.
    mgr->createObject(handle, true);
    // get number of templates loaded
    int numLoadedLights = mgr->getNumObjects();
    // verify lights were added
    ASSERT_NE(numLoadedLights, orignNumTemplates);

    // get handles of all lights added
    auto lightHandles = mgr->getObjectHandlesBySubstring();
    ASSERT_EQ(lightHandles.size(), numLoadedLights);

    // remove all added handles
    for (auto handle : lightHandles) {
      mgr->removeObjectByHandle(handle);
    }
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumObjects());

  }  // AttributesManagersTest::testCreateAndRemove

  /**
   * @brief Test creation many templates and removing all but defaults.
   * @tparam Class of attributes manager
   * @param mgr the Attributes Manager being tested,
   * @param renderHandle a legal render handle to set for the new template so
   * that registration won't fail.
   */
  template <typename T>
  void testRemoveAllButDefault(std::shared_ptr<T> mgr,
                               const std::string& handle,
                               bool setRenderHandle) {
    // get starting number of templates
    int orignNumTemplates = mgr->getNumObjects();
    // lock all current handles
    std::vector<std::string> origHandles =
        mgr->setLockBySubstring(true, "", true);
    // make sure we have locked all original handles
    ASSERT_EQ(orignNumTemplates, origHandles.size());

    // create multiple new templates, and then test deleting all those created
    // using single command.
    int numToAdd = 10;
    for (int i = 0; i < numToAdd; ++i) {
      // assign template a handle
      std::string newHandleIter("newTemplateHandle_" + std::to_string(i));
      // create a template with a legal handle
      auto attrTemplate1 = mgr->createObject(handle, false);
      // register template with new handle
      int tmpltID = mgr->registerObject(attrTemplate1, newHandleIter);
      // verify template added
      ASSERT_NE(tmpltID, -1);
      auto attrTemplate2 = mgr->getObjectOrCopyByHandle(newHandleIter);
      // verify added template  exists
      ASSERT_NE(nullptr, attrTemplate2);
    }

    // now delete all templates that
    auto removedNamedTemplates =
        mgr->removeObjectsBySubstring("newTemplateHandle_", true);
    // verify that the number removed == the number added
    ASSERT_EQ(removedNamedTemplates.size(), numToAdd);

    // re-add templates
    for (auto& tmplt : removedNamedTemplates) {
      // register template with new handle
      int tmpltID = mgr->registerObject(tmplt);
      // verify template added
      ASSERT_NE(tmpltID, -1);
      auto attrTemplate2 = mgr->getObjectOrCopyByHandle(tmplt->getHandle());
      // verify added template  exists
      ASSERT_NE(nullptr, attrTemplate2);
    }

    // now delete all templates that have just been added
    auto removedTemplates = mgr->removeAllObjects();
    // verify that the number removed == the number added
    ASSERT_EQ(removedTemplates.size(), numToAdd);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumObjects());

    // unlock all original handles
    std::vector<std::string> newOrigHandles =
        mgr->setLockByHandles(origHandles, false);
    // verify orig handles are those that have been unlocked
    ASSERT_EQ(newOrigHandles, origHandles);
    // make sure we have unlocked all original handles
    ASSERT_EQ(orignNumTemplates, newOrigHandles.size());

  }  // AttributesManagersTest::testRemoveAllButDefault

  /**
   * @brief Test creation, copying and removal of new default/empty templates
   * for Object, Physics and Stage Attributes Managers
   * @tparam Class of attributes manager
   * @param mgr the Attributes Manager being tested,
   * @param renderHandle a legal render handle to set for the new template so
   * that registration won't fail.
   */
  template <typename T>
  void testCreateAndRemoveDefault(std::shared_ptr<T> mgr,
                                  const std::string& handle,
                                  bool setRenderHandle) {
    // get starting number of templates
    int orignNumTemplates = mgr->getNumObjects();
    // assign template a handle
    std::string newHandle = "newTemplateHandle";

    // create new template but do not register it
    auto newAttrTemplate0 = mgr->createDefaultObject(newHandle, false);
    // verify real template was returned
    ASSERT_NE(nullptr, newAttrTemplate0);

    // create template from source handle, register it and retrieve it
    // Note: registration of template means this is a copy of registered
    // template
    if (setRenderHandle) {
      auto attrTemplate1 = mgr->createObject(handle, false);
      // set legitimate render handle in template
      newAttrTemplate0->set(
          "render_asset",
          attrTemplate1->template get<std::string>("render_asset"));
    }

    // register modified template and verify that this is the template now
    // stored
    int newID = mgr->registerObject(newAttrTemplate0, newHandle);

    // get a copy of added template
    auto attrTemplate3 = mgr->getObjectOrCopyByHandle(newHandle);

    // remove new template by name
    auto newAttrTemplate1 = mgr->removeObjectByHandle(newHandle);

    // verify it exists
    ASSERT_NE(nullptr, newAttrTemplate1);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumObjects());

  }  // AttributesManagersTest::testCreateAndRemoveDefault

  /**
   * @brief Test creation, copying and removal of templates for primitive
   * assets.
   * @tparam Class of attributes being managed
   * @param defaultAttribs the default template of the passed type T
   * @param ctorModField the name of the modified field of type @ref U that
   * impacts the constructor.
   * @param legalVal a legal value of ctorModField; This should be different
   * than template default for @ref ctorModField.
   * @param illegalVal a legal value of ctorModField.  If null ptr then no
   * illegal values possible.
   */
  template <typename T>
  void testAssetAttributesModRegRemove(std::shared_ptr<T> defaultAttribs,
                                       const std::string& ctorModField,
                                       int legalVal,
                                       int const* illegalVal) {
    // get starting number of templates
    int orignNumTemplates = assetAttributesManager_->getNumObjects();

    // get name of default template
    std::string oldHandle = defaultAttribs->getHandle();

    // verify default template is valid
    bool isTemplateValid = defaultAttribs->isValidTemplate();
    ASSERT_EQ(isTemplateValid, true);

    // if illegal values are possible
    if (nullptr != illegalVal) {
      // modify template value used by primitive constructor (will change
      // name) illegal modification
      defaultAttribs->set(ctorModField, *illegalVal);
      // verify template is not valid
      bool isTemplateValid = defaultAttribs->isValidTemplate();
      ASSERT_NE(isTemplateValid, true);
    }
    // legal modification, different than default
    defaultAttribs->set(ctorModField, legalVal);
    // verify template is valid
    isTemplateValid = defaultAttribs->isValidTemplate();
    ASSERT_EQ(isTemplateValid, true);
    // rebuild handle to reflect new parameters
    defaultAttribs->buildHandle();

    // get synthesized handle
    std::string newHandle = defaultAttribs->getHandle();
    LOG(INFO) << "Modified Template Handle : " << newHandle;
    // register modified template
    assetAttributesManager_->registerObject(defaultAttribs);

    // verify new handle is in template library
    // get template by handle
    ASSERT(assetAttributesManager_->getObjectLibHasHandle(newHandle));
    // verify old template is still present as well
    ASSERT(assetAttributesManager_->getObjectLibHasHandle(oldHandle));

    // get new template
    std::shared_ptr<T> newAttribs =
        assetAttributesManager_->getObjectOrCopyByHandle<T>(newHandle);
    // verify template has modified values
    int newValue = newAttribs->template get<int>(ctorModField);
    ASSERT_EQ(legalVal, newValue);
    // remove modified template via handle
    auto oldTemplate2 =
        assetAttributesManager_->removeObjectByHandle(newHandle);
    // verify deleted template  exists
    ASSERT_NE(nullptr, oldTemplate2);

    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, assetAttributesManager_->getNumObjects());

  }  // AttributesManagersTest::testAssetAttributesModRegRemove

  AttrMgrs::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;
  AttrMgrs::LightLayoutAttributesManager::ptr lightLayoutAttributesManager_ =
      nullptr;
  AttrMgrs::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;
  AttrMgrs::SceneAttributesManager::ptr sceneAttributesManager_ = nullptr;
  AttrMgrs::StageAttributesManager::ptr stageAttributesManager_ = nullptr;
};  // class AttributesManagersTest

/**
 * @brief This test will verify that the physics attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_PhysicsJSONLoadTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::AttributesManagers_PhysicsJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
      "physics_simulator": "bullet_test",
      "timestep": 1.0,
      "gravity": [1,2,3],
      "friction_coefficient": 1.4,
      "restitution_coefficient": 1.1
    })";
  auto physMgrAttr =
      testBuildAttributesFromJSONString<AttrMgrs::PhysicsAttributesManager,
                                        Attrs::PhysicsManagerAttributes>(
          physicsAttributesManager_, jsonString);
  // verify exists
  ASSERT_NE(nullptr, physMgrAttr);
  // match values set in test JSON
  // TODO : get these values programmatically?
  ASSERT_EQ(physMgrAttr->getGravity(), Magnum::Vector3(1, 2, 3));
  ASSERT_EQ(physMgrAttr->getTimestep(), 1.0);
  ASSERT_EQ(physMgrAttr->getSimulator(), "bullet_test");
  ASSERT_EQ(physMgrAttr->getFrictionCoefficient(), 1.4);
  ASSERT_EQ(physMgrAttr->getRestitutionCoefficient(), 1.1);
}  // AttributesManagers_PhysicsJSONLoadTest

/**
 * @brief This test will verify that the Light Attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_LightJSONLoadTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::AttributesManagers_LightJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
  "lights":{
      "test":{
        "position": [2.5,0.1,3.8],
        "direction": [1.0,-1.0,1.0],
        "intensity": -0.1,
        "color": [2,1,-1],
        "type": "directional",
        "spot": {
          "innerConeAngle": -0.75,
          "outerConeAngle": -1.57
        }
      }
    }
  })";

  auto lightLayoutAttr =
      testBuildAttributesFromJSONString<AttrMgrs::LightLayoutAttributesManager,
                                        Attrs::LightLayoutAttributes>(
          lightLayoutAttributesManager_, jsonString);
  // verify exists
  ASSERT_NE(nullptr, lightLayoutAttr);

  auto lightAttr = lightLayoutAttr->getLightInstance("test");
  // verify that lightAttr exists
  ASSERT_NE(nullptr, lightAttr);

  // match values set in test JSON
  // TODO : get these values programmatically?
  ASSERT_EQ(lightAttr->getPosition(), Magnum::Vector3(2.5, 0.1, 3.8));
  ASSERT_EQ(lightAttr->getDirection(), Magnum::Vector3(1.0, -1.0, 1.0));
  ASSERT_EQ(lightAttr->getColor(), Magnum::Vector3(2, 1, -1));
  ASSERT_EQ(lightAttr->getIntensity(), -0.1);
  ASSERT_EQ(lightAttr->getType(),
            static_cast<int>(esp::gfx::LightType::Directional));
  ASSERT_EQ(lightAttr->getInnerConeAngle(), -0.75_radf);
  ASSERT_EQ(lightAttr->getOuterConeAngle(), -1.57_radf);
}  // AttributesManagers_LightJSONLoadTest
/**
 * @brief This test will verify that the Scene Instance attributes' managers'
 * JSON loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_SceneInstanceJSONLoadTest) {
  LOG(INFO)
      << "Starting "
         "AttributesManagersTest::AttributesManagers_SceneInstanceJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString =
      R"({
      "translation_origin" : "Asset_Local",
      "stage_instance":{
          "template_name": "test_stage_template",
          "translation": [1,2,3],
          "rotation": [0.1, 0.2, 0.3, 0.4]
      },
      "object_instances": [
          {
              "template_name": "test_object_template0",
              "translation_origin": "COM",
              "translation": [0,1,2],
              "rotation": [0.2, 0.3, 0.4, 0.5],
              "motion_type": "KINEMATIC"
          },
          {
              "template_name": "test_object_template1",
              "translation": [0,-1,-2],
              "rotation": [0.5, 0.6, 0.7, 0.8],
              "motion_type": "DYNAMIC"
          }
      ],
      "default_lighting":  "test_lighting_configuration",
      "navmesh_instance": "test_navmesh_path1",
      "semantic_scene_instance": "test_semantic_descriptor_path1"
     })";

  auto sceneAttr =
      testBuildAttributesFromJSONString<AttrMgrs::SceneAttributesManager,
                                        Attrs::SceneAttributes>(
          sceneAttributesManager_, jsonString);

  // verify exists
  ASSERT_NE(nullptr, sceneAttr);

  // match values set in test JSON
  // TODO : get these values programmatically?
  ASSERT_EQ(
      sceneAttr->getTranslationOrigin(),
      static_cast<int>(AttrMgrs::SceneInstanceTranslationOrigin::AssetLocal));
  ASSERT_EQ(sceneAttr->getLightingHandle(), "test_lighting_configuration");
  ASSERT_EQ(sceneAttr->getNavmeshHandle(), "test_navmesh_path1");
  ASSERT_EQ(sceneAttr->getSemanticSceneHandle(),
            "test_semantic_descriptor_path1");
  // verify stage populated properly
  auto stageInstance = sceneAttr->getStageInstance();
  ASSERT_EQ(stageInstance->getHandle(), "test_stage_template");
  ASSERT_EQ(stageInstance->getTranslation(), Magnum::Vector3(1, 2, 3));
  // verify objects
  auto objectInstanceList = sceneAttr->getObjectInstances();
  ASSERT_EQ(objectInstanceList.size(), 2);
  auto objInstance = objectInstanceList[0];
  ASSERT_EQ(objInstance->getHandle(), "test_object_template0");
  ASSERT_EQ(objInstance->getTranslationOrigin(),
            static_cast<int>(AttrMgrs::SceneInstanceTranslationOrigin::COM));
  ASSERT_EQ(objInstance->getTranslation(), Magnum::Vector3(0, 1, 2));
  ASSERT_EQ(objInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::KINEMATIC));

  objInstance = objectInstanceList[1];
  ASSERT_EQ(objInstance->getHandle(), "test_object_template1");
  ASSERT_EQ(objInstance->getTranslation(), Magnum::Vector3(0, -1, -2));
  ASSERT_EQ(objInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::DYNAMIC));

}  // AttributesManagers_SceneInstanceJSONLoadTest

/**
 * @brief This test will verify that the Stage attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_StageJSONLoadTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::AttributesManagers_StageJSONLoadTest";

  // build JSON sample config
  const std::string& jsonString =
      R"({
        "scale":[2,3,4],
        "margin": 0.9,
        "friction_coefficient": 0.321,
        "restitution_coefficient": 0.456,
        "requires_lighting": false,
        "units_to_meters": 1.1,
        "up":[2.1,0,0],
        "front":[0,2.1,0],
        "render_asset": "testJSONRenderAsset.glb",
        "collision_asset": "testJSONCollisionAsset.glb",
        "is_collidable": false,
        "gravity": [9,8,7],
        "origin":[1,2,3],
        "semantic_asset":"testJSONSemanticAsset.glb",
        "nav_asset":"testJSONNavMeshAsset.glb",
        "house_filename":"testJSONHouseFileName.glb"
      })";

  auto stageAttr =
      testBuildAttributesFromJSONString<AttrMgrs::StageAttributesManager,
                                        Attrs::StageAttributes>(
          stageAttributesManager_, jsonString);
  // verify exists
  ASSERT_NE(nullptr, stageAttr);
  // match values set in test JSON
  // TODO : get these values programmatically?
  ASSERT_EQ(stageAttr->getScale(), Magnum::Vector3(2, 3, 4));
  ASSERT_EQ(stageAttr->getMargin(), 0.9);
  ASSERT_EQ(stageAttr->getFrictionCoefficient(), 0.321);
  ASSERT_EQ(stageAttr->getRestitutionCoefficient(), 0.456);
  ASSERT_EQ(stageAttr->getRequiresLighting(), false);
  ASSERT_EQ(stageAttr->getUnitsToMeters(), 1.1);
  ASSERT_EQ(stageAttr->getOrientUp(), Magnum::Vector3(2.1, 0, 0));
  ASSERT_EQ(stageAttr->getOrientFront(), Magnum::Vector3(0, 2.1, 0));
  ASSERT_EQ(stageAttr->getRenderAssetHandle(), "testJSONRenderAsset.glb");
  ASSERT_EQ(stageAttr->getCollisionAssetHandle(), "testJSONCollisionAsset.glb");
  ASSERT_EQ(stageAttr->getIsCollidable(), false);
  // stage-specific attributes
  ASSERT_EQ(stageAttr->getGravity(), Magnum::Vector3(9, 8, 7));
  ASSERT_EQ(stageAttr->getOrigin(), Magnum::Vector3(1, 2, 3));
  ASSERT_EQ(stageAttr->getSemanticAssetHandle(), "testJSONSemanticAsset.glb");
  ASSERT_EQ(stageAttr->getNavmeshAssetHandle(), "testJSONNavMeshAsset.glb");
  ASSERT_EQ(stageAttr->getHouseFilename(), "testJSONHouseFileName.glb");
}  // AttributesManagers_StageJSONLoadTest

/**
 * @brief This test will verify that the Object attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_ObjectJSONLoadTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::AttributesManagers_ObjectJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString =
      R"({
        "scale":[2,3,4],
        "margin": 0.9,
        "friction_coefficient": 0.321,
        "restitution_coefficient": 0.456,
        "requires_lighting": false,
        "units_to_meters": 1.1,
        "up":[2.1,0,0],
        "front":[0,2.1,0],
        "render_asset": "testJSONRenderAsset.glb",
        "collision_asset": "testJSONCollisionAsset.glb",
        "is_collidable": false,
        "mass": 9,
        "use_bounding_box_for_collision": true,
        "join_collision_meshes":true,
        "inertia": [1.1, 0.9, 0.3],
        "semantic_id" : 7,
        "COM": [0.1,0.2,0.3]
      })";
  auto objAttr =
      testBuildAttributesFromJSONString<AttrMgrs::ObjectAttributesManager,
                                        Attrs::ObjectAttributes>(
          objectAttributesManager_, jsonString);
  // verify exists
  ASSERT_NE(nullptr, objAttr);
  // match values set in test JSON
  // TODO : get these values programmatically?
  ASSERT_EQ(objAttr->getScale(), Magnum::Vector3(2, 3, 4));
  ASSERT_EQ(objAttr->getMargin(), 0.9);
  ASSERT_EQ(objAttr->getFrictionCoefficient(), 0.321);
  ASSERT_EQ(objAttr->getRestitutionCoefficient(), 0.456);
  ASSERT_EQ(objAttr->getRequiresLighting(), false);
  ASSERT_EQ(objAttr->getUnitsToMeters(), 1.1);
  ASSERT_EQ(objAttr->getOrientUp(), Magnum::Vector3(2.1, 0, 0));
  ASSERT_EQ(objAttr->getOrientFront(), Magnum::Vector3(0, 2.1, 0));
  ASSERT_EQ(objAttr->getRenderAssetHandle(), "testJSONRenderAsset.glb");
  ASSERT_EQ(objAttr->getCollisionAssetHandle(), "testJSONCollisionAsset.glb");
  ASSERT_EQ(objAttr->getIsCollidable(), false);
  ASSERT_EQ(objAttr->getSemanticId(), 7);
  // object-specific attributes
  ASSERT_EQ(objAttr->getMass(), 9);
  ASSERT_EQ(objAttr->getBoundingBoxCollisions(), true);
  ASSERT_EQ(objAttr->getJoinCollisionMeshes(), true);
  ASSERT_EQ(objAttr->getInertia(), Magnum::Vector3(1.1, 0.9, 0.3));
  ASSERT_EQ(objAttr->getCOM(), Magnum::Vector3(0.1, 0.2, 0.3));

}  // AttributesManagersTest::AttributesManagers_ObjectJSONLoadTest

/**
 * @brief This test will test creating, modifying, registering and deleting
 * Attributes via Attributes Mangers for PhysicsManagerAttributes. These
 * tests should be consistent with most types of future attributes managers
 * specializing the AttributesManager class template that follow the same
 * expected behavior paths as extent attributes/attributesManagers.  Note :
 * PrimitiveAssetAttributes exhibit slightly different behavior and need their
 * own tests.
 */
TEST_F(AttributesManagersTest, PhysicsAttributesManagersCreate) {
  LOG(INFO)
      << "Starting AttributesManagersTest::PhysicsAttributesManagersCreate";

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "PhysicsAttributesManager @ "
            << physicsConfigFile;

  // physics attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::PhysicsAttributesManager>(
      physicsAttributesManager_, physicsConfigFile);
  testCreateAndRemoveDefault<AttrMgrs::PhysicsAttributesManager>(
      physicsAttributesManager_, physicsConfigFile, false);
}  // AttributesManagersTest::PhysicsAttributesManagersCreate

/**
 * @brief This test will test creating, modifying, registering and deleting
 * Attributes via Attributes Mangers for StageAttributes.  These
 * tests should be consistent with most types of future attributes managers
 * specializing the AttributesManager class template that follow the same
 * expected behavior paths as extent attributes/attributesManagers.  Note :
 * PrimitiveAssetAttributes exhibit slightly different behavior and need their
 * own tests.
 */
TEST_F(AttributesManagersTest, StageAttributesManagersCreate) {
  std::string stageConfigFile = Cr::Utility::Directory::join(
      DATA_DIR, "test_assets/scenes/simple_room.glb");

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "StageAttributesManager @ "
            << stageConfigFile;

  // scene attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::StageAttributesManager>(stageAttributesManager_,
                                                        stageConfigFile);
  testCreateAndRemoveDefault<AttrMgrs::StageAttributesManager>(
      stageAttributesManager_, stageConfigFile, true);

}  // AttributesManagersTest::StageAttributesManagersCreate

/**
 * @brief This test will test creating, modifying, registering and deleting
 * Attributes via Attributes Mangers for ObjectAttributes.  These
 * tests should be consistent with most types of future attributes managers
 * specializing the AttributesManager class template that follow the same
 * expected behavior paths as extent attributes/attributesManagers.  Note :
 * PrimitiveAssetAttributes exhibit slightly different behavior and need their
 * own tests.
 */
TEST_F(AttributesManagersTest, ObjectAttributesManagersCreate) {
  std::string objectConfigFile = Cr::Utility::Directory::join(
      DATA_DIR, "test_assets/objects/chair.object_config.json");

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "ObjectAttributesManager @ "
            << objectConfigFile;

  int origNumFileBased = objectAttributesManager_->getNumFileTemplateObjects();
  int origNumPrimBased = objectAttributesManager_->getNumSynthTemplateObjects();

  // object attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::ObjectAttributesManager>(
      objectAttributesManager_, objectConfigFile);
  // verify that no new file-based and no new synth based template objects
  // remain
  int newNumFileBased1 = objectAttributesManager_->getNumFileTemplateObjects();
  int newNumPrimBased1 = objectAttributesManager_->getNumSynthTemplateObjects();
  ASSERT_EQ(origNumFileBased, newNumFileBased1);
  ASSERT_EQ(origNumPrimBased, newNumPrimBased1);
  testCreateAndRemoveDefault<AttrMgrs::ObjectAttributesManager>(
      objectAttributesManager_, objectConfigFile, true);
  // verify that no new file-based and no new synth based template objects
  // remain
  int newNumFileBased2 = objectAttributesManager_->getNumFileTemplateObjects();
  int newNumPrimBased2 = objectAttributesManager_->getNumSynthTemplateObjects();
  ASSERT_EQ(origNumFileBased, newNumFileBased2);
  ASSERT_EQ(origNumPrimBased, newNumPrimBased2);

  // test adding many and removing all but defaults
  testRemoveAllButDefault<AttrMgrs::ObjectAttributesManager>(
      objectAttributesManager_, objectConfigFile, true);
  // verify that no new file-based and no new synth based template objects
  // remain
  int newNumFileBased3 = objectAttributesManager_->getNumFileTemplateObjects();
  int newNumPrimBased3 = objectAttributesManager_->getNumSynthTemplateObjects();
  ASSERT_EQ(origNumFileBased, newNumFileBased3);
  ASSERT_EQ(origNumPrimBased, newNumPrimBased3);
}  // AttributesManagersTest::ObjectAttributesManagersCreate test

TEST_F(AttributesManagersTest, LightLayoutAttributesManagerTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::LightLayoutAttributesManagerTest";

  std::string lightConfigFile = Cr::Utility::Directory::join(
      DATA_DIR, "test_assets/lights/test_lights.lighting_config.json");

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "LightLayoutAttributesManager @ "
            << lightConfigFile;
  // light attributes manager attributes verifcation
  testCreateAndRemoveLights(lightLayoutAttributesManager_, lightConfigFile);

}  // AttributesManagersTest::LightLayoutAttributesManagerTest

/**
 * @brief test primitive asset attributes functionality in attirbutes managers.
 * This includes testing handle auto-gen when relevant fields in asset
 * attributes are changed.
 */
TEST_F(AttributesManagersTest, PrimitiveAssetAttributesTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::PrimitiveAssetAttributesTest";
  /**
   * Primitive asset attributes require slightly different testing since a
   * default set of attributes (matching the default Magnum::Primitive
   * parameters) are created on program load and are always present.  User
   * modification of asset attributes always starts by modifying an existing
   * default template - users will never create an attributes template from
   * scratch.
   */
  int legalModValWF = 64;
  int illegalModValWF = 25;
  int legalModValSolid = 5;
  int illegalModValSolid = 0;

  //////////////////////////
  // get default template for solid capsule
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::CapsulePrimitiveAttributes";
    CapsulePrimitiveAttributes::ptr dfltCapsAttribs =
        assetAttributesManager_->getDefaultCapsuleTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<CapsulePrimitiveAttributes>(
        dfltCapsAttribs, "segments", legalModValSolid, &illegalModValSolid);

    // test wireframe version
    dfltCapsAttribs = assetAttributesManager_->getDefaultCapsuleTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<CapsulePrimitiveAttributes>(
        dfltCapsAttribs, "segments", legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid cone
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::ConePrimitiveAttributes";

    ConePrimitiveAttributes::ptr dfltConeAttribs =
        assetAttributesManager_->getDefaultConeTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<ConePrimitiveAttributes>(
        dfltConeAttribs, "segments", legalModValSolid, &illegalModValSolid);

    // test wireframe version
    dfltConeAttribs = assetAttributesManager_->getDefaultConeTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<ConePrimitiveAttributes>(
        dfltConeAttribs, "segments", legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid cylinder
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::CylinderPrimitiveAttributes";

    CylinderPrimitiveAttributes::ptr dfltCylAttribs =
        assetAttributesManager_->getDefaultCylinderTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<CylinderPrimitiveAttributes>(
        dfltCylAttribs, "segments", 5, &illegalModValSolid);

    // test wireframe version
    dfltCylAttribs = assetAttributesManager_->getDefaultCylinderTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<CylinderPrimitiveAttributes>(
        dfltCylAttribs, "segments", legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid UV Sphere
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::UVSpherePrimitiveAttributes";

    UVSpherePrimitiveAttributes::ptr dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<UVSpherePrimitiveAttributes>(
        dfltUVSphereAttribs, "segments", 5, &illegalModValSolid);

    // test wireframe version
    dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<UVSpherePrimitiveAttributes>(
        dfltUVSphereAttribs, "segments", legalModValWF, &illegalModValWF);
  }
}  // AttributesManagersTest::AsssetAttributesManagerGetAndModify test

}  // namespace
