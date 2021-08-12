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
      ESP_ERROR() << "testBuildAttributesFromJSONString : Failed to parse"
                  << jsonString << "as JSON.";
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
    // verify ID exists
    bool idIsPresent = mgr->getObjectLibHasID(attrTemplate1->getID());
    ASSERT_EQ(idIsPresent, true);
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
    // verify ID does not exist in library now
    idIsPresent = mgr->getObjectLibHasID(oldTemplate3->getID());
    ASSERT_EQ(idIsPresent, false);
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
   * @brief This method will test the user-defined configuration values to see
   * that they match with expected passed values.  The config is expected to
   * hold one of each type that it supports.
   * @param userConfig The configuration object whose contents are to be tested
   * @param str_val Expected string value
   * @param bool_val Expected boolean value
   * @param float_val Exptected float value
   * @param vec_val Expected Magnum::Vector3 value
   * @param quat_val Expected Quaternion value - note that the JSON is read with
   * scalar at idx 0, whereas the quaternion constructor takes the vector
   * component in the first position and the scalar in the second.
   */
  void testUserDefinedConfigVals(
      std::shared_ptr<esp::core::Configuration> userConfig,
      const std::string& str_val,
      bool bool_val,
      int int_val,
      float float_val,
      Magnum::Vector3 vec_val,
      Magnum::Quaternion quat_val) {
    // user defined attributes from light instance
    ASSERT_NE(nullptr, userConfig);
    ASSERT_EQ(userConfig->getString("user_string"), str_val);
    ASSERT_EQ(userConfig->getBool("user_bool"), bool_val);
    ASSERT_EQ(userConfig->getInt("user_int"), int_val);
    ASSERT_EQ(userConfig->getFloat("user_float"), float_val);
    ASSERT_EQ(userConfig->getVec3("user_vec3"), vec_val);
    ASSERT_EQ(userConfig->getQuat("user_quat"), quat_val);

  }  // AttributesManagersTest::testUserDefinedConfigVals

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
    ESP_DEBUG() << "Modified Template Handle :" << newHandle;
    // register modified template
    assetAttributesManager_->registerObject(defaultAttribs);

    // verify new handle is in template library
    CORRADE_INTERNAL_ASSERT(
        assetAttributesManager_->getObjectLibHasHandle(newHandle));
    // verify old template is still present as well
    CORRADE_INTERNAL_ASSERT(
        assetAttributesManager_->getObjectLibHasHandle(oldHandle));

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

  void testAssetAttributesTemplateCreateFromHandle(
      const std::string& newTemplateName) {
    // get starting number of templates
    int orignNumTemplates = assetAttributesManager_->getNumObjects();
    // first verify that no template with given name exists
    bool templateExists =
        assetAttributesManager_->getObjectLibHasHandle(newTemplateName);
    ASSERT_EQ(templateExists, false);
    // create new template based on handle and verify that it is created
    auto newTemplate = assetAttributesManager_->createTemplateFromHandle(
        newTemplateName, true);
    ASSERT_NE(newTemplate, nullptr);

    // now verify that template is in library
    templateExists =
        assetAttributesManager_->getObjectLibHasHandle(newTemplateName);
    ASSERT_EQ(templateExists, true);

    // remove new template via handle
    auto oldTemplate =
        assetAttributesManager_->removeObjectByHandle(newTemplateName);
    // verify deleted template  exists
    ASSERT_NE(nullptr, oldTemplate);

    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, assetAttributesManager_->getNumObjects());

  }  // AttributesManagersTest::testAssetAttributesTemplateCreateFromHandle

  esp::logging::LoggingContext loggingContext_;
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
  ESP_DEBUG() << "Starting AttributesManagers_PhysicsJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
  "physics_simulator": "bullet_test",
  "timestep": 1.0,
  "gravity": [1,2,3],
  "friction_coefficient": 1.4,
  "restitution_coefficient": 1.1,
  "user_defined" : {
      "user_string" : "pm defined string",
      "user_bool" : true,
      "user_int" : 15,
      "user_float" : 12.6,
      "user_vec3" : [215.4, 217.6, 2110.1],
      "user_quat" : [0.2, 5.2, 6.2, 7.2]
  }
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
  // test physics manager attributes-level user config vals
  testUserDefinedConfigVals(physMgrAttr->getUserConfiguration(),
                            "pm defined string", true, 15, 12.6f,
                            Magnum::Vector3(215.4, 217.6, 2110.1),
                            Magnum::Quaternion({5.2f, 6.2f, 7.2f}, 0.2f));

}  // AttributesManagers_PhysicsJSONLoadTest

/**
 * @brief This test will verify that the Light Attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_LightJSONLoadTest) {
  ESP_DEBUG() << "Starting AttributesManagers_LightJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
  "lights":{
      "test":{
        "position": [2.5,0.1,3.8],
        "direction": [1.0,-1.0,1.0],
        "intensity": -0.1,
        "color": [2,1,-1],
        "type": "directional",
        "position_model" : "camera",
        "spot": {
          "innerConeAngle": -0.75,
          "outerConeAngle": -1.57
        },
        "user_defined" : {
            "user_string" : "light instance defined string",
            "user_bool" : false,
            "user_int" : 42,
            "user_float" : 1.2,
            "user_vec3" : [0.1, 2.3, 4.5],
            "user_quat" : [0.1, 0.2, 0.3, 0.4]
        }
      }
    },
    "user_defined" : {
        "user_string" : "light attribs defined string",
        "user_bool" : true,
        "user_int" : 23,
        "user_float" : 2.3,
        "user_vec3" : [1.1, 3.3, 5.5],
        "user_quat" : [0.5, 0.6, 0.7, 0.8]
    },
    "positive_intensity_scale" : 2.0,
    "negative_intensity_scale" : 1.5
  })";

  auto lightLayoutAttr =
      testBuildAttributesFromJSONString<AttrMgrs::LightLayoutAttributesManager,
                                        Attrs::LightLayoutAttributes>(
          lightLayoutAttributesManager_, jsonString);

  // verify exists
  ASSERT_NE(nullptr, lightLayoutAttr);
  // test light layout attributes-level user config vals
  testUserDefinedConfigVals(lightLayoutAttr->getUserConfiguration(),
                            "light attribs defined string", true, 23, 2.3f,
                            Magnum::Vector3(1.1, 3.3, 5.5),
                            Magnum::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f));
  ASSERT_EQ(lightLayoutAttr->getPositiveIntensityScale(), 2.0);
  ASSERT_EQ(lightLayoutAttr->getNegativeIntensityScale(), 1.5);
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
  ASSERT_EQ(lightAttr->getPositionModel(),
            static_cast<int>(esp::gfx::LightPositionModel::Camera));
  ASSERT_EQ(lightAttr->getInnerConeAngle(), -0.75_radf);
  ASSERT_EQ(lightAttr->getOuterConeAngle(), -1.57_radf);

  // test user defined attributes from light instance
  testUserDefinedConfigVals(lightAttr->getUserConfiguration(),
                            "light instance defined string", false, 42, 1.2f,
                            Magnum::Vector3(0.1, 2.3, 4.5),
                            Magnum::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f));

}  // AttributesManagers_LightJSONLoadTest

/**
 * @brief This test will verify that the Scene Instance attributes' managers'
 * JSON loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_SceneInstanceJSONLoadTest) {
  ESP_DEBUG() << "Starting AttributesManagers_SceneInstanceJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
  "translation_origin" : "Asset_Local",
  "stage_instance":{
      "template_name": "test_stage_template",
      "translation": [1,2,3],
      "rotation": [0.1, 0.2, 0.3, 0.4],
      "shader_type" : "pbr",
      "user_defined" : {
          "user_string" : "stage instance defined string",
          "user_bool" : true,
          "user_int" : 11,
          "user_float" : 2.2,
          "user_vec3" : [1.2, 3.4, 5.6],
          "user_quat" : [0.4, 0.5, 0.6, 0.7]
      }
  },
  "object_instances": [
      {
          "template_name": "test_object_template0",
          "translation_origin": "COM",
          "translation": [0,1,2],
          "rotation": [0.2, 0.3, 0.4, 0.5],
          "motion_type": "KINEMATIC",
          "user_defined" : {
              "user_string" : "obj0 instance defined string",
              "user_bool" : false,
              "user_int" : 12,
              "user_float" : 2.3,
              "user_vec3" : [1.3, 3.5, 5.7],
              "user_quat" : [0.3, 0.2, 0.6, 0.1]
          }
      },
      {
          "template_name": "test_object_template1",
          "translation": [0,-1,-2],
          "rotation": [0.5, 0.6, 0.7, 0.8],
          "motion_type": "DYNAMIC",
          "user_defined" : {
              "user_string" : "obj1 instance defined string",
              "user_bool" : false,
              "user_int" : 1,
              "user_float" : 1.1,
              "user_vec3" : [10.3, 30.5, -5.07],
              "user_quat" : [1.3, 1.2, 1.6, 1.1]
          }
      }
      ],
      "articulated_object_instances": [
          {
              "template_name": "test_urdf_template0",
              "translation_origin": "COM",
              "fixed_base": false,
              "auto_clamp_joint_limits" : true,
              "translation": [5,4,5],
              "rotation": [0.2, 0.3, 0.4, 0.5],
              "initial_joint_pose": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
              "initial_joint_velocities": [1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6],
              "motion_type": "DYNAMIC",
              "user_defined" : {
                  "user_string" : "test_urdf_template0 instance defined string",
                  "user_bool" : false,
                  "user_int" : 2,
                  "user_float" : 1.22,
                  "user_vec3" : [120.3, 302.5, -25.07],
                  "user_quat" : [1.23, 1.22, 1.26, 1.21],
                  "user_def_obj" : {
                      "position" : [0.1, 0.2, 0.3],
                      "rotation" : [0.5, 0.3, 0.1]
                  }
              }
          },
          {
              "template_name": "test_urdf_template1",
              "fixed_base" : true,
              "auto_clamp_joint_limits" : true,
              "translation": [3, 2, 1],
              "rotation": [0.5, 0.6, 0.7, 0.8],
              "motion_type": "KINEMATIC",
              "user_defined" : {
                  "user_string" : "test_urdf_template1 instance defined string",
                  "user_bool" : false,
                  "user_int" : 21,
                  "user_float" : 11.22,
                  "user_vec3" : [190.3, 902.5, -95.07],
                  "user_quat" : [1.25, 9.22, 9.26, 0.21]
              }
          }
      ],
      "default_lighting":  "test_lighting_configuration",
      "navmesh_instance": "test_navmesh_path1",
      "semantic_scene_instance": "test_semantic_descriptor_path1",
      "user_defined" : {
          "user_string" : "scene instance defined string",
          "user_bool" : true,
          "user_int" : 99,
          "user_float" : 9.1,
          "user_vec3" : [12.3, 32.5, 25.07],
          "user_quat" : [0.3, 3.2, 2.6, 5.1]
      }
     })";

  auto sceneAttr =
      testBuildAttributesFromJSONString<AttrMgrs::SceneAttributesManager,
                                        Attrs::SceneAttributes>(
          sceneAttributesManager_, jsonString);

  // verify exists
  ASSERT_NE(nullptr, sceneAttr);

  // match values set in test JSON
  ASSERT_EQ(
      sceneAttr->getTranslationOrigin(),
      static_cast<int>(Attrs::SceneInstanceTranslationOrigin::AssetLocal));
  ASSERT_EQ(sceneAttr->getLightingHandle(), "test_lighting_configuration");
  ASSERT_EQ(sceneAttr->getNavmeshHandle(), "test_navmesh_path1");
  ASSERT_EQ(sceneAttr->getSemanticSceneHandle(),
            "test_semantic_descriptor_path1");
  // test scene instance attributes-level user config vals
  testUserDefinedConfigVals(sceneAttr->getUserConfiguration(),
                            "scene instance defined string", true, 99, 9.1f,
                            Magnum::Vector3(12.3, 32.5, 25.07),
                            Magnum::Quaternion({3.2f, 2.6f, 5.1f}, 0.3f));

  // verify stage populated properly
  auto stageInstance = sceneAttr->getStageInstance();
  ASSERT_EQ(stageInstance->getHandle(), "test_stage_template");
  ASSERT_EQ(stageInstance->getTranslation(), Magnum::Vector3(1, 2, 3));
  ASSERT_EQ(stageInstance->getRotation(),
            Magnum::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f));
  // test stage instance attributes-level user config vals
  testUserDefinedConfigVals(stageInstance->getUserConfiguration(),
                            "stage instance defined string", true, 11, 2.2f,
                            Magnum::Vector3(1.2, 3.4, 5.6),
                            Magnum::Quaternion({0.5f, 0.6f, 0.7f}, 0.4f));
  // make sure that is not default value "flat"
  ASSERT_EQ(stageInstance->getShaderType(),
            static_cast<int>(Attrs::ObjectInstanceShaderType::PBR));

  // verify objects
  auto objectInstanceList = sceneAttr->getObjectInstances();
  ASSERT_EQ(objectInstanceList.size(), 2);
  auto objInstance = objectInstanceList[0];
  ASSERT_EQ(objInstance->getHandle(), "test_object_template0");
  ASSERT_EQ(objInstance->getTranslationOrigin(),
            static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
  ASSERT_EQ(objInstance->getTranslation(), Magnum::Vector3(0, 1, 2));
  ASSERT_EQ(objInstance->getRotation(),
            Magnum::Quaternion({0.3f, 0.4f, 0.5f}, 0.2f));
  ASSERT_EQ(objInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::KINEMATIC));

  // test object 0 instance attributes-level user config vals
  testUserDefinedConfigVals(objInstance->getUserConfiguration(),
                            "obj0 instance defined string", false, 12, 2.3f,
                            Magnum::Vector3(1.3, 3.5, 5.7),
                            Magnum::Quaternion({0.2f, 0.6f, 0.1f}, 0.3f));

  objInstance = objectInstanceList[1];
  ASSERT_EQ(objInstance->getHandle(), "test_object_template1");
  ASSERT_EQ(objInstance->getTranslation(), Magnum::Vector3(0, -1, -2));
  ASSERT_EQ(objInstance->getRotation(),
            Magnum::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f));
  ASSERT_EQ(objInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::DYNAMIC));

  // test object 0 instance attributes-level user config vals
  testUserDefinedConfigVals(objInstance->getUserConfiguration(),
                            "obj1 instance defined string", false, 1, 1.1f,
                            Magnum::Vector3(10.3, 30.5, -5.07),
                            Magnum::Quaternion({1.2f, 1.6f, 1.1f}, 1.3f));

  // verify articulated object instances
  auto artObjInstances = sceneAttr->getArticulatedObjectInstances();
  ASSERT_EQ(artObjInstances.size(), 2);
  auto artObjInstance = artObjInstances[0];
  ASSERT_EQ(artObjInstance->getHandle(), "test_urdf_template0");
  ASSERT_EQ(artObjInstance->getTranslationOrigin(),
            static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
  ASSERT_EQ(artObjInstance->getFixedBase(), false);
  ASSERT_EQ(artObjInstance->getAutoClampJointLimits(), true);

  ASSERT_EQ(artObjInstance->getTranslation(), Magnum::Vector3(5, 4, 5));
  ASSERT_EQ(artObjInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::DYNAMIC));
  // verify init join pose
  const auto& initJointPoseMap = artObjInstance->getInitJointPose();
  const std::vector<float> jtPoseVals{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  int idx = 0;
  for (std::map<std::string, float>::const_iterator iter =
           initJointPoseMap.begin();
       iter != initJointPoseMap.end(); ++iter) {
    ASSERT_EQ(iter->second, jtPoseVals[idx++]);
  }
  // verify init joint vels
  const auto& initJoinVelMap = artObjInstance->getInitJointVelocities();
  const std::vector<float> jtVelVals{1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6};
  idx = 0;
  for (std::map<std::string, float>::const_iterator iter =
           initJoinVelMap.begin();
       iter != initJoinVelMap.end(); ++iter) {
    ASSERT_EQ(iter->second, jtVelVals[idx++]);
  }

  // test test_urdf_template0 ao instance attributes-level user config vals
  testUserDefinedConfigVals(artObjInstance->getUserConfiguration(),
                            "test_urdf_template0 instance defined string",
                            false, 2, 1.22f,
                            Magnum::Vector3(120.3f, 302.5f, -25.07f),
                            Magnum::Quaternion({1.22f, 1.26f, 1.21f}, 1.23f));

  // test nested configuration
  auto artObjNestedConfig =
      artObjInstance->getUserConfiguration()->getConfigSubgroupAsPtr(
          "user_def_obj");
  ASSERT_NE(artObjNestedConfig, nullptr);
  ASSERT_EQ(artObjNestedConfig->hasValues(), true);
  ASSERT_EQ(artObjNestedConfig->getVec3("position"),
            Magnum::Vector3(0.1f, 0.2f, 0.3f));
  ASSERT_EQ(artObjNestedConfig->getVec3("rotation"),
            Magnum::Vector3(0.5f, 0.3f, 0.1f));
  ESP_WARNING() << "Articulated Object test 3";

  artObjInstance = artObjInstances[1];
  ASSERT_EQ(artObjInstance->getHandle(), "test_urdf_template1");
  ASSERT_EQ(artObjInstance->getFixedBase(), true);
  ASSERT_EQ(artObjInstance->getAutoClampJointLimits(), true);
  ASSERT_EQ(artObjInstance->getTranslation(), Magnum::Vector3(3, 2, 1));
  ASSERT_EQ(artObjInstance->getMotionType(),
            static_cast<int>(esp::physics::MotionType::KINEMATIC));
  // test test_urdf_template0 ao instance attributes-level user config vals
  testUserDefinedConfigVals(artObjInstance->getUserConfiguration(),
                            "test_urdf_template1 instance defined string",
                            false, 21, 11.22f,
                            Magnum::Vector3(190.3f, 902.5f, -95.07f),
                            Magnum::Quaternion({9.22f, 9.26f, 0.21f}, 1.25f));

}  // AttributesManagers_SceneInstanceJSONLoadTest

/**
 * @brief This test will verify that the Stage attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_StageJSONLoadTest) {
  ESP_DEBUG() << "Starting AttributesManagers_StageJSONLoadTest";

  // build JSON sample config
  const std::string& jsonString =
      R"({
        "scale":[2,3,4],
        "margin": 0.9,
        "friction_coefficient": 0.321,
        "restitution_coefficient": 0.456,
        "requires_lighting": true,
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
        "house_filename":"testJSONHouseFileName.glb",
        "shader_type" : "material",
        "user_defined" : {
            "user_string" : "stage defined string",
            "user_bool" : false,
            "user_int" : 3,
            "user_float" : 0.8,
            "user_vec3" : [5.4, 7.6, 10.1],
            "user_quat" : [0.1, 1.5, 2.6, 3.7]
        }
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
  ASSERT_EQ(stageAttr->getRequiresLighting(), true);
  ASSERT_EQ(stageAttr->getUnitsToMeters(), 1.1);
  ASSERT_EQ(stageAttr->getOrientUp(), Magnum::Vector3(2.1, 0, 0));
  ASSERT_EQ(stageAttr->getOrientFront(), Magnum::Vector3(0, 2.1, 0));
  ASSERT_EQ(stageAttr->getRenderAssetHandle(), "testJSONRenderAsset.glb");
  ASSERT_EQ(stageAttr->getCollisionAssetHandle(), "testJSONCollisionAsset.glb");
  ASSERT_EQ(stageAttr->getIsCollidable(), false);
  // stage-specific attributes
  ASSERT_EQ(stageAttr->getGravity(), Magnum::Vector3(9, 8, 7));
  // make sure that is not default value "flat"
  ASSERT_EQ(stageAttr->getShaderType(),
            static_cast<int>(Attrs::ObjectInstanceShaderType::Material));
  ASSERT_EQ(stageAttr->getOrigin(), Magnum::Vector3(1, 2, 3));
  ASSERT_EQ(stageAttr->getSemanticAssetHandle(), "testJSONSemanticAsset.glb");
  ASSERT_EQ(stageAttr->getNavmeshAssetHandle(), "testJSONNavMeshAsset.glb");
  ASSERT_EQ(stageAttr->getHouseFilename(), "testJSONHouseFileName.glb");
  // test stage attributes-level user config vals
  testUserDefinedConfigVals(stageAttr->getUserConfiguration(),
                            "stage defined string", false, 3, 0.8f,
                            Magnum::Vector3(5.4, 7.6, 10.1),
                            Magnum::Quaternion({1.5f, 2.6f, 3.7f}, 0.1f));

}  // AttributesManagers_StageJSONLoadTest

/**
 * @brief This test will verify that the Object attributes' managers' JSON
 * loading process is working as expected.
 */
TEST_F(AttributesManagersTest, AttributesManagers_ObjectJSONLoadTest) {
  ESP_DEBUG() << "Starting AttributesManagers_ObjectJSONLoadTest";
  // build JSON sample config
  const std::string& jsonString = R"({
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
  "COM": [0.1,0.2,0.3],
  "shader_type" : "phong",
  "user_defined" : {
      "user_string" : "object defined string",
      "user_bool" : true,
      "user_int" : 5,
      "user_float" : 2.6,
      "user_vec3" : [15.4, 17.6, 110.1],
      "user_quat" : [0.7, 5.5, 6.6, 7.7]
  }
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
  ASSERT_EQ(objAttr->getShaderType(),
            static_cast<int>(Attrs::ObjectInstanceShaderType::Phong));
  ASSERT_EQ(objAttr->getBoundingBoxCollisions(), true);
  ASSERT_EQ(objAttr->getJoinCollisionMeshes(), true);
  ASSERT_EQ(objAttr->getInertia(), Magnum::Vector3(1.1, 0.9, 0.3));
  ASSERT_EQ(objAttr->getCOM(), Magnum::Vector3(0.1, 0.2, 0.3));
  // test object attributes-level user config vals
  testUserDefinedConfigVals(objAttr->getUserConfiguration(),
                            "object defined string", true, 5, 2.6f,
                            Magnum::Vector3(15.4, 17.6, 110.1),
                            Magnum::Quaternion({5.5f, 6.6f, 7.7f}, 0.7f));

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
  ESP_DEBUG() << "Starting PhysicsAttributesManagersCreate";

  ESP_DEBUG() << "Start Test : Create, Edit, Remove Attributes for "
                 "PhysicsAttributesManager @"
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

  ESP_DEBUG() << "Start Test : Create, Edit, Remove Attributes for "
                 "StageAttributesManager @"
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

  ESP_DEBUG() << "Start Test : Create, Edit, Remove Attributes for "
                 "ObjectAttributesManager @"
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
  ESP_DEBUG() << "Starting LightLayoutAttributesManagerTest";

  std::string lightConfigFile = Cr::Utility::Directory::join(
      DATA_DIR, "test_assets/lights/test_lights.lighting_config.json");

  ESP_DEBUG() << "Start Test : Create, Edit, Remove Attributes for "
                 "LightLayoutAttributesManager @"
              << lightConfigFile;
  // light attributes manager attributes verifcation
  testCreateAndRemoveLights(lightLayoutAttributesManager_, lightConfigFile);

}  // AttributesManagersTest::LightLayoutAttributesManagerTest

/**
 * @brief test primitive asset attributes functionality in attirbutes
 * managers. This includes testing handle auto-gen when relevant fields in
 * asset attributes are changed.
 */
TEST_F(AttributesManagersTest, PrimitiveAssetAttributesTest) {
  ESP_DEBUG() << "Starting PrimitiveAssetAttributesTest";
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

  const std::string capsule3DSolidHandle =
      "capsule3DSolid_hemiRings_5_cylRings_2_segments_16_halfLen_1.75_"
      "useTexCoords_true_useTangents_true";
  const std::string capsule3DWireframeHandle =
      "capsule3DWireframe_hemiRings_8_cylRings_2_segments_20_halfLen_1.5";

  const std::string coneSolidHandle =
      "coneSolid_segments_12_halfLen_1.35_rings_1_useTexCoords_true_"
      "useTangents_true_capEnd_true";
  const std::string coneWireframeHandle =
      "coneWireframe_segments_32_halfLen_1.44";

  const std::string cylinderSolidHandle =
      "cylinderSolid_rings_1_segments_28_halfLen_1.11_useTexCoords_true_"
      "useTangents_true_capEnds_true";
  const std::string cylinderWireframeHandle =
      "cylinderWireframe_rings_1_segments_32_halfLen_1.23";

  const std::string uvSphereSolidHandle =
      "uvSphereSolid_rings_16_segments_8_useTexCoords_true_useTangents_true";
  const std::string uvSphereWireframeHandle =
      "uvSphereWireframe_rings_20_segments_24";

  //////////////////////////
  // get default template for solid capsule
  {
    ESP_DEBUG() << "Starting CapsulePrimitiveAttributes";
    CapsulePrimitiveAttributes::ptr dfltCapsAttribs =
        assetAttributesManager_->getDefaultCapsuleTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<CapsulePrimitiveAttributes>(
        dfltCapsAttribs, "segments", legalModValSolid, &illegalModValSolid);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(capsule3DSolidHandle);

    // test wireframe version
    dfltCapsAttribs = assetAttributesManager_->getDefaultCapsuleTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<CapsulePrimitiveAttributes>(
        dfltCapsAttribs, "segments", legalModValWF, &illegalModValWF);
    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(capsule3DWireframeHandle);
  }
  //////////////////////////
  // get default template for solid cone
  {
    ESP_DEBUG() << "Starting ConePrimitiveAttributes";

    ConePrimitiveAttributes::ptr dfltConeAttribs =
        assetAttributesManager_->getDefaultConeTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<ConePrimitiveAttributes>(
        dfltConeAttribs, "segments", legalModValSolid, &illegalModValSolid);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(coneSolidHandle);

    // test wireframe version
    dfltConeAttribs = assetAttributesManager_->getDefaultConeTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<ConePrimitiveAttributes>(
        dfltConeAttribs, "segments", legalModValWF, &illegalModValWF);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(coneWireframeHandle);
  }
  //////////////////////////
  // get default template for solid cylinder
  {
    ESP_DEBUG() << "Starting CylinderPrimitiveAttributes";

    CylinderPrimitiveAttributes::ptr dfltCylAttribs =
        assetAttributesManager_->getDefaultCylinderTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<CylinderPrimitiveAttributes>(
        dfltCylAttribs, "segments", 5, &illegalModValSolid);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(cylinderSolidHandle);

    // test wireframe version
    dfltCylAttribs = assetAttributesManager_->getDefaultCylinderTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<CylinderPrimitiveAttributes>(
        dfltCylAttribs, "segments", legalModValWF, &illegalModValWF);
    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(cylinderWireframeHandle);
  }
  //////////////////////////
  // get default template for solid UV Sphere
  {
    ESP_DEBUG() << "Starting UVSpherePrimitiveAttributes";

    UVSpherePrimitiveAttributes::ptr dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<UVSpherePrimitiveAttributes>(
        dfltUVSphereAttribs, "segments", 5, &illegalModValSolid);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(uvSphereSolidHandle);

    // test wireframe version
    dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<UVSpherePrimitiveAttributes>(
        dfltUVSphereAttribs, "segments", legalModValWF, &illegalModValWF);

    // test that a new template can be created from the specified handles
    testAssetAttributesTemplateCreateFromHandle(uvSphereWireframeHandle);
  }
}  // AttributesManagersTest::AsssetAttributesManagerGetAndModify test

}  // namespace
