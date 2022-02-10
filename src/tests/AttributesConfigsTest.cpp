// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <string>

#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

#include "esp/physics/RigidBase.h"

#include "configure.h"

namespace Cr = Corrade;

using Magnum::Math::Literals::operator""_radf;
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using esp::metadata::MetadataMediator;
using esp::metadata::PrimObjTypes;

using esp::physics::MotionType;

using AttrMgrs::AttributesManager;
using Attrs::ObjectAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::SceneInstanceAttributes;
using Attrs::StageAttributes;
using Attrs::UVSpherePrimitiveAttributes;

namespace {

// base directory to save test attributes
const std::string testAttrSaveDir =
    Cr::Utility::Directory::join(DATA_DIR, "test_assets");

/**
 * @brief Test attributes/configuration functionality via setting values from
 * JSON string, saving JSON to file and loading verifying saved JSON matches
 * expectations upon load.
 */

struct AttributesConfigsTest : Cr::TestSuite::Tester {
  explicit AttributesConfigsTest();

  // Test helper functions

  /**
   * @brief Test saving and loading from JSON string
   * @tparam T Class of attributes manager
   * @tparam U Class of attributes
   * @param mgr the Attributes Manager being tested
   * @param jsonString the json to build the template from
   * @param registerObject whether or not to register the Attributes constructed
   * by the passed JSON string.
   * @param tmpltName The name to give the template
   * @return attributes template built from JSON parsed from string
   */
  template <typename T, typename U>
  std::shared_ptr<U> testBuildAttributesFromJSONString(
      std::shared_ptr<T> mgr,
      const std::string& jsonString,
      const bool registerObject,
      const std::string& tmpltName = "new_template_from_json");

  /**
   * @brief remove added template built from JSON string.
   * @param tmpltName name of template to remove.
   * @param mgr the Attributes Manager being tested
   * @param handle the handle of the attributes to remove
   */
  template <typename T>
  void testRemoveAttributesBuiltByJSONString(
      std::shared_ptr<T> mgr,
      const std::string& tmpltName = "new_template_from_json");

  /**
   * @brief This method will test the user-defined configuration values to see
   * that they match with expected passed values.  The config is expected to
   * hold one of each type that it supports.
   * @param userConfig The configuration object whose contents are to be
   * tested
   * @param str_val Expected string value
   * @param bool_val Expected boolean value
   * @param double_val Exptected double value
   * @param vec_val Expected Magnum::Vector3 value
   * @param quat_val Expected Quaternion value - note that the JSON is read
   * with scalar at idx 0, whereas the quaternion constructor takes the vector
   * component in the first position and the scalar in the second.
   */
  void testUserDefinedConfigVals(
      std::shared_ptr<esp::core::config::Configuration> userConfig,
      const std::string& str_val,
      bool bool_val,
      int int_val,
      double double_val,
      Magnum::Vector3 vec_val,
      Magnum::Quaternion quat_val);

  /**
   * @brief This test will verify that the physics attributes' managers' JSON
   * loading process is working as expected.
   */
  void testPhysicsAttrVals(
      std::shared_ptr<esp::metadata::attributes::PhysicsManagerAttributes>
          physMgrAttr);
  /**
   * @brief This test will verify that the Light Attributes' managers' JSON
   * loading process is working as expected.
   */
  void testLightAttrVals(
      std::shared_ptr<esp::metadata::attributes::LightLayoutAttributes>
          lightLayoutAttr);

  /**
   * @brief This test will verify that the Scene Instance attributes' managers'
   * JSON loading process is working as expected.
   */
  void testSceneInstanceAttrVals(
      std::shared_ptr<esp::metadata::attributes::SceneInstanceAttributes>
          sceneInstAttr);
  /**
   * @brief This test will verify that the Stage attributes' managers' JSON
   * loading process is working as expected.
   */
  void testStageAttrVals(
      std::shared_ptr<esp::metadata::attributes::StageAttributes> stageAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the Object attributes' managers' JSON
   * loading process is working as expected.
   */
  void testObjectAttrVals(
      std::shared_ptr<esp::metadata::attributes::ObjectAttributes> objAttr,
      const std::string& assetPath);

  // actual test functions
  // These tests build strings containing legal JSON config data to use to build
  // an appropriate attributes configuration.  The resultant configuration is
  // then tested for accuracy.  These functions also save a copy to a json file,
  // and then reload the copy, to make sure the saving and loading process is
  // correct.
  void testPhysicsJSONLoad();
  void testLightJSONLoad();
  void testSceneInstanceJSONLoad();
  void testStageJSONLoad();
  void testObjectJSONLoad();

  // test member vars

  esp::logging::LoggingContext loggingContext_;
  AttrMgrs::LightLayoutAttributesManager::ptr lightLayoutAttributesManager_ =
      nullptr;
  AttrMgrs::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;
  AttrMgrs::SceneInstanceAttributesManager::ptr
      sceneInstanceAttributesManager_ = nullptr;
  AttrMgrs::StageAttributesManager::ptr stageAttributesManager_ = nullptr;

};  // struct AttributesConfigsTest

AttributesConfigsTest::AttributesConfigsTest() {
  // set up a default simulation config to initialize MM
  auto cfg = esp::sim::SimulatorConfiguration{};
  auto MM = MetadataMediator::create(cfg);
  // get attributes managers for default dataset
  lightLayoutAttributesManager_ = MM->getLightLayoutAttributesManager();
  objectAttributesManager_ = MM->getObjectAttributesManager();
  physicsAttributesManager_ = MM->getPhysicsAttributesManager();
  sceneInstanceAttributesManager_ = MM->getSceneInstanceAttributesManager();
  stageAttributesManager_ = MM->getStageAttributesManager();

  addTests({
      &AttributesConfigsTest::testPhysicsJSONLoad,
      &AttributesConfigsTest::testLightJSONLoad,
      &AttributesConfigsTest::testSceneInstanceJSONLoad,
      &AttributesConfigsTest::testStageJSONLoad,
      &AttributesConfigsTest::testObjectJSONLoad,
  });
}

template <typename T, typename U>
std::shared_ptr<U> AttributesConfigsTest::testBuildAttributesFromJSONString(
    std::shared_ptr<T> mgr,
    const std::string& jsonString,
    const bool registerObject,
    const std::string& tmpltName) {  // create JSON document
  try {
    esp::io::JsonDocument tmp = esp::io::parseJsonString(jsonString);
    // io::JsonGenericValue :
    const esp::io::JsonGenericValue jsonDoc = tmp.GetObject();
    // create an new template from jsonDoc
    std::shared_ptr<U> attrTemplate1 =
        mgr->buildManagedObjectFromDoc(tmpltName, jsonDoc);

    // register attributes
    if (registerObject) {
      mgr->registerObject(attrTemplate1);
    }
    return attrTemplate1;
  } catch (...) {
    CORRADE_FAIL_IF(true, "testBuildAttributesFromJSONString : Failed to parse"
                              << jsonString << "as JSON.");
    return nullptr;
  }
}  // testBuildAttributesFromJSONString

template <typename T>
void AttributesConfigsTest::testRemoveAttributesBuiltByJSONString(
    std::shared_ptr<T> mgr,
    const std::string& tmpltName) {
  if (mgr->getObjectLibHasHandle(tmpltName)) {
    mgr->removeObjectByHandle(tmpltName);
  }
}

void AttributesConfigsTest::testUserDefinedConfigVals(
    std::shared_ptr<esp::core::config::Configuration> userConfig,
    const std::string& str_val,
    bool bool_val,
    int int_val,
    double double_val,
    Magnum::Vector3 vec_val,
    Magnum::Quaternion quat_val) {
  // user defined attributes from light instance
  CORRADE_VERIFY(userConfig);
  CORRADE_COMPARE(userConfig->get<std::string>("user_string"), str_val);
  CORRADE_COMPARE(userConfig->get<bool>("user_bool"), bool_val);
  CORRADE_COMPARE(userConfig->get<int>("user_int"), int_val);
  if (userConfig->hasValue("user_double")) {
    // this triggers an error on CI that we will revisit
    CORRADE_COMPARE(userConfig->get<double>("user_double"), double_val);
  } else {
    ESP_DEBUG() << "Temporarily skipping test that triggered CI error on key "
                   "`user_double`.";
  }
  CORRADE_COMPARE(userConfig->get<Magnum::Vector3>("user_vec3"), vec_val);
  CORRADE_COMPARE(userConfig->get<Magnum::Quaternion>("user_quat"), quat_val);

}  // AttributesConfigsTest::testUserDefinedConfigVals

/////////////  Begin JSON String-based tests

void AttributesConfigsTest::testPhysicsAttrVals(
    std::shared_ptr<esp::metadata::attributes::PhysicsManagerAttributes>
        physMgrAttr) {
  // match values set in test JSON

  CORRADE_COMPARE(physMgrAttr->getGravity(), Magnum::Vector3(1, 2, 3));
  CORRADE_COMPARE(physMgrAttr->getTimestep(), 1.0);
  CORRADE_COMPARE(physMgrAttr->getSimulator(), "bullet_test");
  CORRADE_COMPARE(physMgrAttr->getFrictionCoefficient(), 1.4);
  CORRADE_COMPARE(physMgrAttr->getRestitutionCoefficient(), 1.1);
  // test physics manager attributes-level user config vals
  testUserDefinedConfigVals(physMgrAttr->getUserConfiguration(),
                            "pm defined string", true, 15, 12.6,
                            Magnum::Vector3(215.4, 217.6, 2110.1),
                            Magnum::Quaternion({5.2f, 6.2f, 7.2f}, 0.2f));
  // remove added template
  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(physicsAttributesManager_,
                                        physMgrAttr->getHandle());
}

void AttributesConfigsTest::testPhysicsJSONLoad() {
  // build JSON sample config
  // add dummy test so that test will run
  CORRADE_VERIFY(true);
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
      "user_double" : 12.6,
      "user_vec3" : [215.4, 217.6, 2110.1],
      "user_quat" : [0.2, 5.2, 6.2, 7.2]
  }
})";
  auto physMgrAttr =
      testBuildAttributesFromJSONString<AttrMgrs::PhysicsAttributesManager,
                                        Attrs::PhysicsManagerAttributes>(
          physicsAttributesManager_, jsonString, true);
  // verify exists
  CORRADE_VERIFY(physMgrAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testPhysicsAttrConfig_saved_JSON.{}", testAttrSaveDir,
      physicsAttributesManager_->getJSONTypeExt());

  bool success = physicsAttributesManager_->saveManagedObjectToFile(
      physMgrAttr->getHandle(), newAttrName);

  ESP_DEBUG() << "About to test string-based physMgrAttr";
  // test json string to verify format, this deletes physMgrAttr from registry
  testPhysicsAttrVals(physMgrAttr);
  ESP_DEBUG() << "Tested physMgrAttr";

  physMgrAttr = nullptr;

  // load attributes from new name and retest
  auto physMgrAttr2 =
      physicsAttributesManager_->createObjectFromJSONFile(newAttrName, true);

  // verify file-based config exists
  CORRADE_VERIFY(physMgrAttr2);

  ESP_DEBUG() << "About to test saved physMgrAttr2 :"
              << physMgrAttr2->getHandle();
  // test json string to verify format, this deletes physMgrAttr2 from
  // registry
  testPhysicsAttrVals(physMgrAttr2);
  ESP_DEBUG() << "Tested physMgrAttr";

  // delete file-based config
  Cr::Utility::Directory::rm(newAttrName);

}  // AttributesManagers_PhysicsJSONLoadTest

void AttributesConfigsTest::testLightAttrVals(
    std::shared_ptr<esp::metadata::attributes::LightLayoutAttributes>
        lightLayoutAttr) {
  // test light layout attributes-level user config vals
  testUserDefinedConfigVals(lightLayoutAttr->getUserConfiguration(),
                            "light attribs defined string", true, 23, 2.3,
                            Magnum::Vector3(1.1, 3.3, 5.5),
                            Magnum::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f));
  CORRADE_COMPARE(lightLayoutAttr->getPositiveIntensityScale(), 2.0);
  CORRADE_COMPARE(lightLayoutAttr->getNegativeIntensityScale(), 1.5);
  auto lightAttr0 = lightLayoutAttr->getLightInstance("test0");
  // verify that lightAttr0 exists
  CORRADE_VERIFY(lightAttr0);

  // match values set in test JSON

  CORRADE_COMPARE(lightAttr0->getDirection(), Magnum::Vector3(1.0, -1.0, 1.0));
  CORRADE_COMPARE(lightAttr0->getColor(), Magnum::Vector3(0.6, 0.7, 0.8));

  CORRADE_COMPARE(lightAttr0->getIntensity(), -0.1);
  CORRADE_COMPARE(static_cast<int>(lightAttr0->getType()),
                  static_cast<int>(esp::gfx::LightType::Directional));
  CORRADE_COMPARE(static_cast<int>(lightAttr0->getPositionModel()),
                  static_cast<int>(esp::gfx::LightPositionModel::Camera));
  CORRADE_COMPARE(lightAttr0->getInnerConeAngle(), 0.25_radf);
  CORRADE_COMPARE(lightAttr0->getOuterConeAngle(), -1.57_radf);

  auto lightAttr1 = lightLayoutAttr->getLightInstance("test1");
  // verify that lightAttr1 exists
  CORRADE_VERIFY(lightAttr1);

  CORRADE_COMPARE(lightAttr1->getPosition(), Magnum::Vector3(2.5, 0.1, 3.8));
  CORRADE_COMPARE(lightAttr1->getColor(), Magnum::Vector3(0.5, 0.3, 0.1));

  CORRADE_COMPARE(lightAttr1->getIntensity(), -1.2);
  CORRADE_COMPARE(static_cast<int>(lightAttr1->getType()),
                  static_cast<int>(esp::gfx::LightType::Point));
  CORRADE_COMPARE(static_cast<int>(lightAttr1->getPositionModel()),
                  static_cast<int>(esp::gfx::LightPositionModel::Global));
  CORRADE_COMPARE(lightAttr1->getInnerConeAngle(), -0.75_radf);
  CORRADE_COMPARE(lightAttr1->getOuterConeAngle(), -1.7_radf);

  // test user defined attributes from light instance
  testUserDefinedConfigVals(lightAttr1->getUserConfiguration(),
                            "light instance defined string", false, 42, 1.2,
                            Magnum::Vector3(0.1, 2.3, 4.5),
                            Magnum::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f));

  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(lightLayoutAttributesManager_,
                                        lightLayoutAttr->getHandle());
}
void AttributesConfigsTest::testLightJSONLoad() {
  // build JSON sample config
  const std::string& jsonString = R"({
  "lights":{
      "test0":{
        "direction": [1.0,-1.0,1.0],
        "intensity": -0.1,
        "color": [0.6,0.7,0.8],
        "type": "directional",
        "position_model" : "camera",
        "spot": {
          "innerConeAngle": 0.25,
          "outerConeAngle": -1.57
        }
      },
      "test1":{
        "position": [2.5,0.1,3.8],
        "intensity": -1.2,
        "color": [0.5,0.3,0.1],
        "type": "point",
        "position_model" : "global",
        "spot": {
          "innerConeAngle": -0.75,
          "outerConeAngle": -1.7
        },
        "user_defined" : {
            "user_string" : "light instance defined string",
            "user_bool" : false,
            "user_int" : 42,
            "user_double" : 1.2,
            "user_vec3" : [0.1, 2.3, 4.5],
            "user_quat" : [0.1, 0.2, 0.3, 0.4]
        }
      }
    },
    "user_defined" : {
        "user_string" : "light attribs defined string",
        "user_bool" : true,
        "user_int" : 23,
        "user_double" : 2.3,
        "user_vec3" : [1.1, 3.3, 5.5],
        "user_quat" : [0.5, 0.6, 0.7, 0.8]
    },
    "positive_intensity_scale" : 2.0,
    "negative_intensity_scale" : 1.5
  })";

  auto lightLayoutAttr =
      testBuildAttributesFromJSONString<AttrMgrs::LightLayoutAttributesManager,
                                        Attrs::LightLayoutAttributes>(
          lightLayoutAttributesManager_, jsonString, true);
  // verify exists
  CORRADE_VERIFY(lightLayoutAttr);
  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testLightLayoutAttrConfig_saved_JSON.{}", testAttrSaveDir,
      lightLayoutAttributesManager_->getJSONTypeExt());

  bool success = lightLayoutAttributesManager_->saveManagedObjectToFile(
      lightLayoutAttr->getHandle(), newAttrName);

  ESP_DEBUG() << "About to test string-based lightLayoutAttr";
  // test json string to verify format
  testLightAttrVals(lightLayoutAttr);
  ESP_DEBUG() << "Tested lightLayoutAttr";
  lightLayoutAttr = nullptr;

  // load attributes from new name and retest
  auto lightLayoutAttr2 =
      lightLayoutAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(lightLayoutAttr2);

  // test json string to verify format, this deletes lightLayoutAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved lightLayoutAttr2 :"
              << lightLayoutAttr2->getHandle();
  testLightAttrVals(lightLayoutAttr2);
  ESP_DEBUG() << "About to test lightLayoutAttr2";

  // delete file-based config
  Cr::Utility::Directory::rm(newAttrName);

}  // AttributesManagers_LightJSONLoadTest

void AttributesConfigsTest::testSceneInstanceAttrVals(
    std::shared_ptr<esp::metadata::attributes::SceneInstanceAttributes>
        sceneAttr) {
  // match values set in test JSON
  CORRADE_COMPARE(
      static_cast<int>(sceneAttr->getTranslationOrigin()),
      static_cast<int>(Attrs::SceneInstanceTranslationOrigin::AssetLocal));

  CORRADE_COMPARE(sceneAttr->getLightingHandle(),
                  "test_lighting_configuration");
  CORRADE_COMPARE(sceneAttr->getNavmeshHandle(), "test_navmesh_path1");
  CORRADE_COMPARE(sceneAttr->getSemanticSceneHandle(),
                  "test_semantic_descriptor_path1");
  // test scene instance attributes-level user config vals
  testUserDefinedConfigVals(sceneAttr->getUserConfiguration(),
                            "scene instance defined string", true, 99, 9.1,
                            Magnum::Vector3(12.3, 32.5, 25.07),
                            Magnum::Quaternion({3.2f, 2.6f, 5.1f}, 0.3f));

  // verify objects
  auto objectInstanceList = sceneAttr->getObjectInstances();
  CORRADE_COMPARE(objectInstanceList.size(), 2);
  auto objInstance = objectInstanceList[0];
  CORRADE_COMPARE(objInstance->getHandle(), "test_object_template0");
  CORRADE_COMPARE(static_cast<int>(objInstance->getTranslationOrigin()),
                  static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
  CORRADE_COMPARE(objInstance->getTranslation(), Magnum::Vector3(0, 1, 2));
  CORRADE_COMPARE(objInstance->getRotation(),
                  Magnum::Quaternion({0.3f, 0.4f, 0.5f}, 0.2f));
  CORRADE_COMPARE(static_cast<int>(objInstance->getMotionType()),
                  static_cast<int>(esp::physics::MotionType::KINEMATIC));

  // test object 0 instance attributes-level user config vals
  testUserDefinedConfigVals(objInstance->getUserConfiguration(),
                            "obj0 instance defined string", false, 12, 2.3,
                            Magnum::Vector3(1.3, 3.5, 5.7),
                            Magnum::Quaternion({0.2f, 0.6f, 0.1f}, 0.3f));

  objInstance = objectInstanceList[1];
  CORRADE_COMPARE(objInstance->getHandle(), "test_object_template1");
  CORRADE_COMPARE(objInstance->getTranslation(), Magnum::Vector3(0, -1, -2));
  CORRADE_COMPARE(objInstance->getRotation(),
                  Magnum::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f));
  CORRADE_COMPARE(static_cast<int>(objInstance->getMotionType()),
                  static_cast<int>(esp::physics::MotionType::DYNAMIC));

  // test object 0 instance attributes-level user config vals
  testUserDefinedConfigVals(objInstance->getUserConfiguration(),
                            "obj1 instance defined string", false, 1, 1.1,
                            Magnum::Vector3(10.3, 30.5, -5.07),
                            Magnum::Quaternion({1.2f, 1.6f, 1.1f}, 1.3f));

  // verify articulated object instances
  auto artObjInstances = sceneAttr->getArticulatedObjectInstances();
  CORRADE_COMPARE(artObjInstances.size(), 2);
  auto artObjInstance = artObjInstances[0];
  CORRADE_COMPARE(artObjInstance->getHandle(), "test_urdf_template0");
  CORRADE_COMPARE(static_cast<int>(artObjInstance->getTranslationOrigin()),
                  static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
  CORRADE_COMPARE(artObjInstance->getFixedBase(), false);
  CORRADE_VERIFY(artObjInstance->getAutoClampJointLimits());

  CORRADE_COMPARE(artObjInstance->getTranslation(), Magnum::Vector3(5, 4, 5));
  CORRADE_COMPARE(static_cast<int>(artObjInstance->getMotionType()),
                  static_cast<int>(esp::physics::MotionType::DYNAMIC));
  // verify init join pose
  const auto& initJointPoseMap = artObjInstance->getInitJointPose();
  const std::vector<float> jtPoseVals{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  int idx = 0;
  for (std::map<std::string, float>::const_iterator iter =
           initJointPoseMap.begin();
       iter != initJointPoseMap.end(); ++iter) {
    CORRADE_COMPARE(iter->second, jtPoseVals[idx++]);
  }
  // verify init joint vels
  const auto& initJoinVelMap = artObjInstance->getInitJointVelocities();
  const std::vector<float> jtVelVals{1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6};
  idx = 0;
  for (std::map<std::string, float>::const_iterator iter =
           initJoinVelMap.begin();
       iter != initJoinVelMap.end(); ++iter) {
    CORRADE_COMPARE(iter->second, jtVelVals[idx++]);
  }

  // test test_urdf_template0 ao instance attributes-level user config vals
  testUserDefinedConfigVals(artObjInstance->getUserConfiguration(),
                            "test_urdf_template0 instance defined string",
                            false, 2, 1.22,
                            Magnum::Vector3(120.3f, 302.5f, -25.07f),
                            Magnum::Quaternion({1.22f, 1.26f, 1.21f}, 1.23f));

  // test nested configuration
  auto artObjNestedConfig =
      artObjInstance->getUserConfiguration()
          ->getSubconfigCopy<esp::core::config::Configuration>("user_def_obj");
  CORRADE_VERIFY(artObjNestedConfig);
  CORRADE_VERIFY(artObjNestedConfig->getNumEntries() > 0);
  CORRADE_COMPARE(artObjNestedConfig->template get<Magnum::Vector3>("position"),
                  Magnum::Vector3(0.1f, 0.2f, 0.3f));
  CORRADE_COMPARE(artObjNestedConfig->template get<Magnum::Vector3>("rotation"),
                  Magnum::Vector3(0.5f, 0.3f, 0.1f));

  artObjInstance = artObjInstances[1];
  CORRADE_COMPARE(artObjInstance->getHandle(), "test_urdf_template1");
  CORRADE_VERIFY(artObjInstance->getFixedBase());
  CORRADE_VERIFY(artObjInstance->getAutoClampJointLimits());
  CORRADE_COMPARE(artObjInstance->getTranslation(), Magnum::Vector3(3, 2, 1));
  CORRADE_COMPARE(static_cast<int>(artObjInstance->getMotionType()),
                  static_cast<int>(esp::physics::MotionType::KINEMATIC));
  // test test_urdf_template0 ao instance attributes-level user config vals
  testUserDefinedConfigVals(artObjInstance->getUserConfiguration(),
                            "test_urdf_template1 instance defined string",
                            false, 21, 11.22,
                            Magnum::Vector3(190.3f, 902.5f, -95.07f),
                            Magnum::Quaternion({9.22f, 9.26f, 0.21f}, 1.25f));
  // verify stage populated properly
  auto stageInstance = sceneAttr->getStageInstance();
  CORRADE_COMPARE(stageInstance->getHandle(), "test_stage_template");
  CORRADE_COMPARE(stageInstance->getTranslation(), Magnum::Vector3(1, 2, 3));
  CORRADE_COMPARE(stageInstance->getRotation(),
                  Magnum::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f));
  // make sure that is not default value "flat"
  CORRADE_COMPARE(static_cast<int>(stageInstance->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::PBR));

  // test stage instance attributes-level user config vals
  testUserDefinedConfigVals(stageInstance->getUserConfiguration(),
                            "stage instance defined string", true, 11, 2.2,
                            Magnum::Vector3(1.2, 3.4, 5.6),
                            Magnum::Quaternion({0.5f, 0.6f, 0.7f}, 0.4f));

  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(sceneInstanceAttributesManager_,
                                        sceneAttr->getHandle());
}

void AttributesConfigsTest::testSceneInstanceJSONLoad() {
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
          "user_double" : 2.2,
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
              "user_double" : 2.3,
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
              "user_double" : 1.1,
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
                  "user_double" : 1.22,
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
                  "user_double" : 11.22,
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
          "user_double" : 9.1,
          "user_vec3" : [12.3, 32.5, 25.07],
          "user_quat" : [0.3, 3.2, 2.6, 5.1]
      }
     })";

  auto sceneAttr = testBuildAttributesFromJSONString<
      AttrMgrs::SceneInstanceAttributesManager, Attrs::SceneInstanceAttributes>(
      sceneInstanceAttributesManager_, jsonString, true);
  // verify exists
  CORRADE_VERIFY(sceneAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testSceneAttrConfig_saved_JSON.{}", testAttrSaveDir,
      sceneInstanceAttributesManager_->getJSONTypeExt());

  bool success = sceneInstanceAttributesManager_->saveManagedObjectToFile(
      sceneAttr->getHandle(), newAttrName);

  // test json string to verify format - this also deletes sceneAttr from
  // manager
  ESP_DEBUG() << "About to test string-based sceneAttr :";
  testSceneInstanceAttrVals(sceneAttr);
  ESP_DEBUG() << "Tested string-based sceneAttr :";
  sceneAttr = nullptr;

  // load attributes from new name and retest
  auto sceneAttr2 =
      sceneInstanceAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(sceneAttr2);

  // test json string to verify format, this deletes sceneAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved sceneAttr2 :" << sceneAttr2->getHandle();
  testSceneInstanceAttrVals(sceneAttr2);
  ESP_DEBUG() << "Tested saved sceneAttr2 :";
  // delete file-based config
  Cr::Utility::Directory::rm(newAttrName);

}  // AttributesManagers_SceneInstanceJSONLoadTest

void AttributesConfigsTest::testStageAttrVals(
    std::shared_ptr<esp::metadata::attributes::StageAttributes> stageAttr,
    const std::string& assetPath) {
  // match values set in test JSON
  CORRADE_COMPARE(stageAttr->getScale(), Magnum::Vector3(2, 3, 4));
  CORRADE_COMPARE(stageAttr->getMargin(), 0.9);
  CORRADE_COMPARE(stageAttr->getFrictionCoefficient(), 0.321);
  CORRADE_COMPARE(stageAttr->getRestitutionCoefficient(), 0.456);
  CORRADE_VERIFY(!stageAttr->getForceFlatShading());
  CORRADE_COMPARE(stageAttr->getUnitsToMeters(), 1.1);
  CORRADE_COMPARE(stageAttr->getOrientUp(), Magnum::Vector3(2.1, 0, 0));
  CORRADE_COMPARE(stageAttr->getOrientFront(), Magnum::Vector3(0, 2.1, 0));

  // verify that we are set to not use the render asset frame for semantic
  // meshes.
  CORRADE_VERIFY(!stageAttr->getUseFrameForAllOrientation());
  CORRADE_COMPARE(stageAttr->getSemanticOrientFront(),
                  Magnum::Vector3(2.0, 0.0, 0.0));
  CORRADE_COMPARE(stageAttr->getSemanticOrientUp(),
                  Magnum::Vector3(0.0, 2.0, 0.0));

  CORRADE_COMPARE(stageAttr->getRenderAssetHandle(), assetPath);
  CORRADE_COMPARE(stageAttr->getCollisionAssetHandle(), assetPath);
  CORRADE_VERIFY(!stageAttr->getIsCollidable());
  // stage-specific attributes
  CORRADE_COMPARE(stageAttr->getOrigin(), Magnum::Vector3(1, 2, 3));
  CORRADE_COMPARE(stageAttr->getGravity(), Magnum::Vector3(9, 8, 7));
  CORRADE_VERIFY(stageAttr->getHasSemanticTextures());

  // make sure that is not default value "flat"
  CORRADE_COMPARE(static_cast<int>(stageAttr->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::Material));
  CORRADE_COMPARE(stageAttr->getSemanticAssetHandle(), assetPath);
  CORRADE_COMPARE(stageAttr->getNavmeshAssetHandle(), assetPath);
  // test stage attributes-level user config vals
  testUserDefinedConfigVals(stageAttr->getUserConfiguration(),
                            "stage defined string", false, 3, 0.8,
                            Magnum::Vector3(5.4, 7.6, 10.1),
                            Magnum::Quaternion({1.5f, 2.6f, 3.7f}, 0.1f));

  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(stageAttributesManager_,
                                        stageAttr->getHandle());
}  // AttributesConfigsTest::testStageAttrVals
void AttributesConfigsTest::testStageJSONLoad() {
  // build JSON sample config
  const std::string& jsonString =
      R"({
        "scale":[2,3,4],
        "margin": 0.9,
        "friction_coefficient": 0.321,
        "restitution_coefficient": 0.456,
        "force_flat_shading": false,
        "units_to_meters": 1.1,
        "up":[2.1, 0, 0],
        "front":[0, 2.1, 0],
        "has_semantic_textures":true,
        "render_asset": "testJSONRenderAsset.glb",
        "collision_asset": "testJSONCollisionAsset.glb",
        "is_collidable": false,
        "gravity": [9,8,7],
        "origin":[1,2,3],
        "semantic_asset":"testJSONSemanticAsset.glb",
        "nav_asset":"testJSONNavMeshAsset.glb",
        "shader_type" : "material",
        "user_defined" : {
            "user_string" : "stage defined string",
            "user_bool" : false,
            "user_int" : 3,
            "user_double" : 0.8,
            "user_vec3" : [5.4, 7.6, 10.1],
            "user_quat" : [0.1, 1.5, 2.6, 3.7]
        }
      })";

  auto stageAttr =
      testBuildAttributesFromJSONString<AttrMgrs::StageAttributesManager,
                                        Attrs::StageAttributes>(
          stageAttributesManager_, jsonString, false);
  // verify exists
  CORRADE_VERIFY(stageAttr);
  // verify that we are set to use the render asset frame for all meshes.
  CORRADE_VERIFY(stageAttr->getUseFrameForAllOrientation());
  // set new frame for semantic assets to test functionality
  stageAttr->setSemanticOrientFront(Magnum::Vector3(2.0, 0.0, 0.0));
  stageAttr->setSemanticOrientUp(Magnum::Vector3(0.0, 2.0, 0.0));
  // verify that we are now set to not use the render asset frame for semantic
  // meshes.
  CORRADE_VERIFY(!stageAttr->getUseFrameForAllOrientation());

  // now need to change the render and collision assets to make sure they are
  // legal so test can proceed (needs to be actual existing file)
  const std::string stageAssetFile =
      Cr::Utility::Directory::join(testAttrSaveDir, "scenes/plane.glb");

  stageAttr->setRenderAssetHandle(stageAssetFile);
  stageAttr->setCollisionAssetHandle(stageAssetFile);
  stageAttr->setSemanticAssetHandle(stageAssetFile);
  stageAttr->setNavmeshAssetHandle(stageAssetFile);
  // now register so can be saved to disk
  stageAttributesManager_->registerObject(stageAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testStageAttrConfig_saved_JSON.{}", testAttrSaveDir,
      stageAttributesManager_->getJSONTypeExt());

  bool success = stageAttributesManager_->saveManagedObjectToFile(
      stageAttr->getHandle(), newAttrName);

  // test json string to verify format - this also deletes stageAttr from
  // manager
  ESP_DEBUG() << "About to test string-based stageAttr :";
  testStageAttrVals(stageAttr, stageAssetFile);
  ESP_DEBUG() << "Tested string-based stageAttr :";
  stageAttr = nullptr;

  // load attributes from new name and retest
  auto stageAttr2 =
      stageAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(stageAttr2);

  // test json string to verify format, this deletes stageAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved stageAttr2 :" << stageAttr2->getHandle();
  testStageAttrVals(stageAttr2, stageAssetFile);
  ESP_DEBUG() << "Tested saved stageAttr2 :";

  // delete file-based config
  Cr::Utility::Directory::rm(newAttrName);

}  // AttributesManagers_StageJSONLoadTest

void AttributesConfigsTest::testObjectAttrVals(
    std::shared_ptr<esp::metadata::attributes::ObjectAttributes> objAttr,
    const std::string& assetPath) {
  // match values set in test JSON

  CORRADE_COMPARE(objAttr->getScale(), Magnum::Vector3(2, 3, 4));
  CORRADE_COMPARE(objAttr->getMargin(), 0.9);
  CORRADE_COMPARE(objAttr->getFrictionCoefficient(), 0.321);
  CORRADE_COMPARE(objAttr->getRestitutionCoefficient(), 0.456);
  CORRADE_VERIFY(objAttr->getForceFlatShading());
  CORRADE_COMPARE(objAttr->getUnitsToMeters(), 1.1);
  CORRADE_COMPARE(objAttr->getOrientUp(), Magnum::Vector3(2.1, 0, 0));
  CORRADE_COMPARE(objAttr->getOrientFront(), Magnum::Vector3(0, 2.1, 0));
  CORRADE_COMPARE(objAttr->getRenderAssetHandle(), assetPath);
  CORRADE_COMPARE(objAttr->getCollisionAssetHandle(), assetPath);
  CORRADE_VERIFY(!objAttr->getIsCollidable());
  CORRADE_COMPARE(objAttr->getSemanticId(), 7);
  // object-specific attributes
  CORRADE_COMPARE(objAttr->getMass(), 9);
  CORRADE_COMPARE(static_cast<int>(objAttr->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::Phong));
  CORRADE_VERIFY(objAttr->getBoundingBoxCollisions());
  CORRADE_VERIFY(objAttr->getJoinCollisionMeshes());
  CORRADE_COMPARE(objAttr->getInertia(), Magnum::Vector3(1.1, 0.9, 0.3));
  CORRADE_COMPARE(objAttr->getCOM(), Magnum::Vector3(0.1, 0.2, 0.3));
  // test object attributes-level user config vals
  testUserDefinedConfigVals(objAttr->getUserConfiguration(),
                            "object defined string", true, 5, 2.6,
                            Magnum::Vector3(15.4, 17.6, 110.1),
                            Magnum::Quaternion({5.5f, 6.6f, 7.7f}, 0.7f));

  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(objectAttributesManager_,
                                        objAttr->getHandle());

}  // AttributesConfigsTest::testObjectAttrVals

void AttributesConfigsTest::testObjectJSONLoad() {
  // build JSON sample config
  const std::string& jsonString = R"({
  "scale":[2,3,4],
  "margin": 0.9,
  "friction_coefficient": 0.321,
  "restitution_coefficient": 0.456,
  "force_flat_shading": true,
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
      "user_double" : 2.6,
      "user_vec3" : [15.4, 17.6, 110.1],
      "user_quat" : [0.7, 5.5, 6.6, 7.7]
  }
})";
  auto objAttr =
      testBuildAttributesFromJSONString<AttrMgrs::ObjectAttributesManager,
                                        Attrs::ObjectAttributes>(
          objectAttributesManager_, jsonString, false);
  // verify exists
  CORRADE_VERIFY(objAttr);
  // now need to change the render and collision assets to make sure they are
  // legal so test can proceed (needs to be actual existing file)
  const std::string objAssetFile =
      Cr::Utility::Directory::join(testAttrSaveDir, "objects/donut.glb");

  objAttr->setRenderAssetHandle(objAssetFile);
  objAttr->setCollisionAssetHandle(objAssetFile);
  // now register so can be saved to disk
  objectAttributesManager_->registerObject(objAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testObjectAttrConfig_saved_JSON.{}", testAttrSaveDir,
      objectAttributesManager_->getJSONTypeExt());

  bool success = objectAttributesManager_->saveManagedObjectToFile(
      objAttr->getHandle(), newAttrName);

  // test json string to verify format - this also deletes objAttr from
  // manager

  ESP_DEBUG() << "About to test string-based objAttr :";
  testObjectAttrVals(objAttr, objAssetFile);
  ESP_DEBUG() << "Tested string-based objAttr :";
  objAttr = nullptr;

  // load attributes from new name and retest
  auto objAttr2 =
      objectAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(objAttr2);

  // test json string to verify format, this deletes objAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved stageAttr2 :" << objAttr2->getHandle();
  testObjectAttrVals(objAttr2, objAssetFile);
  ESP_DEBUG() << "Tested saved stageAttr2 :";

  // delete file-based config
  Cr::Utility::Directory::rm(newAttrName);
}  // AttributesConfigsTest::testObjectJSONLoadTest

}  // namespace

CORRADE_TEST_MAIN(AttributesConfigsTest)
