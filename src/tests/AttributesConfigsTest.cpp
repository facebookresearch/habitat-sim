// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <string>

#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/managers/AOAttributesManager.h"
#include "esp/metadata/managers/AbstractAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PbrShaderAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/SensorAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

#include "esp/physics/RigidBase.h"

#include "configure.h"

// Remove once SensorSpecs are removed
#ifdef ESP_BUILD_WITH_AUDIO
#include "esp/sensor/AudioSensor.h"
#endif
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/EquirectangularSensor.h"
#include "esp/sensor/FisheyeSensor.h"

#ifdef ESP_BUILD_WITH_AUDIO
using esp::sensor::AudioSensorSpec;
#endif
using esp::sensor::CameraSensorSpec;
using esp::sensor::EquirectangularSensorSpec;
using esp::sensor::FisheyeSensorDoubleSphereSpec;
using esp::sensor::SensorSpec;
// End SensorSpec remove

namespace Cr = Corrade;
namespace Mn = Magnum;
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;

using Mn::Math::Literals::operator""_degf;
using Mn::Math::Literals::operator""_radf;

using esp::metadata::MetadataMediator;
using esp::metadata::PrimObjTypes;

using esp::physics::MotionType;

using AttrMgrs::AbstractAttributesManager;
using Attrs::ArticulatedObjectAttributes;
#ifdef ESP_BUILD_WITH_AUDIO
using Attrs::AudioSensorAttributes;
#endif
using Attrs::CameraSensorAttributes;
using Attrs::CustomSensorAttributes;
using Attrs::EquirectangularSensorAttributes;
using Attrs::FisheyeSensorAttributes;
using Attrs::MarkerSets;
using Attrs::ObjectAttributes;
using Attrs::PbrShaderAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::SceneInstanceAttributes;
using Attrs::SemanticAttributes;
using Attrs::StageAttributes;
using Attrs::UVSpherePrimitiveAttributes;

namespace {

/**
 * @brief Test attributes/configuration functionality via setting values from
 * JSON string, saving JSON to file and loading verifying saved JSON matches
 * expectations upon load.
 */

struct AttributesConfigsTest : Cr::TestSuite::Tester {
  explicit AttributesConfigsTest();

  // Test helper functions

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
   * @param strListSize Expected length of user-defined array of strings
   * @param strValue Expected string value
   * @param boolValue Expected boolean value
   * @param doubleValue Exptected double value
   * @param vec2Value Expected Mn::Vector2 value.
   * @param vec3Value Expected Mn::Vector3 value.
   * @param quatValue Expected quaternion value. Note that the JSON is read
   * with scalar at idx 0, whereas the quaternion constructor takes the vector
   * component in the first position and the scalar in the second. The JSON
   * label is expected to contain 'quat', 'rotat' or 'orient', case insensitive,
   * to be treated as an Mn::Quaternion.
   * @param vec4Value Expected Mn::Vector4 element. Any field with a label not
   * meeting Mn::Quaternion constraints is treated as an Mn::Vector4.
   */
  void testUserDefinedConfigVals(
      std::shared_ptr<esp::core::config::Configuration> userConfig,
      int strListSize,
      const std::string& strValue,
      bool boolValue,
      int int_val,
      double doubleValue,
      Mn::Vector2 vec2Value,
      Mn::Vector3 vec3Value,
      Mn::Quaternion quatValue,
      Mn::Vector4 vec4Value);

  /**
   * @brief Collection of markers making up a single MarkerSet
   */
  typedef std::unordered_map<std::string, Mn::Vector3> MarkerSetTestMap;
  /**
   * @brief Collection of MarkerSet making up a single LinkSet
   */
  typedef std::unordered_map<std::string, MarkerSetTestMap> LinkSetTestMap;
  /**
   * @brief Collection of LinkSets making up a single TaskSet
   */
  typedef std::unordered_map<std::string, LinkSetTestMap> TaskSetTestMap;
  /**
   * @brief Collection of TaskSets making up the entire MarkerSets construction.
   */
  typedef std::unordered_map<std::string, TaskSetTestMap> AllMarkerSetsTestMap;

  /**
   * @brief This method will test the marker-sets configurations being loaded in
   * stage, object and ao configs.
   * @param markerSetsConfig The marker-sets configuration object whose contents
   * are to be tested.
   * @param markerSetsInfoHierarchy The hieraarchy the marker sets should
   * follow.
   */
  void testMarkerSetsConfigVals(
      std::shared_ptr<Attrs::MarkerSets> markerSetsConfig,
      const AllMarkerSetsTestMap&  // markers in link subset
          markerSetsInfoHierarchy);

  //////////////////////////////
  // Test attributes values
  // These tests validate passed attributes' values to what is expected.

  /**
   * @brief This test will verify that the physics attributes' managers' JSON
   * loading process is working as expected.
   */
  void testPhysicsAttrVals(
      std::shared_ptr<Attrs::PhysicsManagerAttributes> physMgrAttr);

  /**
   * @brief This test will verify that the PBR/IBL shader config attributes'
   * managers' JSON loading process is working as expected.
   */
  void testPbrShaderAttrVals(
      std::shared_ptr<Attrs::PbrShaderAttributes> pbrShaderAttr);
  /**
   * @brief This test will verify that the Light Attributes' managers' JSON
   * loading process is working as expected.
   */
  void testLightAttrVals(
      std::shared_ptr<Attrs::LightLayoutAttributes> lightLayoutAttr);

  /**
   * @brief This test will verify that the Scene Instance Attributes' managers'
   * JSON loading process is working as expected.
   */
  void testSceneInstanceAttrVals(
      std::shared_ptr<Attrs::SceneInstanceAttributes> sceneInstAttr);

  /**
   * @brief This test will verify that the root-level scene instance user
   * defined attribute values are as expected.
   */
  void testSceneInstanceRootUserDefinedAttrVals(
      std::shared_ptr<esp::core::config::Configuration> userAttrs);
  /**
   * @brief This test will verify that the Semantic attributes' managers' JSON
   * loading process is working as expected.
   */
  void testSemanticAttrVals(
      std::shared_ptr<Attrs::SemanticAttributes> semanticAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the AudioSensorAttributes' will match
   * expected values.
   */

#ifdef ESP_BUILD_WITH_AUDIO

  void testAudioSensorAttrVals(
      std::shared_ptr<Attrs::AudioSensorAttributes> audioAttr,
      const std::string& assetPath);
#endif
  /**
   * @brief This test will verify that the CameraSensorAttributes' will match
   * expected values.
   */
  void testCameraSensorAttrVals(
      std::shared_ptr<Attrs::CameraSensorAttributes> cameraAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the CustomSensorAttributes' will match
   * expected values.
   */
  void testCustomSensorAttrVals(
      std::shared_ptr<Attrs::CustomSensorAttributes> customAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the EquirectangularSensorAttributes' will
   * match expected values.
   */
  void testEquirectangularSensorAttrVals(
      std::shared_ptr<Attrs::EquirectangularSensorAttributes> equirectAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the FisheyeSensorAttributes' will match
   * expected values.
   */
  void testFisheyeSensorAttrVals(
      std::shared_ptr<Attrs::FisheyeSensorAttributes> fisheyeAttr,
      const std::string& assetPath);

  /**
   * @brief This test will verify that the Stage attributes' managers' JSON
   * loading process is working as expected.
   */
  void testStageAttrVals(std::shared_ptr<Attrs::StageAttributes> stageAttr,
                         const std::string& assetPath);

  /**
   * @brief This test will verify that the Object attributes' managers' JSON
   * loading process is working as expected.
   */
  void testObjectAttrVals(std::shared_ptr<Attrs::ObjectAttributes> objAttr,
                          const std::string& assetPath);
  /**
   * @brief This test will verify that the Articulated Object attributes'
   * managers' JSON loading process is working as expected.
   */
  void testArticulatedObjectAttrVals(
      std::shared_ptr<Attrs::ArticulatedObjectAttributes> artObjAttr,
      const std::string& assetPath,
      const std::string& urdfPath);

  // actual test functions
  // These tests build strings containing legal JSON config data to use to build
  // an appropriate attributes configuration.  The resultant configuration is
  // then tested for accuracy.  These functions also save a copy to a json file,
  // and then reload the copy, to make sure the saving and loading process is
  // correct.
  void testPhysicsJSONLoad();
  void testPbrShaderAttrJSONLoad();
  void testLightJSONLoad();
  void testSceneInstanceJSONLoad();
  void testSemanticJSONLoad();
  /**
   * @brief Remove once SensorSpec is removed
   */
  void testSensorAttrSpecLoad();
  void testSensorAttrJSONLoad();
  void testStageJSONLoad();
  void testObjectJSONLoad();
  void testArticulatedObjectJSONLoad();

  // test member vars

  esp::metadata::MetadataMediator::uptr MM = nullptr;
  esp::logging::LoggingContext loggingContext_;
  AttrMgrs::LightLayoutAttributesManager::ptr lightLayoutAttributesManager_ =
      nullptr;
  AttrMgrs::AOAttributesManager::ptr artObjAttributesManager_ = nullptr;
  AttrMgrs::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  AttrMgrs::PbrShaderAttributesManager::ptr pbrShaderAttributesManager_ =
      nullptr;
  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;
  AttrMgrs::SceneInstanceAttributesManager::ptr
      sceneInstanceAttributesManager_ = nullptr;
  AttrMgrs::SemanticAttributesManager::ptr semanticAttributesManager_ = nullptr;

  AttrMgrs::SensorAttributesManager::ptr sensorAttributesManager_ = nullptr;

  AttrMgrs::StageAttributesManager::ptr stageAttributesManager_ = nullptr;

};  // struct AttributesConfigsTest

AttributesConfigsTest::AttributesConfigsTest() {
  // set up a default simulation config to initialize MM
  auto cfg = esp::sim::SimulatorConfiguration{};
  MM = MetadataMediator::create_unique(cfg);
  // get attributes managers for default dataset
  lightLayoutAttributesManager_ = MM->getLightLayoutAttributesManager();
  artObjAttributesManager_ = MM->getAOAttributesManager();
  objectAttributesManager_ = MM->getObjectAttributesManager();
  physicsAttributesManager_ = MM->getPhysicsAttributesManager();
  pbrShaderAttributesManager_ = MM->getPbrShaderAttributesManager();
  sceneInstanceAttributesManager_ = MM->getSceneInstanceAttributesManager();
  semanticAttributesManager_ = MM->getSemanticAttributesManager();
  sensorAttributesManager_ = MM->getSensorAttributesManager();
  stageAttributesManager_ = MM->getStageAttributesManager();

  addTests({
      &AttributesConfigsTest::testPhysicsJSONLoad,
      &AttributesConfigsTest::testPbrShaderAttrJSONLoad,
      &AttributesConfigsTest::testLightJSONLoad,
      &AttributesConfigsTest::testSceneInstanceJSONLoad,
      &AttributesConfigsTest::testSensorAttrSpecLoad,
      &AttributesConfigsTest::testSemanticJSONLoad,
      &AttributesConfigsTest::testSensorAttrJSONLoad,
      &AttributesConfigsTest::testStageJSONLoad,
      &AttributesConfigsTest::testObjectJSONLoad,
      &AttributesConfigsTest::testArticulatedObjectJSONLoad,
  });
}

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
    int strListSize,
    const std::string& strValue,
    bool boolValue,
    int int_val,
    double doubleValue,
    Mn::Vector2 vec2Value,
    Mn::Vector3 vec3Value,
    Mn::Quaternion quatValue,
    Mn::Vector4 vec4Value) {
  // user defined attributes from light instance
  CORRADE_VERIFY(userConfig);
  // get user subconfig to test individual fields within string list
  auto userStrListSubconfig =
      userConfig->getSubconfigCopy<esp::core::config::Configuration>(
          "user_str_array");
  // Verify it exists
  CORRADE_VERIFY(userStrListSubconfig);
  // Verify size
  CORRADE_COMPARE(userStrListSubconfig->getNumEntries(), strListSize);

  // Verify fields
  // ["test_00", "test_01", "test_02", "test_03"],
  for (int i = 0; i < strListSize; ++i) {
    const std::string subKey = Cr::Utility::formatString("{:.03d}", i);
    const std::string fieldVal = Cr::Utility::formatString("test_{:.02d}", i);

    CORRADE_COMPARE(userStrListSubconfig->get<std::string>(subKey), fieldVal);
  }

  // Verify primary subconfig
  CORRADE_COMPARE(userConfig->get<std::string>("user_string"), strValue);
  CORRADE_COMPARE(userConfig->get<bool>("user_bool"), boolValue);
  CORRADE_COMPARE(userConfig->get<int>("user_int"), int_val);
  if (userConfig->hasValue("user_double")) {
    // this triggers an error on CI that we will revisit
    CORRADE_COMPARE(userConfig->get<double>("user_double"), doubleValue);
  } else {
    ESP_DEBUG() << "Temporarily skipping test that triggered CI error on key "
                   "`user_double`.";
  }
  CORRADE_COMPARE(userConfig->get<Mn::Vector2>("user_vec2"), vec2Value);
  CORRADE_COMPARE(userConfig->get<Mn::Vector3>("user_vec3"), vec3Value);
  // Test access as a color
  CORRADE_COMPARE(userConfig->get<Mn::Color3>("user_vec3"), vec3Value);
  CORRADE_COMPARE(userConfig->get<Mn::Quaternion>("user_quat"), quatValue);
  CORRADE_COMPARE(userConfig->get<Mn::Vector4>("user_vec4"), vec4Value);
  // Test access as a color
  CORRADE_COMPARE(userConfig->get<Mn::Color4>("user_vec4"), vec4Value);

}  // AttributesConfigsTest::testUserDefinedConfigVals

void AttributesConfigsTest::testMarkerSetsConfigVals(
    std::shared_ptr<Attrs::MarkerSets> markerSetsConfig,
    const AllMarkerSetsTestMap& markerSetsInfoHierarchy) {
  // Testing as nested subconfigs
  for (const auto& taskSetInfoEntry : markerSetsInfoHierarchy) {
    // Key is first
    const std::string taskSetKey = taskSetInfoEntry.first;
    CORRADE_VERIFY(markerSetsConfig->hasSubconfig(taskSetKey));
    // Retrive named TaskSet
    const auto taskSet = markerSetsConfig->getSubconfigView(taskSetKey);
    CORRADE_VERIFY(taskSet);
    // Submap of test data
    const auto& taskSetInfoMap = taskSetInfoEntry.second;
    for (const auto& linkSetInfoEntry : taskSetInfoMap) {
      const std::string linkSetKey = linkSetInfoEntry.first;
      CORRADE_VERIFY(taskSet->hasSubconfig(linkSetKey));
      // Retrieve named per-link markerset
      const auto linkSet = taskSet->getSubconfigView(linkSetKey);
      CORRADE_VERIFY(linkSet);
      // Per linkmarker sets
      const auto& linkSetInfoMap = linkSetInfoEntry.second;
      for (const auto& markerSetInfoEntry : linkSetInfoMap) {
        const std::string markerSetKey = markerSetInfoEntry.first;
        CORRADE_VERIFY(linkSet->hasSubconfig(markerSetKey));
        // Retrive a specific link's markerSet
        const auto markerSet = linkSet->getSubconfigView(markerSetKey);
        CORRADE_VERIFY(markerSet);
        // Verify that subconfig named "markers' exists"
        CORRADE_VERIFY(markerSet->hasSubconfig("markers"));
        // Get the set of markers
        const auto& markers = markerSet->getSubconfigView("markers");
        // key-value map that should match mapping of markers in link subset
        const auto& markersInfo = markerSetInfoEntry.second;
        // Verify there are the expected number of marker points
        CORRADE_COMPARE(markers->getNumValues(), markersInfo.size());
        // Verify that subconfig has each marker point
        for (const auto& markerInfo : markersInfo) {
          const std::string markerKey = markerInfo.first;
          // Verify markers contains a value with passed key
          CORRADE_VERIFY(markers->hasValue(markerKey));
          const Mn::Vector3 markerPoint = markerInfo.second;
          // Make sure marker point is present
          CORRADE_COMPARE(markers->get<Mn::Vector3>(markerKey), markerPoint);
        }  // for each marker within marker set
      }    // for each marker set within link set
    }      // for each link set in task set
  }        // for each task set within MarkerSets aggregation
  // TODO : test as MarkerSets class hierarchy
}  // AttributesConfigsTest::testMarkerSetsConfigVals

/////////////  Begin JSON String-based tests

void AttributesConfigsTest::testPhysicsAttrVals(
    std::shared_ptr<Attrs::PhysicsManagerAttributes> physMgrAttr) {
  // match values set in test JSON

  CORRADE_COMPARE(physMgrAttr->getGravity(), Mn::Vector3(1, 2, 3));
  CORRADE_COMPARE(physMgrAttr->getTimestep(), 1.0);
  CORRADE_COMPARE(physMgrAttr->getSimulator(), "bullet_test");
  CORRADE_COMPARE(physMgrAttr->getFrictionCoefficient(), 1.4);
  CORRADE_COMPARE(physMgrAttr->getRestitutionCoefficient(), 1.1);
  // test physics manager attributes-level user config vals
  testUserDefinedConfigVals(
      physMgrAttr->getUserConfiguration(), 4, "pm defined string", true, 15,
      12.6, Mn::Vector2(1.0f, 2.0f), Mn::Vector3(215.4, 217.6, 2110.1),
      Mn::Quaternion({5.2f, 6.2f, 7.2f}, 0.2f),
      Mn::Vector4(3.5f, 4.6f, 5.7f, 6.9f));
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
      "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
      "user_string" : "pm defined string",
      "user_bool" : true,
      "user_int" : 15,
      "user_double" : 12.6,
      "user_vec2" : [1.0, 2.0],
      "user_vec3" : [215.4, 217.6, 2110.1],
      "user_quat" : [0.2, 5.2, 6.2, 7.2],
      "user_vec4" : [3.5, 4.6, 5.7, 6.9]
  }
})";
  // Build an attributes based on the above json string
  auto physMgrAttr = physicsAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, true);
  // verify exists
  CORRADE_VERIFY(physMgrAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testPhysicsAttrConfig_saved_JSON.{}", TEST_ASSETS,
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
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testPhysicsJSONLoad

void AttributesConfigsTest::testPbrShaderAttrVals(
    std::shared_ptr<Attrs::PbrShaderAttributes> pbrShaderAttr) {
  CORRADE_VERIFY(!pbrShaderAttr->getEnableDirectLighting());
  CORRADE_VERIFY(!pbrShaderAttr->getEnableIBL());

  CORRADE_COMPARE(pbrShaderAttr->getIBLBrdfLUTAssetHandle(),
                  "brdflut_test_only_image.png");
  CORRADE_COMPARE(pbrShaderAttr->getIBLEnvMapAssetHandle(),
                  "envmap_test_only_image.hdr");
  CORRADE_COMPARE(pbrShaderAttr->getPbrShaderHelperKey(),
                  "brdflut_test_only_image.png_envmap_test_only_image.hdr");

  CORRADE_COMPARE(pbrShaderAttr->getDirectLightIntensity(), 1.23f);

  CORRADE_VERIFY(pbrShaderAttr->getSkipCalcMissingTBN());
  CORRADE_VERIFY(pbrShaderAttr->getUseMikkelsenTBN());

  CORRADE_VERIFY(pbrShaderAttr->getUseDirectLightTonemap());
  CORRADE_VERIFY(!pbrShaderAttr->getUseIBLTonemap());
  CORRADE_VERIFY(!pbrShaderAttr->getUseBurleyDiffuse());

  // verify the layer skipping is present
  CORRADE_VERIFY(pbrShaderAttr->getSkipCalcClearcoatLayer());
  CORRADE_VERIFY(pbrShaderAttr->getSkipCalcSpecularLayer());
  CORRADE_VERIFY(pbrShaderAttr->getSkipCalcAnisotropyLayer());

  CORRADE_COMPARE(pbrShaderAttr->getDirectDiffuseScale(), 1.5f);
  CORRADE_COMPARE(pbrShaderAttr->getDirectSpecularScale(), 2.5f);
  CORRADE_COMPARE(pbrShaderAttr->getIBLDiffuseScale(), 3.5f);
  CORRADE_COMPARE(pbrShaderAttr->getIBLSpecularScale(), 4.5f);

  CORRADE_COMPARE(pbrShaderAttr->getTonemapExposure(), 6.7f);

  CORRADE_VERIFY(pbrShaderAttr->getMapMatTxtrToLinear());
  CORRADE_VERIFY(pbrShaderAttr->getMapIBLTxtrToLinear());
  CORRADE_VERIFY(pbrShaderAttr->getMapOutputToSRGB());
  CORRADE_COMPARE(pbrShaderAttr->getGamma(), 8.9f);

  // test PBR/IBL Shader attributes-level user config vals
  testUserDefinedConfigVals(pbrShaderAttr->getUserConfiguration(), 5,
                            "pbr defined string", false, 11, 22.6,
                            Mn::Vector2(3.0f, 4.0f), Mn::Vector3(5.4, 6.5, 7.1),
                            Mn::Quaternion({6.7f, 7.8f, 8.9f}, 0.3f),
                            Mn::Vector4(2.3f, 4.5f, 6.7f, 8.9f));
  // remove added template
  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(pbrShaderAttributesManager_,
                                        pbrShaderAttr->getHandle());

}  // AttributesConfigsTest::testPbrShaderAttrVals

void AttributesConfigsTest::testPbrShaderAttrJSONLoad() {
  // build JSON sample config
  // add dummy test so that test will run
  CORRADE_VERIFY(true);
  const std::string& jsonString = R"({
  "enable_direct_lights": false,
  "enable_ibl": false,
  "ibl_blut_filename": "brdflut_test_only_image.png",
  "ibl_envmap_filename": "envmap_test_only_image.hdr",
  "direct_light_intensity": 1.23,
  "skip_missing_tbn_calc": true,
  "use_mikkelsen_tbn": true,
  "map_mat_txtr_to_linear": true,
  "map_ibl_txtr_to_linear": true,
  "map_output_to_srgb": true,
  "use_direct_tonemap": true,
  "use_ibl_tonemap": false,
  "use_burley_diffuse": false,
  "skip_clearcoat_calc": true,
  "skip_specular_layer_calc": true,
  "skip_anisotropy_layer_calc": true,
  "direct_diffuse_scale": 1.5,
  "direct_specular_scale": 2.5,
  "ibl_diffuse_scale": 3.5,
  "ibl_specular_scale": 4.5,
  "tonemap_exposure": 6.7,
  "gamma": 8.9,
  "user_defined" : {
      "user_str_array" : ["test_00", "test_01", "test_02", "test_03", "test_04"],
      "user_string" : "pbr defined string",
      "user_bool" : false,
      "user_int" : 11,
      "user_double" : 22.6,
      "user_vec2" : [3.0, 4.0],
      "user_vec3" : [5.4, 6.5, 7.1],
      "user_quat" : [0.3, 6.7, 7.8, 8.9],
      "user_vec4" : [2.3, 4.5, 6.7, 8.9]
  }
})";

  auto pbrMgrAttr = pbrShaderAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, true);

  // verify exists
  CORRADE_VERIFY(pbrMgrAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testPbrShaderAttrConfig_saved_JSON.{}", TEST_ASSETS,
      pbrShaderAttributesManager_->getJSONTypeExt());

  bool success = pbrShaderAttributesManager_->saveManagedObjectToFile(
      pbrMgrAttr->getHandle(), newAttrName);

  ESP_DEBUG() << "About to test string-based pbrMgrAttr";
  // test json string to verify format, this deletes pbrMgrAttr from registry
  testPbrShaderAttrVals(pbrMgrAttr);
  ESP_DEBUG() << "Tested pbrMgrAttr";

  pbrMgrAttr = nullptr;

  // load attributes from new name and retest
  auto pbrMgrAttr2 =
      pbrShaderAttributesManager_->createObjectFromJSONFile(newAttrName, true);

  // verify file-based config exists
  CORRADE_VERIFY(pbrMgrAttr2);

  ESP_DEBUG() << "About to test saved pbrMgrAttr2 :"
              << pbrMgrAttr2->getHandle();
  // test json string to verify format, this deletes pbrMgrAttr2 from
  // registry
  testPbrShaderAttrVals(pbrMgrAttr2);
  ESP_DEBUG() << "Tested pbrMgrAttr";

  // delete file-based config
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testPbrShaderAttrJSONLoad

void AttributesConfigsTest::testLightAttrVals(
    std::shared_ptr<Attrs::LightLayoutAttributes> lightLayoutAttr) {
  // test light layout attributes-level user config vals
  testUserDefinedConfigVals(lightLayoutAttr->getUserConfiguration(), 4,
                            "light attribs defined string", true, 23, 2.3,
                            Mn::Vector2(1.1f, 2.2f), Mn::Vector3(1.1, 3.3, 5.5),
                            Mn::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f),
                            Mn::Vector4(1.5f, 1.6f, 1.7f, 1.9f));
  CORRADE_COMPARE(lightLayoutAttr->getPositiveIntensityScale(), 2.0);
  CORRADE_COMPARE(lightLayoutAttr->getNegativeIntensityScale(), 1.5);
  auto lightAttr0 = lightLayoutAttr->getLightInstance("test0");
  // verify that lightAttr0 exists
  CORRADE_VERIFY(lightAttr0);

  // match values set in test JSON

  CORRADE_COMPARE(lightAttr0->getDirection(), Mn::Vector3(1.0, -1.0, 1.0));
  CORRADE_COMPARE(lightAttr0->getColor(), Mn::Vector3(0.6, 0.7, 0.8));

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

  CORRADE_COMPARE(lightAttr1->getPosition(), Mn::Vector3(2.5, 0.1, 3.8));
  CORRADE_COMPARE(lightAttr1->getColor(), Mn::Vector3(0.5, 0.3, 0.1));

  CORRADE_COMPARE(lightAttr1->getIntensity(), -1.2);
  CORRADE_COMPARE(static_cast<int>(lightAttr1->getType()),
                  static_cast<int>(esp::gfx::LightType::Point));
  CORRADE_COMPARE(static_cast<int>(lightAttr1->getPositionModel()),
                  static_cast<int>(esp::gfx::LightPositionModel::Global));
  CORRADE_COMPARE(lightAttr1->getInnerConeAngle(), -0.75_radf);
  CORRADE_COMPARE(lightAttr1->getOuterConeAngle(), -1.7_radf);

  // test user defined attributes from light instance
  testUserDefinedConfigVals(lightAttr1->getUserConfiguration(), 4,
                            "light instance defined string", false, 42, 1.2,
                            Mn::Vector2(1.2f, 2.1f), Mn::Vector3(0.1, 2.3, 4.5),
                            Mn::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f),
                            Mn::Vector4(1.1f, 1.2f, 1.3f, 1.4f));

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
            "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
            "user_string" : "light instance defined string",
            "user_bool" : false,
            "user_int" : 42,
            "user_double" : 1.2,
            "user_vec2" : [1.2, 2.1],
            "user_vec3" : [0.1, 2.3, 4.5],
            "user_quat" : [0.1, 0.2, 0.3, 0.4],
            "user_vec4" : [1.1, 1.2, 1.3, 1.4]
        }
      }
    },
    "user_defined" : {
        "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
        "user_string" : "light attribs defined string",
        "user_bool" : true,
        "user_int" : 23,
        "user_double" : 2.3,
        "user_vec2" : [1.1, 2.2],
        "user_vec3" : [1.1, 3.3, 5.5],
        "user_quat" : [0.5, 0.6, 0.7, 0.8],
        "user_vec4" : [1.5, 1.6, 1.7, 1.9]
    },
    "positive_intensity_scale" : 2.0,
    "negative_intensity_scale" : 1.5
  })";

  // Build an attributes based on the above json string
  auto lightLayoutAttr =
      lightLayoutAttributesManager_->createObjectFromJSONString(
          "new_template_from_json", jsonString, true);
  // verify exists
  CORRADE_VERIFY(lightLayoutAttr);
  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testLightLayoutAttrConfig_saved_JSON.{}", TEST_ASSETS,
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
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testLightJSONLoad

void AttributesConfigsTest::testSceneInstanceRootUserDefinedAttrVals(
    std::shared_ptr<esp::core::config::Configuration> userAttrs) {
  // test Scene Instance Attributes-level user config vals
  testUserDefinedConfigVals(userAttrs, 4, "scene instance defined string", true,
                            99, 9.1, Mn::Vector2(1.3f, 2.4f),
                            Mn::Vector3(12.3, 32.5, 25.07),
                            Mn::Quaternion({3.2f, 2.6f, 5.1f}, 0.3f),
                            Mn::Vector4(13.5f, 14.6f, 15.7f, 16.9f));
}  // AttributesConfigsTest::testSceneInstanceRootUserDefinedAttrVals

void AttributesConfigsTest::testSceneInstanceAttrVals(
    std::shared_ptr<Attrs::SceneInstanceAttributes> sceneAttr) {
  // match values set in test JSON
  CORRADE_COMPARE(
      static_cast<int>(sceneAttr->getTranslationOrigin()),
      static_cast<int>(Attrs::SceneInstanceTranslationOrigin::AssetLocal));

  CORRADE_COMPARE(sceneAttr->getLightingHandle(),
                  "test_lighting_configuration");
  CORRADE_COMPARE(sceneAttr->getNavmeshHandle(), "test_navmesh_path1");
  CORRADE_COMPARE(sceneAttr->getSemanticSceneHandle(),
                  "test_semantic_descriptor_path1");
  // test Scene Instance Attributes-level user config vals
  testSceneInstanceRootUserDefinedAttrVals(sceneAttr->getUserConfiguration());

  // test Scene Instance Attributes-level user config vals retrieved from MM
  // directly
  testSceneInstanceRootUserDefinedAttrVals(
      MM->getSceneInstanceUserConfiguration(sceneAttr->getHandle()));

  // verify objects
  auto objectInstanceList = sceneAttr->getObjectInstances();
  CORRADE_COMPARE(objectInstanceList.size(), 2);
  {
    auto objInstance = objectInstanceList[0];
    CORRADE_COMPARE(objInstance->getHandle(), "test_object_template0");
    CORRADE_COMPARE(
        static_cast<int>(objInstance->getTranslationOrigin()),
        static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
    CORRADE_COMPARE(objInstance->getTranslation(), Mn::Vector3(0, 1, 2));
    CORRADE_COMPARE(objInstance->getRotation(),
                    Mn::Quaternion({0.3f, 0.4f, 0.5f}, 0.2f));
    CORRADE_COMPARE(static_cast<int>(objInstance->getMotionType()),
                    static_cast<int>(esp::physics::MotionType::KINEMATIC));
    CORRADE_VERIFY(objInstance->getApplyScaleToMass());
    CORRADE_COMPARE(objInstance->getUniformScale(), 1.1f);
    CORRADE_COMPARE(objInstance->getNonUniformScale(),
                    Mn::Vector3(1.1f, 2.2f, 3.3f));

    // test object 0 instance attributes-level user config vals
    testUserDefinedConfigVals(
        objInstance->getUserConfiguration(), 4, "obj0 instance defined string",
        false, 12, 2.3, Mn::Vector2(1.6f, 2.8f), Mn::Vector3(1.3, 3.5, 5.7),
        Mn::Quaternion({0.2f, 0.6f, 0.1f}, 0.3f),
        Mn::Vector4(4.5f, 3.6f, 2.7f, 1.9f));
  }
  {
    auto objInstance = objectInstanceList[1];
    CORRADE_COMPARE(objInstance->getHandle(), "test_object_template1");
    CORRADE_COMPARE(objInstance->getTranslation(), Mn::Vector3(0, -1, -2));
    CORRADE_COMPARE(objInstance->getRotation(),
                    Mn::Quaternion({0.6f, 0.7f, 0.8f}, 0.5f));
    CORRADE_COMPARE(static_cast<int>(objInstance->getMotionType()),
                    static_cast<int>(esp::physics::MotionType::DYNAMIC));
    CORRADE_VERIFY(!objInstance->getApplyScaleToMass());
    CORRADE_COMPARE(objInstance->getUniformScale(), 2.1f);
    CORRADE_COMPARE(objInstance->getNonUniformScale(),
                    Mn::Vector3(2.1f, 3.2f, 4.3f));

    // test object 1 instance attributes-level user config vals
    testUserDefinedConfigVals(
        objInstance->getUserConfiguration(), 4, "obj1 instance defined string",
        false, 1, 1.1, Mn::Vector2(2.1f, 3.2f), Mn::Vector3(10.3, 30.5, -5.07),
        Mn::Quaternion({1.2f, 1.6f, 1.1f}, 1.3f),
        Mn::Vector4(4.5f, 5.6f, 6.7f, 7.9f));
  }

  // verify articulated object instances
  auto artObjInstances = sceneAttr->getArticulatedObjectInstances();
  CORRADE_COMPARE(artObjInstances.size(), 3);
  {
    auto artObjInstance = artObjInstances[0];
    CORRADE_COMPARE(artObjInstance->getHandle(), "test_urdf_template0");
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getTranslationOrigin()),
        static_cast<int>(Attrs::SceneInstanceTranslationOrigin::COM));
    CORRADE_COMPARE(static_cast<int>(artObjInstance->getBaseType()),
                    static_cast<int>(Attrs::ArticulatedObjectBaseType::Fixed));
    CORRADE_VERIFY(artObjInstance->getApplyScaleToMass());
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getInertiaSource()),
        static_cast<int>(Attrs::ArticulatedObjectInertiaSource::URDF));
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getLinkOrder()),
        static_cast<int>(Attrs::ArticulatedObjectLinkOrder::URDFOrder));
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getRenderMode()),
        static_cast<int>(Attrs::ArticulatedObjectRenderMode::LinkVisuals));
    CORRADE_VERIFY(artObjInstance->getAutoClampJointLimits());

    CORRADE_COMPARE(artObjInstance->getTranslation(), Mn::Vector3(5, 4, 5));
    CORRADE_COMPARE(static_cast<int>(artObjInstance->getMotionType()),
                    static_cast<int>(esp::physics::MotionType::DYNAMIC));
    // verify init join pose
    const auto& initJointPoseVec = artObjInstance->getInitJointPose();
    const std::vector<float> jtPoseVals{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    CORRADE_COMPARE(initJointPoseVec, jtPoseVals);

    // verify init joint vels
    const auto& initJointVelVec = artObjInstance->getInitJointVelocities();
    const std::vector<float> jtVelVals{1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6};
    CORRADE_COMPARE(initJointVelVec, jtVelVals);

    // test test_urdf_template0 ao instance attributes-level user config vals
    testUserDefinedConfigVals(artObjInstance->getUserConfiguration(), 4,
                              "test_urdf_template0 instance defined string",
                              false, 2, 1.22, Mn::Vector2(3.1f, 4.2f),
                              Mn::Vector3(120.3f, 302.5f, -25.07f),
                              Mn::Quaternion({1.22f, 1.26f, 1.21f}, 1.23f),
                              Mn::Vector4(13.5f, 24.6f, 35.7f, 46.9f));

    // test nested configuration
    auto artObjNestedConfig =
        artObjInstance->getUserConfiguration()
            ->getSubconfigCopy<esp::core::config::Configuration>(
                "user_def_obj");
    CORRADE_VERIFY(artObjNestedConfig);
    CORRADE_VERIFY(artObjNestedConfig->getNumEntries() > 0);
    CORRADE_COMPARE(artObjNestedConfig->template get<Mn::Vector3>("position"),
                    Mn::Vector3(0.1f, 0.2f, 0.3f));
    CORRADE_COMPARE(artObjNestedConfig->template get<Mn::Vector3>("rotation"),
                    Mn::Vector3(0.5f, 0.3f, 0.1f));
  }
  {
    auto artObjInstance = artObjInstances[1];
    CORRADE_COMPARE(artObjInstance->getHandle(), "test_urdf_template1");

    CORRADE_COMPARE(static_cast<int>(artObjInstance->getBaseType()),
                    static_cast<int>(Attrs::ArticulatedObjectBaseType::Free));
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getInertiaSource()),
        static_cast<int>(Attrs::ArticulatedObjectInertiaSource::Computed));
    CORRADE_VERIFY(!artObjInstance->getApplyScaleToMass());
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getLinkOrder()),
        static_cast<int>(Attrs::ArticulatedObjectLinkOrder::TreeTraversal));
    CORRADE_COMPARE(static_cast<int>(artObjInstance->getRenderMode()),
                    static_cast<int>(Attrs::ArticulatedObjectRenderMode::Both));
    CORRADE_VERIFY(artObjInstance->getAutoClampJointLimits());
    CORRADE_COMPARE(artObjInstance->getTranslation(), Mn::Vector3(3, 2, 1));
    CORRADE_COMPARE(static_cast<int>(artObjInstance->getMotionType()),
                    static_cast<int>(esp::physics::MotionType::KINEMATIC));
    // test test_urdf_template0 ao instance attributes-level user config vals
    testUserDefinedConfigVals(artObjInstance->getUserConfiguration(), 4,
                              "test_urdf_template1 instance defined string",
                              false, 21, 11.22, Mn::Vector2(1.9f, 2.9f),
                              Mn::Vector3(190.3f, 902.5f, -95.07f),
                              Mn::Quaternion({9.22f, 9.26f, 0.21f}, 1.25f),
                              Mn::Vector4(13.5f, 4.6f, 25.7f, 76.9f));
  }
  {
    auto artObjInstance = artObjInstances[2];
    CORRADE_COMPARE(artObjInstance->getHandle(), "test_urdf_template2");
    // Nothing specified in instance,so use defaults
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getBaseType()),
        static_cast<int>(Attrs::ArticulatedObjectBaseType::Unspecified));
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getInertiaSource()),
        static_cast<int>(Attrs::ArticulatedObjectInertiaSource::Unspecified));
    CORRADE_VERIFY(artObjInstance->getApplyScaleToMass());
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getLinkOrder()),
        static_cast<int>(Attrs::ArticulatedObjectLinkOrder::Unspecified));
    CORRADE_COMPARE(
        static_cast<int>(artObjInstance->getRenderMode()),
        static_cast<int>(Attrs::ArticulatedObjectRenderMode::Unspecified));
    // Same as template 1
    CORRADE_VERIFY(artObjInstance->getAutoClampJointLimits());
    CORRADE_COMPARE(artObjInstance->getTranslation(), Mn::Vector3(3, 2, 1));
    CORRADE_COMPARE(static_cast<int>(artObjInstance->getMotionType()),
                    static_cast<int>(esp::physics::MotionType::KINEMATIC));
    // test test_urdf_template0 ao instance attributes-level user config vals
    testUserDefinedConfigVals(artObjInstance->getUserConfiguration(), 4,
                              "test_urdf_template1 instance defined string",
                              false, 21, 11.22, Mn::Vector2(1.9f, 2.9f),
                              Mn::Vector3(190.3f, 902.5f, -95.07f),
                              Mn::Quaternion({9.22f, 9.26f, 0.21f}, 1.25f),
                              Mn::Vector4(13.5f, 4.6f, 25.7f, 76.9f));
  }
  {
    // verify stage populated properly
    auto stageInstance = sceneAttr->getStageInstance();
    CORRADE_COMPARE(stageInstance->getHandle(), "test_stage_template");
    CORRADE_COMPARE(stageInstance->getTranslation(), Mn::Vector3(1, 2, 3));
    CORRADE_COMPARE(stageInstance->getRotation(),
                    Mn::Quaternion({0.2f, 0.3f, 0.4f}, 0.1f));
    // make sure that is not default value "flat"
    CORRADE_COMPARE(static_cast<int>(stageInstance->getShaderType()),
                    static_cast<int>(Attrs::ObjectInstanceShaderType::PBR));
    CORRADE_COMPARE(stageInstance->getUniformScale(), 1.9f);
    CORRADE_COMPARE(stageInstance->getNonUniformScale(),
                    Mn::Vector3(1.5f, 2.5f, 3.5f));

    // test stage instance attributes-level user config vals
    testUserDefinedConfigVals(
        stageInstance->getUserConfiguration(), 4,
        "stage instance defined string", true, 11, 2.2, Mn::Vector2(4.1f, 5.2f),
        Mn::Vector3(1.2, 3.4, 5.6), Mn::Quaternion({0.5f, 0.6f, 0.7f}, 0.4f),
        Mn::Vector4(3.5f, 4.6f, 5.7f, 6.9f));
  }
  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(sceneInstanceAttributesManager_,
                                        sceneAttr->getHandle());
}  // AttributesConfigsTest::testSceneInstanceAttrVals

void AttributesConfigsTest::testSceneInstanceJSONLoad() {
  // build JSON sample config
  const std::string& jsonString = R"({
  "translation_origin" : "Asset_Local",
  "stage_instance":{
      "template_name": "test_stage_template",
      "translation": [1,2,3],
      "rotation": [0.1, 0.2, 0.3, 0.4],
      "shader_type" : "pbr",
      "uniform_scale" : 1.9,
      "non_uniform_scale" : [1.5,2.5,3.5],
      "user_defined" : {
          "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
          "user_string" : "stage instance defined string",
          "user_bool" : true,
          "user_int" : 11,
          "user_double" : 2.2,
          "user_vec2" : [4.1, 5.2],
          "user_vec3" : [1.2, 3.4, 5.6],
          "user_quat" : [0.4, 0.5, 0.6, 0.7],
          "user_vec4" : [3.5, 4.6, 5.7, 6.9]
      }
  },
  "object_instances": [
      {
          "template_name": "test_object_template0",
          "translation_origin": "COM",
          "translation": [0,1,2],
          "rotation": [0.2, 0.3, 0.4, 0.5],
          "motion_type": "KINEMATIC",
          "apply_scale_to_mass": true,
          "uniform_scale" : 1.1,
          "non_uniform_scale" : [1.1,2.2,3.3],
          "user_defined" : {
              "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
              "user_string" : "obj0 instance defined string",
              "user_bool" : false,
              "user_int" : 12,
              "user_double" : 2.3,
              "user_vec2" : [1.6, 2.8],
              "user_vec3" : [1.3, 3.5, 5.7],
              "user_vec4" : [4.5, 3.6, 2.7, 1.9],
              "user_quat" : [0.3, 0.2, 0.6, 0.1]
          }
      },
      {
          "template_name": "test_object_template1",
          "translation": [0,-1,-2],
          "rotation": [0.5, 0.6, 0.7, 0.8],
          "motion_type": "DYNAMIC",
          "apply_scale_to_mass": false,
          "uniform_scale" : 2.1,
          "non_uniform_scale" : [2.1,3.2,4.3],
          "user_defined" : {
              "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
              "user_string" : "obj1 instance defined string",
              "user_bool" : false,
              "user_int" : 1,
              "user_double" : 1.1,
              "user_vec2" : [2.1, 3.2],
              "user_vec3" : [10.3, 30.5, -5.07],
              "user_vec4" : [4.5, 5.6, 6.7, 7.9],
              "user_quat" : [1.3, 1.2, 1.6, 1.1]
          }
      }
      ],
      "articulated_object_instances": [
          {
              "template_name": "test_urdf_template0",
              "translation_origin": "COM",
              "base_type" : "fixed",
              "inertia_source" : "urdf",
              "apply_scale_to_mass": true,
              "link_order" : "urdf_order",
              "render_mode": "link_visuals",
              "auto_clamp_joint_limits" : true,
              "translation": [5,4,5],
              "rotation": [0.2, 0.3, 0.4, 0.5],
              "initial_joint_pose": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
              "initial_joint_velocities": [1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6],
              "motion_type": "DYNAMIC",
              "user_defined" : {
                  "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
                  "user_string" : "test_urdf_template0 instance defined string",
                  "user_bool" : false,
                  "user_int" : 2,
                  "user_double" : 1.22,
                  "user_vec2" : [3.1, 4.2],
                  "user_vec3" : [120.3, 302.5, -25.07],
                  "user_vec4" : [13.5, 24.6, 35.7, 46.9],
                  "user_quat" : [1.23, 1.22, 1.26, 1.21],
                  "user_def_obj" : {
                      "position" : [0.1, 0.2, 0.3],
                      "rotation" : [0.5, 0.3, 0.1]
                  }
              }
          },
          {
              "template_name": "test_urdf_template1",
              "base_type" : "free",
              "inertia_source" : "computed",
              "apply_scale_to_mass": false,
              "link_order" : "tree_traversal",
              "render_mode": "both",
              "auto_clamp_joint_limits" : true,
              "translation": [3, 2, 1],
              "rotation": [0.5, 0.6, 0.7, 0.8],
              "motion_type": "KINEMATIC",
              "user_defined" : {
                  "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
                  "user_string" : "test_urdf_template1 instance defined string",
                  "user_bool" : false,
                  "user_int" : 21,
                  "user_double" : 11.22,
                  "user_vec2" : [1.9, 2.9],
                  "user_vec3" : [190.3, 902.5, -95.07],
                  "user_vec4" : [13.5, 4.6, 25.7, 76.9],
                  "user_quat" : [1.25, 9.22, 9.26, 0.21]
              }
          },
          {
              "template_name": "test_urdf_template2",
              "auto_clamp_joint_limits" : true,
              "translation": [3, 2, 1],
              "rotation": [0.5, 0.6, 0.7, 0.8],
              "motion_type": "KINEMATIC",
              "user_defined" : {
                  "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
                  "user_string" : "test_urdf_template1 instance defined string",
                  "user_bool" : false,
                  "user_int" : 21,
                  "user_double" : 11.22,
                  "user_vec2" : [1.9, 2.9],
                  "user_vec3" : [190.3, 902.5, -95.07],
                  "user_vec4" : [13.5, 4.6, 25.7, 76.9],
                  "user_quat" : [1.25, 9.22, 9.26, 0.21]
              }
          }
      ],
      "default_lighting":  "test_lighting_configuration",
      "navmesh_instance": "test_navmesh_path1",
      "semantic_scene_instance": "test_semantic_descriptor_path1",
      "user_defined" : {
          "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
          "user_string" : "scene instance defined string",
          "user_bool" : true,
          "user_int" : 99,
          "user_double" : 9.1,
          "user_vec2" : [1.3, 2.4],
          "user_vec3" : [12.3, 32.5, 25.07],
          "user_vec4" : [13.5, 14.6, 15.7, 16.9],
          "user_quat" : [0.3, 3.2, 2.6, 5.1]
      }
     })";

  // Build an attributes based on the above json string
  auto sceneAttr = sceneInstanceAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, true);
  // verify exists
  CORRADE_VERIFY(sceneAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testSceneAttrConfig_saved_JSON.{}", TEST_ASSETS,
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
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testSceneInstanceJSONLoad
void AttributesConfigsTest::testSemanticAttrVals(
    std::shared_ptr<Attrs::SemanticAttributes> semanticAttr,
    const std::string& assetPath) {
  CORRADE_COMPARE(
      semanticAttr->getSemanticDescriptorFullPath(),
      Cr::Utility::Path::join(assetPath, "test_semantic_lexicon.json"));

  CORRADE_COMPARE(
      semanticAttr->getSemanticAssetFullPath(),
      Cr::Utility::Path::join(assetPath, "test_semantic_asset.glb"));

  // verify regions
  auto regionInstanceList = semanticAttr->getRegionInstances();

  CORRADE_COMPARE(regionInstanceList.size(), 2);
  CORRADE_COMPARE(semanticAttr->getNumRegionInstances(), 2);

  {
    auto regionDesc = regionInstanceList[0];
    CORRADE_COMPARE(regionDesc->getHandle(), "bedroom.000");
    CORRADE_COMPARE(regionDesc->getLabel(), "bedroom");

    auto polyLoop = regionDesc->getPolyLoop();
    CORRADE_COMPARE(polyLoop.size(), 8);

    std::vector<Magnum::Vector3> testLoop{
        Mn::Vector3{-18.06, 0.0, -13.95}, Mn::Vector3{-15.95, 0.0, -13.95},
        Mn::Vector3{-18.06, 0.0, -12.18}, Mn::Vector3{-16.46, 0.0, -12.18},
        Mn::Vector3{-16.46, 0.0, -12.84}, Mn::Vector3{-13.17, 0.0, -12.84},
        Mn::Vector3{-13.17, 0.0, -16.33}, Mn::Vector3{-15.95, 0.0, -16.33},
    };

    for (int i = 0; i < polyLoop.size(); ++i) {
      CORRADE_COMPARE(polyLoop[i], testLoop[i]);
    }

    CORRADE_COMPARE(regionDesc->getFloorHeight(), 0.0);
    CORRADE_COMPARE(regionDesc->getExtrusionHeight(), 2.8);
    CORRADE_COMPARE(regionDesc->getMinBounds(),
                    Mn::Vector3(-18.06f, 0.0f, -16.33f));
    CORRADE_COMPARE(regionDesc->getMaxBounds(),
                    Mn::Vector3(-13.17, 2.8, -12.18));
  }
  {
    auto regionDesc = regionInstanceList[1];
    CORRADE_COMPARE(regionDesc->getHandle(), "bedroom.001");
    CORRADE_COMPARE(regionDesc->getLabel(), "bedroom");

    auto polyLoop = regionDesc->getPolyLoop();
    CORRADE_COMPARE(polyLoop.size(), 6);

    std::vector<Magnum::Vector3> testLoop{
        Mn::Vector3{-6.95, 0.1, -13.41}, Mn::Vector3{-6.95, 0.1, -8.75},
        Mn::Vector3{-5.47, 0.1, -8.75},  Mn::Vector3{-5.47, 0.1, -8.95},
        Mn::Vector3{-3.62, 0.1, -8.95},  Mn::Vector3{-3.62, 0.1, -13.41}};

    for (int i = 0; i < polyLoop.size(); ++i) {
      CORRADE_COMPARE(polyLoop[i], testLoop[i]);
    }

    CORRADE_COMPARE(regionDesc->getFloorHeight(), 0.1);
    CORRADE_COMPARE(regionDesc->getExtrusionHeight(), 2.9);
    CORRADE_COMPARE(regionDesc->getMinBounds(),
                    Mn::Vector3(-6.95f, 0.1f, -13.41f));
    CORRADE_COMPARE(regionDesc->getMaxBounds(),
                    Mn::Vector3(-3.62f, 2.9f, -8.75f));
  }

  testUserDefinedConfigVals(semanticAttr->getUserConfiguration(), 4,
                            "semantic attribs defined string", true, 273, 7.3,
                            Mn::Vector2(7.1f, 7.2f), Mn::Vector3(7.1, 3.3, 5.5),
                            Mn::Quaternion({0.6f, 0.7f, 0.8f}, 7.5f),
                            Mn::Vector4(7.5f, 1.6f, 1.7f, 1.9f));

  //
}  // AttributesConfigsTest::testSemanticAttrVals

void AttributesConfigsTest::testSemanticJSONLoad() {
  const std::string& jsonString =
      R"({
      "semantic_descriptor_filename": "test_semantic_lexicon.json",
      "semantic_asset": "test_semantic_asset.glb",
      "region_annotations": [
        {
          "name": "bedroom.000",
          "label": "bedroom",
          "poly_loop": [
            [-18.06, 0.0,-13.95],
            [-15.95, 0.0,-13.95],
            [-18.06, 0.0,-12.18],
            [-16.46, 0.0,-12.18],
            [-16.46, 0.0,-12.84],
            [-13.17, 0.0,-12.84],
            [-13.17, 0.0,-16.33],
            [-15.95, 0.0,-16.33]
          ],
          "floor_height": 0.0,
          "extrusion_height": 2.8,
          "min_bounds": [-18.06, 0.0, -16.33],
          "max_bounds": [-13.17, 2.8, -12.18]
        },
        {
          "name": "bedroom.001",
          "label": "bedroom",
          "poly_loop": [
            [-6.95, 0.1,-13.41],
            [-6.95, 0.1,-8.75],
            [-5.47, 0.1,-8.75],
            [-5.47, 0.1,-8.95],
            [-3.62, 0.1,-8.95],
            [-3.62, 0.1,-13.41]
          ],
          "floor_height": 0.1,
          "extrusion_height": 2.9,
          "min_bounds": [-6.95, 0.1, -13.41],
          "max_bounds": [-3.62, 2.9, -8.75]
        }
      ],
      "user_defined" : {
        "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
        "user_string" : "semantic attribs defined string",
        "user_bool" : true,
        "user_int" : 273,
        "user_double" : 7.3,
        "user_vec2" : [7.1, 7.2],
        "user_vec3" : [7.1, 3.3, 5.5],
        "user_quat" : [7.5, 0.6, 0.7, 0.8],
        "user_vec4" : [7.5, 1.6, 1.7, 1.9]
      }
    })";
  // Build an attributes based on the above json string
  auto semanticAttr = semanticAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, true);
  // verify exists
  CORRADE_VERIFY(semanticAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testsemanticAttrConfig_saved_JSON.{}", TEST_ASSETS,
      semanticAttributesManager_->getJSONTypeExt());

  bool success = semanticAttributesManager_->saveManagedObjectToFile(
      semanticAttr->getHandle(), newAttrName);

  ESP_DEBUG() << "About to test string-based semanticAttr";
  // test json string to verify format
  testSemanticAttrVals(semanticAttr, "");
  ESP_DEBUG() << "Tested semanticAttr";
  semanticAttr = nullptr;

  // load attributes from new name and retest
  auto semanticAttr2 =
      semanticAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(semanticAttr2);

  // test json string to verify format, this deletes semanticAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved semanticAttr2 :"
              << semanticAttr2->getHandle();
  testSemanticAttrVals(semanticAttr2, TEST_ASSETS);
  ESP_DEBUG() << "About to test semanticAttr2";

  // delete file-based config
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testSemanticJSONLoad

#ifdef ESP_BUILD_WITH_AUDIO
void AttributesConfigsTest::testAudioSensorAttrVals(
    std::shared_ptr<Attrs::AudioSensorAttributes> audioAttr,
    const std::string& assetPath) {
  // match values to expected values in JSON or spec
  CORRADE_COMPARE(audioAttr->getHandle(), "audio_sensor_handle_:0000");
  CORRADE_COMPARE(static_cast<int>(audioAttr->getSensorType()),
                  static_cast<int>(esp::sensor::SensorType::Audio));
  CORRADE_COMPARE(
      static_cast<int>(audioAttr->getSensorSubType()),
      static_cast<int>(esp::sensor::SensorSubType::ImpulseResponse));
  CORRADE_COMPARE(audioAttr->getPosition(), Mn::Vector3(1.5, 2.5, 3.5));
  CORRADE_COMPARE(audioAttr->getOrientation(), Mn::Vector3(1.0, 2.0, 3.0));
  CORRADE_COMPARE(audioAttr->getNoiseModel(), "audio_noise");
  CORRADE_COMPARE(audioAttr->getOutputDirectory(), "audio_sensor_output_dir");
  // TODO :
  // Support user_defined attributes once being read from JSON
}  // AttributesConfigsTest::testAudioSensorAttrVals
#endif  // ESP_BUILD_WITH_AUDIO

void AttributesConfigsTest::testCameraSensorAttrVals(
    std::shared_ptr<Attrs::CameraSensorAttributes> cameraAttr,
    const std::string& assetPath) {
  // base SensorSpec values
  CORRADE_COMPARE(cameraAttr->getHandle(), "camera_sensor_handle_:0000");
  CORRADE_COMPARE(static_cast<int>(cameraAttr->getSensorType()),
                  static_cast<int>(esp::sensor::SensorType::Color));
  CORRADE_COMPARE(static_cast<int>(cameraAttr->getSensorSubType()),
                  static_cast<int>(esp::sensor::SensorSubType::Pinhole));
  CORRADE_COMPARE(cameraAttr->getPosition(), Mn::Vector3(2.1, 3.2, 4.3));
  CORRADE_COMPARE(cameraAttr->getOrientation(), Mn::Vector3(4.0, 3.0, 2.0));
  CORRADE_COMPARE(cameraAttr->getNoiseModel(), "pinhole_camera_noise");
  // visual SensorSpec values
  CORRADE_COMPARE(cameraAttr->getResolution(), Mn::Vector2i(256, 512));
  CORRADE_COMPARE(cameraAttr->getChannels(), 3);
  CORRADE_COMPARE(cameraAttr->getGPUToGPUTransfer(), true);
  CORRADE_COMPARE(cameraAttr->getNearPlane(), 0.1f);
  CORRADE_COMPARE(cameraAttr->getFarPlane(), 2000.0f);
  // camera SensorSpec values
  CORRADE_COMPARE(cameraAttr->getOrthoScale(), 0.2f);
  CORRADE_COMPARE(cameraAttr->getHFOV(), 120.0_degf);
  // TODO :
  // Support user_defined attributes once being read from JSON
}  // AttributesConfigsTest::testCameraSensorAttrVals

void AttributesConfigsTest::testCustomSensorAttrVals(
    std::shared_ptr<Attrs::CustomSensorAttributes> customAttr,
    const std::string& assetPath) {
  // base SensorSpec values
  CORRADE_COMPARE(customAttr->getHandle(), "custom_sensor_handle_:0000");
  CORRADE_COMPARE(static_cast<int>(customAttr->getSensorType()),
                  static_cast<int>(esp::sensor::SensorType::Custom));
  CORRADE_COMPARE(static_cast<int>(customAttr->getSensorSubType()),
                  static_cast<int>(esp::sensor::SensorSubType::Custom));
  CORRADE_COMPARE(customAttr->getPosition(), Mn::Vector3(1.5, 2.5, 3.5));
  CORRADE_COMPARE(customAttr->getOrientation(), Mn::Vector3(1.0, 2.0, 3.0));
  CORRADE_COMPARE(customAttr->getNoiseModel(), "custom_noise");

  // TODO
  // Custom sensor specifying attributes only available from JSON load.

  // TODO :
  // Support user_defined attributes once being read from JSON
}  // AttributesConfigsTest::testCustomSensorAttrVals

void AttributesConfigsTest::testEquirectangularSensorAttrVals(
    std::shared_ptr<Attrs::EquirectangularSensorAttributes> equirectAttr,
    const std::string& assetPath) {
  // base SensorSpec values
  CORRADE_COMPARE(equirectAttr->getHandle(), "equirect_sensor_handle_:0000");
  CORRADE_COMPARE(static_cast<int>(equirectAttr->getSensorType()),
                  static_cast<int>(esp::sensor::SensorType::Depth));
  CORRADE_COMPARE(
      static_cast<int>(equirectAttr->getSensorSubType()),
      static_cast<int>(esp::sensor::SensorSubType::Equirectangular));
  CORRADE_COMPARE(equirectAttr->getPosition(), Mn::Vector3(1.1, 2.2, 3.3));
  CORRADE_COMPARE(equirectAttr->getOrientation(),
                  Mn::Vector3(20.0, 10.0, 30.0));
  CORRADE_COMPARE(equirectAttr->getNoiseModel(), "equirect_camera_noise");
  // visual SensorSpec values
  CORRADE_COMPARE(equirectAttr->getResolution(), Mn::Vector2i(300, 400));
  CORRADE_COMPARE(equirectAttr->getChannels(), 1);
  CORRADE_COMPARE(equirectAttr->getGPUToGPUTransfer(), true);
  CORRADE_COMPARE(equirectAttr->getNearPlane(), 0.7f);
  CORRADE_COMPARE(equirectAttr->getFarPlane(), 1200.0f);
  // cubemap sensor spec values
  CORRADE_COMPARE(equirectAttr->getCubeMapSize(), 320);
  CORRADE_COMPARE(equirectAttr->getCubeMapSizeToUse(), 320);

  // TODO :
  // Support user_defined attributes once being read from JSON
}  // AttributesConfigsTest::testEquirectangularSensorAttrVals

void AttributesConfigsTest::testFisheyeSensorAttrVals(
    std::shared_ptr<Attrs::FisheyeSensorAttributes> fisheyeAttr,
    const std::string& assetPath) {
  // base SensorSpec values
  CORRADE_COMPARE(fisheyeAttr->getHandle(), "fisheye_sensor_handle_:0000");
  CORRADE_COMPARE(static_cast<int>(fisheyeAttr->getSensorType()),
                  static_cast<int>(esp::sensor::SensorType::Semantic));
  CORRADE_COMPARE(static_cast<int>(fisheyeAttr->getSensorSubType()),
                  static_cast<int>(esp::sensor::SensorSubType::Fisheye));
  CORRADE_COMPARE(fisheyeAttr->getPosition(), Mn::Vector3(2.1, 3.2, 4.3));
  CORRADE_COMPARE(fisheyeAttr->getOrientation(), Mn::Vector3(40.0, 34.0, 21.0));
  CORRADE_COMPARE(fisheyeAttr->getNoiseModel(), "fisheye_camera_noise");
  // visual SensorSpec values
  CORRADE_COMPARE(fisheyeAttr->getResolution(), Mn::Vector2i(500, 300));
  CORRADE_COMPARE(fisheyeAttr->getChannels(), 3);
  CORRADE_COMPARE(fisheyeAttr->getGPUToGPUTransfer(), true);
  CORRADE_COMPARE(fisheyeAttr->getNearPlane(), 0.05f);
  CORRADE_COMPARE(fisheyeAttr->getFarPlane(), 1500.0f);
  // cubemap SensorSpec values
  CORRADE_COMPARE(fisheyeAttr->getCubeMapSize(), 0);
  CORRADE_COMPARE(fisheyeAttr->getCubeMapSizeToUse(), 300);
  // fisheye SensorSpec values

  CORRADE_COMPARE(fisheyeAttr->getFocalLength(), Mn::Vector2(5.1, 2.5));
  CORRADE_COMPARE(fisheyeAttr->getPrincipalPointOffset(),
                  Mn::Vector2(200, 100));
  CORRADE_COMPARE(fisheyeAttr->getPrincipalPointOffsetToUse(),
                  Mn::Vector2(200, 100));

  // TODO :
  // Support user_defined attributes once being read from JSON
}  // AttributesConfigsTest::testFisheyeSensorAttrVals

void AttributesConfigsTest::testSensorAttrSpecLoad() {
  // add dummy test so that test will run
  CORRADE_VERIFY(true);
  // to be removed once specs are removed
  // build one of each of 5 types of SensorAttributes, driven by data values,
  // from the appropriate SensorSpec, and then test their values.

#ifdef ESP_BUILD_WITH_AUDIO
  {
    AudioSensorSpec::ptr audioSpec = AudioSensorSpec::create();
    // base SensorSpec values
    audioSpec->uuid = "audio_sensor_handle";
    audioSpec->sensorType = esp::sensor::SensorType::Audio;
    audioSpec->sensorSubType = esp::sensor::SensorSubType::ImpulseResponse;
    audioSpec->position = {1.5, 2.5, 3.5};
    audioSpec->orientation = {1.0, 2.0, 3.0};
    audioSpec->noiseModel = "audio_noise";
    audioSpec->outputDirectory_ = "audio_sensor_output_dir";

    // build audio attributes - data should direct manager to build correct
    // attributes type
    AudioSensorAttributes::ptr audioAttr =
        sensorAttributesManager_
            ->createAttributesFromSensorSpec<AudioSensorAttributes>(audioSpec,
                                                                    true);
    CORRADE_VERIFY(audioAttr);
    // Verify attributes are correct class
    CORRADE_COMPARE(audioAttr->getClassKey(), "AudioSensorAttributes");

    // test audio attributes' values
    testAudioSensorAttrVals(audioAttr, "");
  }
#endif  // ESP_BUILD_WITH_AUDIO
  {
    CameraSensorSpec::ptr cameraSpec = CameraSensorSpec::create();
    // base SensorSpec values
    cameraSpec->uuid = "camera_sensor_handle";
    cameraSpec->sensorType = esp::sensor::SensorType::Color;
    cameraSpec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
    cameraSpec->position = {2.1, 3.2, 4.3};
    cameraSpec->orientation = {4.0, 3.0, 2.0};
    cameraSpec->noiseModel = "pinhole_camera_noise";
    // visual SensorSpec values
    cameraSpec->resolution = Mn::Vector2i(256, 512);
    cameraSpec->channels = 3;
    cameraSpec->gpu2gpuTransfer = true;
    cameraSpec->near = 0.1f;
    cameraSpec->far = 2000.0f;
    // camera SensorSpec values
    cameraSpec->orthoScale = 0.2f;
    cameraSpec->hfov = 120.0_degf;

    // build camera attributes - data should direct manager to build correct
    // attributes type
    CameraSensorAttributes::ptr cameraAttr =
        sensorAttributesManager_
            ->createAttributesFromSensorSpec<CameraSensorAttributes>(cameraSpec,
                                                                     true);
    CORRADE_VERIFY(cameraAttr);
    // Verify attributes are correct class
    CORRADE_COMPARE(cameraAttr->getClassKey(), "CameraSensorAttributes");

    // test camera attributes' values
    testCameraSensorAttrVals(cameraAttr, "");
  }
  {
    SensorSpec::ptr customSpec = SensorSpec::create();
    // base SensorSpec values
    customSpec->uuid = "custom_sensor_handle";
    customSpec->sensorType = esp::sensor::SensorType::Custom;
    customSpec->sensorSubType = esp::sensor::SensorSubType::Custom;
    customSpec->position = {1.5, 2.5, 3.5};
    customSpec->orientation = {1.0, 2.0, 3.0};
    customSpec->noiseModel = "custom_noise";

    // build custom attributes - data should direct manager to build correct
    // attributes type

    CustomSensorAttributes::ptr custAttr =
        sensorAttributesManager_
            ->createAttributesFromSensorSpec<CustomSensorAttributes>(customSpec,
                                                                     true);
    CORRADE_VERIFY(custAttr);
    // Verify attributes are correct class
    CORRADE_COMPARE(custAttr->getClassKey(), "CustomSensorAttributes");

    // test custom attributes' values
    testCustomSensorAttrVals(custAttr, "");
  }
  {
    EquirectangularSensorSpec::ptr equirectSpec =
        EquirectangularSensorSpec::create();
    // base SensorSpec values
    equirectSpec->uuid = "equirect_sensor_handle";
    equirectSpec->sensorType = esp::sensor::SensorType::Depth;
    equirectSpec->sensorSubType = esp::sensor::SensorSubType::Equirectangular;
    equirectSpec->position = {1.1, 2.2, 3.3};
    equirectSpec->orientation = {20.0, 10.0, 30.0};
    equirectSpec->noiseModel = "equirect_camera_noise";
    // visual SensorSpec values
    equirectSpec->resolution = Mn::Vector2i(300, 400);
    equirectSpec->channels = 1;
    equirectSpec->gpu2gpuTransfer = true;
    equirectSpec->near = 0.7f;
    equirectSpec->far = 1200.0f;
    // cubemap sensor spec values
    equirectSpec->cubemapSize = 320;
    // No further sensorSpec vaues for equirect

    // build equirect attributes - data should direct manager to build correct
    // attributes type
    EquirectangularSensorAttributes::ptr equiAttr =
        sensorAttributesManager_
            ->createAttributesFromSensorSpec<EquirectangularSensorAttributes>(
                equirectSpec, true);

    CORRADE_VERIFY(equiAttr);
    // Verify attributes are correct class
    CORRADE_COMPARE(equiAttr->getClassKey(), "EquirectangularSensorAttributes");

    // test equirect attributes' values
    testEquirectangularSensorAttrVals(equiAttr, "");
  }
  {
    FisheyeSensorDoubleSphereSpec::ptr fisheyeSpec =
        FisheyeSensorDoubleSphereSpec::create();
    // base SensorSpec values
    fisheyeSpec->uuid = "fisheye_sensor_handle";
    fisheyeSpec->sensorType = esp::sensor::SensorType::Semantic;
    fisheyeSpec->sensorSubType = esp::sensor::SensorSubType::Fisheye;
    fisheyeSpec->position = {2.1, 3.2, 4.3};
    fisheyeSpec->orientation = {40.0, 34.0, 21.0};
    fisheyeSpec->noiseModel = "fisheye_camera_noise";
    // visual SensorSpec values
    fisheyeSpec->resolution = Mn::Vector2i(500, 300);
    fisheyeSpec->channels = 3;
    fisheyeSpec->gpu2gpuTransfer = true;
    fisheyeSpec->near = 0.05f;
    fisheyeSpec->far = 1500.0f;
    // cubemap SensorSpec values
    fisheyeSpec->cubemapSize = Cr::Containers::NullOpt;
    // fisheye SensorSpec values
    fisheyeSpec->fisheyeModelType =
        esp::sensor::FisheyeSensorModelType::DoubleSphere;
    fisheyeSpec->focalLength = Mn::Vector2(5.1, 2.5);
    fisheyeSpec->principalPointOffset = Mn::Vector2(200, 100);
    // build Fisheye attributes - data should direct manager to build correct
    // attributes type
    FisheyeSensorAttributes::ptr fisheyeAttr =
        sensorAttributesManager_
            ->createAttributesFromSensorSpec<FisheyeSensorAttributes>(
                fisheyeSpec, true);

    CORRADE_VERIFY(fisheyeAttr);
    // Verify attributes are correct class
    CORRADE_COMPARE(fisheyeAttr->getClassKey(), "FisheyeSensorAttributes");

    // test Fisheye attributes' values
    testFisheyeSensorAttrVals(fisheyeAttr, "");
  }
}  // AttributesConfigsTest::testSensorAttrSpecLoad
void AttributesConfigsTest::testSensorAttrJSONLoad() {
  // add dummy test so that test will run
  CORRADE_VERIFY(true);
}  // AttributesConfigsTest::testSensorAttrJSONLoad

void AttributesConfigsTest::testStageAttrVals(
    std::shared_ptr<Attrs::StageAttributes> stageAttr,
    const std::string& assetPath) {
  // match values set in test JSON
  CORRADE_COMPARE(stageAttr->getScale(), Mn::Vector3(2, 3, 4));
  CORRADE_COMPARE(stageAttr->getMargin(), 0.9);
  CORRADE_COMPARE(stageAttr->getFrictionCoefficient(), 0.321);
  CORRADE_COMPARE(stageAttr->getRestitutionCoefficient(), 0.456);
  CORRADE_VERIFY(!stageAttr->getForceFlatShading());
  CORRADE_COMPARE(stageAttr->getUnitsToMeters(), 1.1);
  CORRADE_COMPARE(stageAttr->getOrientUp(), Mn::Vector3(2.1, 0, 0));
  CORRADE_COMPARE(stageAttr->getOrientFront(), Mn::Vector3(0, 2.1, 0));

  // verify that we are set to not use the render asset frame for semantic
  // meshes.
  CORRADE_VERIFY(!stageAttr->getUseFrameForAllOrientation());
  CORRADE_COMPARE(stageAttr->getSemanticOrientFront(),
                  Mn::Vector3(2.0, 0.0, 0.0));
  CORRADE_COMPARE(stageAttr->getSemanticOrientUp(), Mn::Vector3(0.0, 2.0, 0.0));

  CORRADE_COMPARE(stageAttr->getRenderAssetFullPath(), assetPath);
  CORRADE_COMPARE(stageAttr->getCollisionAssetFullPath(), assetPath);
  CORRADE_VERIFY(!stageAttr->getIsCollidable());
  // stage-specific attributes
  CORRADE_COMPARE(stageAttr->getOrigin(), Mn::Vector3(1, 2, 3));
  CORRADE_COMPARE(stageAttr->getGravity(), Mn::Vector3(9, 8, 7));
  CORRADE_VERIFY(stageAttr->getHasSemanticTextures());

  // make sure that is not default value "flat"
  CORRADE_COMPARE(static_cast<int>(stageAttr->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::Material));
  CORRADE_COMPARE(stageAttr->getSemanticAssetFullPath(), assetPath);
  CORRADE_COMPARE(stageAttr->getNavmeshAssetFullPath(), assetPath);
  // test stage attributes-level user config vals
  testUserDefinedConfigVals(
      stageAttr->getUserConfiguration(), 4, "stage defined string", false, 3,
      0.8, Mn::Vector2(2.3f, 4.5f), Mn::Vector3(5.4, 7.6, 10.1),
      Mn::Quaternion({1.5f, 2.6f, 3.7f}, 0.1f),
      Mn::Vector4(14.5f, 15.6f, 16.7f, 17.9f));

  {
    // Test marker sets

    MarkerSetTestMap marker_set_000_stage_name(
        {{"003", Mn::Vector3{4.1, 5.2, 6.3}},
         {"002", Mn::Vector3{3.1, 4.2, 5.3}},
         {"001", Mn::Vector3{2.1, 3.2, 4.3}},
         {"000", Mn::Vector3{1.1, 2.2, 3.3}}});

    MarkerSetTestMap marker_set_001_stage_name(
        {{"003", Mn::Vector3{4.2, 5.3, 6.4}},
         {"002", Mn::Vector3{3.2, 4.3, 5.4}},
         {"001", Mn::Vector3{2.2, 3.3, 4.4}},
         {"000", Mn::Vector3{1.2, 2.3, 3.4}}});

    LinkSetTestMap link_set_00_stage_name(
        {{"marker_set_000_stage_name", marker_set_000_stage_name},
         {"marker_set_001_stage_name", marker_set_001_stage_name}});

    TaskSetTestMap task_set_0_stage_name{
        {{"link_set_00_stage_name", link_set_00_stage_name}}};

    MarkerSetTestMap marker_set_100_stage_name(
        {{"003", Mn::Vector3{14.1, 15.2, 16.3}},
         {"002", Mn::Vector3{13.1, 14.2, 15.3}},
         {"001", Mn::Vector3{12.1, 13.2, 14.3}},
         {"000", Mn::Vector3{11.1, 12.2, 13.3}}});

    MarkerSetTestMap marker_set_101_stage_name(
        {{"003", Mn::Vector3{14.2, 15.3, 16.4}},
         {"002", Mn::Vector3{13.2, 14.3, 15.4}},
         {"001", Mn::Vector3{12.2, 13.3, 14.4}},
         {"000", Mn::Vector3{11.2, 12.3, 13.4}}});

    MarkerSetTestMap marker_set_102_stage_name(
        {{"003", Mn::Vector3{124.2, 125.3, 126.4}},
         {"002", Mn::Vector3{123.2, 124.3, 125.4}},
         {"001", Mn::Vector3{122.2, 123.3, 124.4}},
         {"000", Mn::Vector3{121.2, 122.3, 123.4}}});

    LinkSetTestMap link_set_10_stage_name(
        {{"marker_set_100_stage_name", marker_set_100_stage_name},
         {"marker_set_101_stage_name", marker_set_101_stage_name},
         {"marker_set_102_stage_name", marker_set_102_stage_name}});

    MarkerSetTestMap marker_set_110_stage_name(
        {{"003", Mn::Vector3{14.3, 15.4, 16.5}},
         {"002", Mn::Vector3{13.3, 14.4, 15.5}},
         {"001", Mn::Vector3{12.3, 13.4, 14.5}},
         {"000", Mn::Vector3{11.3, 12.4, 13.5}}});

    MarkerSetTestMap marker_set_111_stage_name(
        {{"003", Mn::Vector3{14.4, 15.5, 16.6}},
         {"002", Mn::Vector3{13.4, 14.5, 15.6}},
         {"004", Mn::Vector3{15.4, 16.5, 17.6}},
         {"001", Mn::Vector3{12.4, 13.5, 14.6}},
         {"000", Mn::Vector3{11.4, 12.5, 13.6}}});

    LinkSetTestMap link_set_11_stage_name(
        {{"marker_set_110_stage_name", marker_set_110_stage_name},
         {"marker_set_110_stage_name", marker_set_110_stage_name}});

    TaskSetTestMap task_set_1_stage_name(
        {{"link_set_10_stage_name", link_set_10_stage_name},
         {"link_set_11_stage_name", link_set_11_stage_name}});

    AllMarkerSetsTestMap markerSetMap(
        {{"task_set_0_stage_name", task_set_0_stage_name},
         {"task_set_1_stage_name", task_set_1_stage_name}});

    testMarkerSetsConfigVals(stageAttr->getMarkerSetsConfiguration(),
                             markerSetMap);
  }
  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(stageAttributesManager_,
                                        stageAttr->getHandle());
}  // AttributesConfigsTest::testStageAttrVals
void AttributesConfigsTest::testStageJSONLoad() {
  // build JSON sample config
  const std::string& jsonString = R"({
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
  "marker_sets" : {
      "task_set_0_stage_name" : {
          "link_set_00_stage_name" : {
              "marker_set_000_stage_name" : {
                "markers" : [
                    [1.1, 2.2, 3.3],
                    [2.1, 3.2, 4.3],
                    [3.1, 4.2, 5.3],
                    [4.1, 5.2, 6.3]
                  ]
              },
              "marker_set_001_stage_name" : {
                "markers" : {
                    "0":[1.2, 2.3, 3.4],
                    "1":[2.2, 3.3, 4.4],
                    "2":[3.2, 4.3, 5.4],
                    "3":[4.2, 5.3, 6.4]
                }
              }
          }
      },
      "task_set_1_stage_name" : {
          "link_set_10_stage_name" : {
              "marker_set_100_stage_name" : {
                "markers" : [
                    [11.1, 12.2, 13.3],
                    [12.1, 13.2, 14.3],
                    [13.1, 14.2, 15.3],
                    [14.1, 15.2, 16.3]
                  ]
              },
              "marker_set_101_stage_name" : {
                "markers" : {
                    "0":[11.2, 12.3, 13.4],
                    "1":[12.2, 13.3, 14.4],
                    "2":[13.2, 14.3, 15.4],
                    "3":[14.2, 15.3, 16.4]
                }
              },
              "marker_set_102_stage_name" : {
                "markers" : {
                    "0":[121.2, 122.3, 123.4],
                    "1":[122.2, 123.3, 124.4],
                    "2":[123.2, 124.3, 125.4],
                    "3":[124.2, 125.3, 126.4]
                }
              }
          },
          "link_set_11_stage_name" : {
              "marker_set_110_stage_name" : {
                "markers" : {
                    "0":[11.3, 12.4, 13.5],
                    "1":[12.3, 13.4, 14.5],
                    "2":[13.3, 14.4, 15.5],
                    "3":[14.3, 15.4, 16.5]
                }
              },
              "marker_set_111_stage_name" : {
                "markers" : [
                    [11.4, 12.5, 13.6],
                    [12.4, 13.5, 14.6],
                    [13.4, 14.5, 15.6],
                    [14.4, 15.5, 16.6],
                    [15.4, 16.5, 17.6]
                  ]
              }
          }
      }
  },
  "user_defined" : {
      "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
      "user_string" : "stage defined string",
      "user_bool" : false,
      "user_int" : 3,
      "user_double" : 0.8,
      "user_vec2" : [2.3, 4.5],
      "user_vec3" : [5.4, 7.6, 10.1],
      "user_vec4" : [14.5, 15.6, 16.7, 17.9],
      "user_quat" : [0.1, 1.5, 2.6, 3.7]
  }
})";

  // Build an attributes based on the above json string
  // Don't register - registration here verifies that the specified file
  // handles in the attributes exist, or it will fail.
  auto stageAttr = stageAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, false);
  // verify exists
  CORRADE_VERIFY(stageAttr);
  // verify that we are set to use the render asset frame for all meshes.
  CORRADE_VERIFY(stageAttr->getUseFrameForAllOrientation());
  // set new frame for semantic assets to test functionality
  stageAttr->setSemanticOrientFront(Mn::Vector3(2.0, 0.0, 0.0));
  stageAttr->setSemanticOrientUp(Mn::Vector3(0.0, 2.0, 0.0));
  // verify that we are now set to not use the render asset frame for semantic
  // meshes.
  CORRADE_VERIFY(!stageAttr->getUseFrameForAllOrientation());

  // now need to change the render and collision assets to make sure they are
  // legal so test can proceed (needs to be actual existing file)
  const std::string stageAssetFile =
      Cr::Utility::Path::join(TEST_ASSETS, "scenes/plane.glb");

  stageAttr->setRenderAssetHandle(stageAssetFile);
  stageAttr->setCollisionAssetHandle(stageAssetFile);
  stageAttr->setSemanticAssetHandle(stageAssetFile);
  stageAttr->setNavmeshAssetHandle(stageAssetFile);
  // now register so can be saved to disk
  stageAttributesManager_->registerObject(stageAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testStageAttrConfig_saved_JSON.{}", TEST_ASSETS,
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
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testStageJSONLoad(

void AttributesConfigsTest::testObjectAttrVals(
    std::shared_ptr<Attrs::ObjectAttributes> objAttr,
    const std::string& assetPath) {
  // match values set in test JSON

  CORRADE_COMPARE(objAttr->getScale(), Mn::Vector3(2, 3, 4));
  CORRADE_COMPARE(objAttr->getMargin(), 0.9);
  CORRADE_COMPARE(objAttr->getFrictionCoefficient(), 0.321);
  CORRADE_COMPARE(objAttr->getRestitutionCoefficient(), 0.456);
  CORRADE_VERIFY(objAttr->getForceFlatShading());
  CORRADE_COMPARE(objAttr->getUnitsToMeters(), 1.1);
  CORRADE_COMPARE(objAttr->getOrientUp(), Mn::Vector3(2.1, 0, 0));
  CORRADE_COMPARE(objAttr->getOrientFront(), Mn::Vector3(0, 2.1, 0));
  CORRADE_COMPARE(objAttr->getRenderAssetFullPath(), assetPath);
  CORRADE_COMPARE(objAttr->getCollisionAssetFullPath(), assetPath);
  CORRADE_VERIFY(!objAttr->getIsCollidable());
  CORRADE_COMPARE(objAttr->getSemanticId(), 7);
  // object-specific attributes
  CORRADE_COMPARE(objAttr->getMass(), 9);
  CORRADE_COMPARE(static_cast<int>(objAttr->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::Phong));
  CORRADE_VERIFY(objAttr->getBoundingBoxCollisions());
  CORRADE_VERIFY(objAttr->getJoinCollisionMeshes());
  CORRADE_COMPARE(objAttr->getInertia(), Mn::Vector3(1.1, 0.9, 0.3));
  CORRADE_COMPARE(objAttr->getCOM(), Mn::Vector3(0.1, 0.2, 0.3));
  // test object attributes-level user config vals
  testUserDefinedConfigVals(
      objAttr->getUserConfiguration(), 4, "object defined string", true, 5, 2.6,
      Mn::Vector2(4.1f, 2.8f), Mn::Vector3(15.4, 17.6, 110.1),
      Mn::Quaternion({5.5f, 6.6f, 7.7f}, 0.7f),
      Mn::Vector4(1.5f, 1.6f, 6.7f, 7.9f));

  {
    // Test marker sets

    MarkerSetTestMap marker_set_000_obj_name(
        {{"003", Mn::Vector3{4.1, 5.2, 6.3}},
         {"002", Mn::Vector3{3.1, 4.2, 5.3}},
         {"001", Mn::Vector3{2.1, 3.2, 4.3}},
         {"000", Mn::Vector3{1.1, 2.2, 3.3}}});

    MarkerSetTestMap marker_set_001_obj_name(
        {{"003", Mn::Vector3{4.2, 5.3, 6.4}},
         {"002", Mn::Vector3{3.2, 4.3, 5.4}},
         {"001", Mn::Vector3{2.2, 3.3, 4.4}},
         {"000", Mn::Vector3{1.2, 2.3, 3.4}}});

    LinkSetTestMap link_set_00_obj_name(
        {{"marker_set_000_obj_name", marker_set_000_obj_name},
         {"marker_set_001_obj_name", marker_set_001_obj_name}});

    MarkerSetTestMap marker_set_010_obj_name(
        {{"003", Mn::Vector3{4.3, 5.4, 6.5}},
         {"002", Mn::Vector3{3.3, 4.4, 5.5}},
         {"001", Mn::Vector3{2.3, 3.4, 4.5}},
         {"000", Mn::Vector3{1.3, 2.4, 3.5}}});

    MarkerSetTestMap marker_set_011_obj_name(
        {{"003", Mn::Vector3{4.4, 5.5, 6.6}},
         {"002", Mn::Vector3{3.4, 4.5, 5.6}},
         {"001", Mn::Vector3{2.4, 3.5, 4.6}},
         {"000", Mn::Vector3{1.4, 2.5, 3.6}}});

    LinkSetTestMap link_set_01_obj_name(
        {{"marker_set_010_obj_name", marker_set_010_obj_name},
         {"marker_set_011_obj_name", marker_set_011_obj_name}});

    TaskSetTestMap task_set_0_obj_name{
        {{"link_set_00_obj_name", link_set_00_obj_name},
         {"link_set_01_obj_name", link_set_01_obj_name}}};

    MarkerSetTestMap marker_set_100_obj_name(
        {{"003", Mn::Vector3{14.1, 15.2, 16.3}},
         {"002", Mn::Vector3{13.1, 14.2, 15.3}},
         {"001", Mn::Vector3{12.1, 13.2, 14.3}},
         {"000", Mn::Vector3{11.1, 12.2, 13.3}}});

    MarkerSetTestMap marker_set_101_obj_name(
        {{"003", Mn::Vector3{14.2, 15.3, 16.4}},
         {"002", Mn::Vector3{13.2, 14.3, 15.4}},
         {"001", Mn::Vector3{12.2, 13.3, 14.4}},
         {"000", Mn::Vector3{11.2, 12.3, 13.4}}});

    LinkSetTestMap link_set_10_obj_name(
        {{"marker_set_100_obj_name", marker_set_100_obj_name},
         {"marker_set_101_obj_name", marker_set_101_obj_name}});

    MarkerSetTestMap marker_set_110_obj_name(
        {{"003", Mn::Vector3{14.3, 15.4, 16.5}},
         {"002", Mn::Vector3{13.3, 14.4, 15.5}},
         {"001", Mn::Vector3{12.3, 13.4, 14.5}},
         {"000", Mn::Vector3{11.3, 12.4, 13.5}}});

    MarkerSetTestMap marker_set_111_obj_name(
        {{"003", Mn::Vector3{14.4, 15.5, 16.6}},
         {"002", Mn::Vector3{13.4, 14.5, 15.6}},
         {"001", Mn::Vector3{12.4, 13.5, 14.6}},
         {"000", Mn::Vector3{11.4, 12.5, 13.6}}});

    LinkSetTestMap link_set_11_obj_name(
        {{"marker_set_110_obj_name", marker_set_110_obj_name},
         {"marker_set_110_obj_name", marker_set_110_obj_name}});

    TaskSetTestMap task_set_1_obj_name(
        {{"link_set_10_obj_name", link_set_10_obj_name},
         {"link_set_11_obj_name", link_set_11_obj_name}});

    AllMarkerSetsTestMap markerSetMap(
        {{"task_set_0_obj_name", task_set_0_obj_name},
         {"task_set_1_obj_name", task_set_1_obj_name}});

    testMarkerSetsConfigVals(objAttr->getMarkerSetsConfiguration(),
                             markerSetMap);
  }

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
  "rolling_friction_coefficient": 0.654,
  "spinning_friction_coefficient": 0.987,
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
  "marker_sets" : {
      "task_set_0_obj_name" : {
          "link_set_00_obj_name" : {
              "marker_set_000_obj_name" : {
                "markers" : [
                    [1.1, 2.2, 3.3],
                    [2.1, 3.2, 4.3],
                    [3.1, 4.2, 5.3],
                    [4.1, 5.2, 6.3]
                  ]
              },
              "marker_set_001_obj_name" : {
                "markers" : {
                    "0":[1.2, 2.3, 3.4],
                    "1":[2.2, 3.3, 4.4],
                    "2":[3.2, 4.3, 5.4],
                    "3":[4.2, 5.3, 6.4]
                }
              }
          },
          "link_set_01_obj_name" : {
              "marker_set_010_obj_name" : {
                "markers" : {
                    "0":[1.3, 2.4, 3.5],
                    "1":[2.3, 3.4, 4.5],
                    "2":[3.3, 4.4, 5.5],
                    "3":[4.3, 5.4, 6.5]
                }
              },
              "marker_set_011_obj_name" : {
                "markers" : [
                    [1.4, 2.5, 3.6],
                    [2.4, 3.5, 4.6],
                    [3.4, 4.5, 5.6],
                    [4.4, 5.5, 6.6]
                  ]
              }
          }
      },
      "task_set_1_obj_name" : {
          "link_set_10_obj_name" : {
              "marker_set_100_obj_name" : {
                "markers" : [
                    [11.1, 12.2, 13.3],
                    [12.1, 13.2, 14.3],
                    [13.1, 14.2, 15.3],
                    [14.1, 15.2, 16.3]
                  ]
              },
              "marker_set_101_obj_name" : {
                "markers" : {
                    "0":[11.2, 12.3, 13.4],
                    "1":[12.2, 13.3, 14.4],
                    "2":[13.2, 14.3, 15.4],
                    "3":[14.2, 15.3, 16.4]
                }
              }
          },
          "link_set_11_obj_name" : {
              "marker_set_110_obj_name" : {
                "markers" : {
                    "0":[11.3, 12.4, 13.5],
                    "1":[12.3, 13.4, 14.5],
                    "2":[13.3, 14.4, 15.5],
                    "3":[14.3, 15.4, 16.5]
                }
              },
              "marker_set_111_obj_name" : {
                "markers" : [
                    [11.4, 12.5, 13.6],
                    [12.4, 13.5, 14.6],
                    [13.4, 14.5, 15.6],
                    [14.4, 15.5, 16.6]
                  ]
              }
          }
      }
  },
  "user_defined" : {
      "user_str_array" : ["test_00", "test_01", "test_02", "test_03"],
      "user_string" : "object defined string",
      "user_bool" : true,
      "user_int" : 5,
      "user_double" : 2.6,
      "user_vec2" : [4.1, 2.8],
      "user_vec3" : [15.4, 17.6, 110.1],
      "user_vec4" : [1.5, 1.6, 6.7, 7.9],
      "user_quat" : [0.7, 5.5, 6.6, 7.7]
  }
})";

  // Build an attributes based on the above json string
  // Don't register - registration here verifies that the specified file
  // handles in the attributes exist, or it will fail.
  auto objAttr = objectAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, false);
  // verify exists
  CORRADE_VERIFY(objAttr);
  // now need to change the render and collision assets to make sure they are
  // legal so test can proceed (needs to be actual existing file)
  const std::string objAssetFile =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/donut.glb");

  objAttr->setRenderAssetHandle(objAssetFile);
  objAttr->setCollisionAssetHandle(objAssetFile);
  // now register so can be saved to disk
  objectAttributesManager_->registerObject(objAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testObjectAttrConfig_saved_JSON.{}", TEST_ASSETS,
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
  ESP_DEBUG() << "About to test saved objAttr2 :" << objAttr2->getHandle();
  testObjectAttrVals(objAttr2, objAssetFile);
  ESP_DEBUG() << "Tested saved objAttr2 :";

  // delete file-based config
  Cr::Utility::Path::remove(newAttrName);
}  // AttributesConfigsTest::testObjectJSONLoadTest

void AttributesConfigsTest::testArticulatedObjectAttrVals(
    std::shared_ptr<Attrs::ArticulatedObjectAttributes> artObjAttr,
    const std::string& assetPath,
    const std::string& urdfPath) {
  // match values set in test JSON
  CORRADE_COMPARE(artObjAttr->getURDFFullPath(), urdfPath);
  CORRADE_COMPARE(artObjAttr->getRenderAssetFullPath(), assetPath);
  CORRADE_COMPARE(artObjAttr->getSemanticId(), 100);

  CORRADE_COMPARE(static_cast<int>(artObjAttr->getBaseType()),
                  static_cast<int>(Attrs::ArticulatedObjectBaseType::Fixed));

  CORRADE_COMPARE(
      static_cast<int>(artObjAttr->getInertiaSource()),
      static_cast<int>(Attrs::ArticulatedObjectInertiaSource::URDF));

  CORRADE_COMPARE(
      static_cast<int>(artObjAttr->getLinkOrder()),
      static_cast<int>(Attrs::ArticulatedObjectLinkOrder::TreeTraversal));

  CORRADE_COMPARE(static_cast<int>(artObjAttr->getRenderMode()),
                  static_cast<int>(Attrs::ArticulatedObjectRenderMode::Skin));

  CORRADE_COMPARE(static_cast<int>(artObjAttr->getShaderType()),
                  static_cast<int>(Attrs::ObjectInstanceShaderType::PBR));

  // test object attributes-level user config vals
  testUserDefinedConfigVals(artObjAttr->getUserConfiguration(), 5,
                            "articulated object defined string", true, 6, 3.6,
                            Mn::Vector2(4.7f, 2.4f),
                            Mn::Vector3(15.1, 17.6, 110.1),
                            Mn::Quaternion({5.3f, 6.4f, 7.5f}, 0.8f),
                            Mn::Vector4(1.6f, 1.2f, 6.3f, 7.4f));

  {
    // Test marker sets

    MarkerSetTestMap marker_set_000_ao_name(
        {{"003", Mn::Vector3{14.1, 5.2, 6.3}},
         {"002", Mn::Vector3{13.1, 4.2, 5.3}},
         {"001", Mn::Vector3{12.1, 3.2, 4.3}},
         {"000", Mn::Vector3{11.1, 2.2, 3.3}}});

    MarkerSetTestMap marker_set_001_ao_name(
        {{"003", Mn::Vector3{14.2, 5.3, 6.4}},
         {"002", Mn::Vector3{13.2, 4.3, 5.4}},
         {"001", Mn::Vector3{12.2, 3.3, 4.4}},
         {"000", Mn::Vector3{11.2, 2.3, 3.4}}});

    LinkSetTestMap link_set_00_ao_name(
        {{"marker_set_000_ao_name", marker_set_000_ao_name},
         {"marker_set_001_ao_name", marker_set_001_ao_name}});

    MarkerSetTestMap marker_set_010_ao_name(
        {{"003", Mn::Vector3{14.3, 5.4, 6.5}},
         {"002", Mn::Vector3{13.3, 4.4, 5.5}},
         {"001", Mn::Vector3{12.3, 3.4, 4.5}},
         {"000", Mn::Vector3{11.3, 2.4, 3.5}}});

    MarkerSetTestMap marker_set_011_ao_name(
        {{"003", Mn::Vector3{14.4, 5.5, 6.6}},
         {"002", Mn::Vector3{13.4, 4.5, 5.6}},
         {"001", Mn::Vector3{12.4, 3.5, 4.6}},
         {"000", Mn::Vector3{11.4, 2.5, 3.6}}});

    LinkSetTestMap link_set_01_ao_name(
        {{"marker_set_010_ao_name", marker_set_010_ao_name},
         {"marker_set_011_ao_name", marker_set_011_ao_name}});

    MarkerSetTestMap marker_set_020_ao_name(
        {{"003", Mn::Vector3{24.3, 5.4, 6.5}},
         {"002", Mn::Vector3{23.3, 4.4, 5.5}},
         {"001", Mn::Vector3{22.3, 3.4, 4.5}},
         {"000", Mn::Vector3{21.3, 2.4, 3.5}}});

    MarkerSetTestMap marker_set_021_ao_name(
        {{"003", Mn::Vector3{24.4, 5.5, 6.6}},
         {"002", Mn::Vector3{23.4, 4.5, 5.6}},
         {"001", Mn::Vector3{22.4, 3.5, 4.6}},
         {"000", Mn::Vector3{21.4, 2.5, 3.6}}});

    MarkerSetTestMap marker_set_022_ao_name(
        {{"003", Mn::Vector3{224.4, 5.5, 6.6}},
         {"002", Mn::Vector3{223.4, 4.5, 5.6}},
         {"001", Mn::Vector3{222.4, 3.5, 4.6}},
         {"000", Mn::Vector3{221.4, 2.5, 3.6}}});

    LinkSetTestMap link_set_02_ao_name(
        {{"marker_set_020_ao_name", marker_set_020_ao_name},
         {"marker_set_021_ao_name", marker_set_021_ao_name},
         {"marker_set_022_ao_name", marker_set_022_ao_name}});

    TaskSetTestMap task_set_0_ao_name{
        {{"link_set_00_ao_name", link_set_00_ao_name},
         {"link_set_01_ao_name", link_set_01_ao_name},
         {"link_set_02_ao_name", link_set_02_ao_name}}};

    MarkerSetTestMap marker_set_100_ao_name(
        {{"003", Mn::Vector3{114.1, 15.2, 16.3}},
         {"002", Mn::Vector3{113.1, 14.2, 15.3}},
         {"001", Mn::Vector3{112.1, 13.2, 14.3}},
         {"000", Mn::Vector3{111.1, 12.2, 13.3}}});

    MarkerSetTestMap marker_set_101_ao_name(
        {{"003", Mn::Vector3{114.2, 15.3, 16.4}},
         {"002", Mn::Vector3{113.2, 14.3, 15.4}},
         {"001", Mn::Vector3{112.2, 13.3, 14.4}},
         {"000", Mn::Vector3{111.2, 12.3, 13.4}}});

    LinkSetTestMap link_set_10_ao_name(
        {{"marker_set_100_ao_name", marker_set_100_ao_name},
         {"marker_set_101_ao_name", marker_set_101_ao_name}});

    MarkerSetTestMap marker_set_110_ao_name(
        {{"003", Mn::Vector3{114.3, 15.4, 16.5}},
         {"002", Mn::Vector3{113.3, 14.4, 15.5}},
         {"001", Mn::Vector3{112.3, 13.4, 14.5}},
         {"000", Mn::Vector3{111.3, 12.4, 13.5}}});

    MarkerSetTestMap marker_set_111_ao_name(
        {{"003", Mn::Vector3{114.4, 15.5, 16.6}},
         {"002", Mn::Vector3{113.4, 14.5, 15.6}},
         {"001", Mn::Vector3{112.4, 13.5, 14.6}},
         {"000", Mn::Vector3{111.4, 12.5, 13.6}}});

    LinkSetTestMap link_set_11_ao_name(
        {{"marker_set_110_ao_name", marker_set_110_ao_name},
         {"marker_set_110_ao_name", marker_set_110_ao_name}});

    TaskSetTestMap task_set_1_ao_name(
        {{"link_set_10_ao_name", link_set_10_ao_name},
         {"link_set_11_ao_name", link_set_11_ao_name}});

    AllMarkerSetsTestMap markerSetMap(
        {{"task_set_0_ao_name", task_set_0_ao_name},
         {"task_set_1_ao_name", task_set_1_ao_name}});

    testMarkerSetsConfigVals(artObjAttr->getMarkerSetsConfiguration(),
                             markerSetMap);
  }

  // remove json-string built attributes added for test
  testRemoveAttributesBuiltByJSONString(artObjAttributesManager_,
                                        artObjAttr->getHandle());

}  // AttributesConfigsTest::testArticulatedObjectAttrVals

void AttributesConfigsTest::testArticulatedObjectJSONLoad() {
  // build JSON sample config
  const std::string& jsonString = R"({
  "urdf_filepath": "urdf_test_file.urdf",
  "render_asset": "testAO_JSONRenderAsset.glb",
  "semantic_id": 100,
  "base_type" : "fixed",
  "inertia_source" : "urdf",
  "link_order" : "tree_traversal",
  "render_mode": "skin",
  "shader_type" : "pbr",
  "marker_sets" : {
      "task_set_0_ao_name" : {
          "link_set_00_ao_name" : {
              "marker_set_000_ao_name" : {
                "markers" : [
                    [11.1, 2.2, 3.3],
                    [12.1, 3.2, 4.3],
                    [13.1, 4.2, 5.3],
                    [14.1, 5.2, 6.3]
                  ]
              },
              "marker_set_001_ao_name" : {
                "markers" : {
                    "0":[11.2, 2.3, 3.4],
                    "1":[12.2, 3.3, 4.4],
                    "2":[13.2, 4.3, 5.4],
                    "3":[14.2, 5.3, 6.4]
                }
              }
          },
          "link_set_01_ao_name" : {
              "marker_set_010_ao_name" : {
                "markers" : {
                    "0":[11.3, 2.4, 3.5],
                    "1":[12.3, 3.4, 4.5],
                    "2":[13.3, 4.4, 5.5],
                    "3":[14.3, 5.4, 6.5]
                }
              },
              "marker_set_011_ao_name" : {
                "markers" : [
                    [11.4, 2.5, 3.6],
                    [12.4, 3.5, 4.6],
                    [13.4, 4.5, 5.6],
                    [14.4, 5.5, 6.6]
                  ]
              }
          },
          "link_set_02_ao_name" : {
              "marker_set_020_ao_name" : {
                "markers" : {
                    "0":[21.3, 2.4, 3.5],
                    "1":[22.3, 3.4, 4.5],
                    "2":[23.3, 4.4, 5.5],
                    "3":[24.3, 5.4, 6.5]
                }
              },
              "marker_set_021_ao_name" : {
                "markers" : [
                    [21.4, 2.5, 3.6],
                    [22.4, 3.5, 4.6],
                    [23.4, 4.5, 5.6],
                    [24.4, 5.5, 6.6]
                  ]
              },
              "marker_set_022_ao_name" : {
                "markers" : [
                    [221.4, 2.5, 3.6],
                    [222.4, 3.5, 4.6],
                    [223.4, 4.5, 5.6],
                    [224.4, 5.5, 6.6]
                  ]
              }
          }
      },
      "task_set_1_ao_name" : {
          "link_set_10_ao_name" : {
              "marker_set_100_ao_name" : {
                "markers" : [
                    [111.1, 12.2, 13.3],
                    [112.1, 13.2, 14.3],
                    [113.1, 14.2, 15.3],
                    [114.1, 15.2, 16.3]
                  ]
              },
              "marker_set_101_ao_name" : {
                "markers" : {
                    "0":[111.2, 12.3, 13.4],
                    "1":[112.2, 13.3, 14.4],
                    "2":[113.2, 14.3, 15.4],
                    "3":[114.2, 15.3, 16.4]
                }
              }
          },
          "link_set_11_ao_name" : {
              "marker_set_110_ao_name" : {
                "markers" : {
                    "0":[111.3, 12.4, 13.5],
                    "1":[112.3, 13.4, 14.5],
                    "2":[113.3, 14.4, 15.5],
                    "3":[114.3, 15.4, 16.5]
                }
              },
              "marker_set_111_ao_name" : {
                "markers" : [
                    [111.4, 12.5, 13.6],
                    [112.4, 13.5, 14.6],
                    [113.4, 14.5, 15.6],
                    [114.4, 15.5, 16.6]
                  ]
              }
          }
      }
  },
  "user_defined" : {
      "user_str_array" : ["test_00", "test_01", "test_02", "test_03", "test_04"],
      "user_string" : "articulated object defined string",
      "user_bool" : true,
      "user_int" : 6,
      "user_double" : 3.6,
      "user_vec2" : [4.7, 2.4],
      "user_vec3" : [15.1, 17.6, 110.1],
      "user_vec4" : [1.6, 1.2, 6.3, 7.4],
      "user_quat" : [0.8, 5.3, 6.4, 7.5]
  }
})";

  // Build an attributes based on the above json string
  // Don't register - registration here verifies that the URDF file handle
  // specified in the attributes exist, or it will fail.
  auto artObjAttr = artObjAttributesManager_->createObjectFromJSONString(
      "new_template_from_json", jsonString, false);

  // verify created attributes exists
  CORRADE_VERIFY(artObjAttr);

  // now need to change urdf filename and render asset file name to make sure
  // they are legal so the test can proceed (needs to be actual existing file
  // so it can be regsitered)
  const std::string aoURDFFlle =
      Cr::Utility::Path::join(TEST_ASSETS, "urdf/prim_chain.urdf");
  const std::string aoRenderAssetName =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/skinned_prism.glb");

  artObjAttr->setURDFPath(aoURDFFlle);
  artObjAttr->setRenderAssetHandle(aoRenderAssetName);

  // now register so can be saved to disk
  artObjAttributesManager_->registerObject(artObjAttr);

  // before test, save attributes to disk with new name
  std::string newAttrName = Cr::Utility::formatString(
      "{}/testArtObjectAttrConfig_saved_JSON.{}", TEST_ASSETS,
      artObjAttributesManager_->getJSONTypeExt());

  bool success = artObjAttributesManager_->saveManagedObjectToFile(
      artObjAttr->getHandle(), newAttrName);

  // test json string to verify format - this also deletes objAttr from
  // manager

  ESP_DEBUG() << "About to test string-based artObjAttr :";
  testArticulatedObjectAttrVals(artObjAttr, aoRenderAssetName, aoURDFFlle);
  ESP_DEBUG() << "Tested string-based artObjAttr :";
  artObjAttr = nullptr;

  // load attributes from new name and retest
  auto artObjAttr2 =
      artObjAttributesManager_->createObjectFromJSONFile(newAttrName);

  // verify file-based config exists
  CORRADE_VERIFY(artObjAttr2);

  // test json string to verify format, this deletes objAttr2 from
  // registry
  ESP_DEBUG() << "About to test saved artObjAttr2 :"
              << artObjAttr2->getHandle();
  testArticulatedObjectAttrVals(artObjAttr2, aoRenderAssetName, aoURDFFlle);
  ESP_DEBUG() << "Tested saved artObjAttr2 :";

  // delete file-based config
  Cr::Utility::Path::remove(newAttrName);

}  // AttributesConfigsTest::testArticulatedObjectJSONLoad

}  // namespace

CORRADE_TEST_MAIN(AttributesConfigsTest)
