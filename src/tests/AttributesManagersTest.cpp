// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/assets/managers/AttributesManagerBase.h"

#include "configure.h"

namespace Cr = Corrade;

namespace AttrMgrs = esp::assets::managers;

using esp::assets::PrimObjTypes;
using esp::assets::ResourceManager;
using esp::assets::managers::AttributesManager;
const std::string dataDir = Cr::Utility::Directory::join(SCENE_DATASETS, "../");
const std::string physicsConfigFile = Cr::Utility::Directory::join(
    SCENE_DATASETS,
    "../test_assets/testing.phys_scene_config.json");

class AttributesManagersTest : public testing::Test {
 protected:
  void SetUp() override {
    // get attributes managers
    assetAttributesManager_ = resourceManager_.getAssetAttributesManager();
    objectAttributesManager_ = resourceManager_.getObjectAttributesManager();
    physicsAttributesManager_ = resourceManager_.getPhysicsAttributesManager();
    sceneAttributesManager_ = resourceManager_.getSceneAttributesManager();
  };

  /**
   * @brief Test creation, copying and removal of templates for Object, Physics
   * and Scene Attributes Managers
   * @tparam Class of attributes manager
   * @param mgr the Attributes Manager being tested,
   * @param handle the handle of the desired attributes template to work with
   */
  template <class T>
  void testCreateAndRemove(std::shared_ptr<T> mgr, const std::string& handle) {
    // meaningless key to modify attributes for verifcation of behavior
    std::string keyStr = "tempKey";
    // get starting number of templates
    int orignNumTemplates = mgr->getNumTemplates();
    // verify template is not present - should not be
    bool isPresentAlready = mgr->getTemplateLibHasHandle(handle);
    ASSERT_NE(isPresentAlready, true);

    // create template from source handle, register it and retrieve it
    // Note: registration of template means this is a copy of registered
    // template
    auto attrTemplate1 = mgr->createAttributesTemplate(handle, true);
    // verify it exists
    ASSERT_NE(nullptr, attrTemplate1);
    // retrieve a copy of the named attributes template
    auto attrTemplate2 = mgr->getTemplateCopyByHandle(handle);
    // verify copy has same quantities and values as original
    ASSERT_EQ(attrTemplate1->getOriginHandle(),
              attrTemplate2->getOriginHandle());

    // test changing a user-defined field in each template, verify the templates
    // are not now the same
    attrTemplate1->setString(keyStr, "temp");
    attrTemplate2->setString(keyStr, "temp2");
    ASSERT_NE(attrTemplate1->getString(keyStr),
              attrTemplate2->getString(keyStr));
    // get original template ID
    int oldID = attrTemplate1->getObjectTemplateID();

    // register modified template and verify that this is the template now
    // stored
    int newID = mgr->registerAttributesTemplate(attrTemplate2, handle);
    // verify IDs are the same
    ASSERT_EQ(oldID, newID);

    // get another copy
    auto attrTemplate3 = mgr->getTemplateCopyByHandle(handle);
    // verify added field is present and the same
    ASSERT_EQ(attrTemplate3->getString(keyStr),
              attrTemplate2->getString(keyStr));
    // change field in new copy
    attrTemplate3->setString(keyStr, "temp3");
    // verify that now they are different
    ASSERT_NE(attrTemplate3->getString(keyStr),
              attrTemplate2->getString(keyStr));

    // test removal
    int removeID = attrTemplate2->getObjectTemplateID();
    // remove template by ID, acquire copy of removed template
    auto oldTemplate = mgr->removeTemplateByID(removeID);
    // verify it exists
    ASSERT_NE(nullptr, oldTemplate);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumTemplates());
    // re-add template copy via registration
    int newAddID = mgr->registerAttributesTemplate(attrTemplate2, handle);
    // verify IDs are the same
    ASSERT_EQ(removeID, newAddID);

    // remove  attributes via handle
    auto oldTemplate2 = mgr->removeTemplateByHandle(handle);
    // verify deleted template  exists
    ASSERT_NE(nullptr, oldTemplate2);
    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, mgr->getNumTemplates());
  }  // AttributesManagersTest::testCreateAndRemove

  /**
   * @brief Test creation, copying and removal of templates for Object, Physics
   * and Scene Attributes Managers
   * @tparam Class of attributes being managed
   * @tparam Class of constructor field that impacts template handle
   * @param defaultAttribs the default template of the passed type T
   * @param ctorModField the name of the modified field of type @ref U that
   * impacts the constructor.
   * @param legalVal a legal value of ctorModField; This should be different
   * than template default for @ref ctorModField.
   * @param illegalVal a legal value of ctorModField.  If null ptr then no
   * illegal values possible.
   */
  template <class T, class U>
  void testAssetAttributesModRegRemove(std::shared_ptr<T> defaultAttribs,
                                       const std::string& ctorModField,
                                       U legalVal,
                                       U const* illegalVal) {
    // get starting number of templates
    int orignNumTemplates = assetAttributesManager_->getNumTemplates();

    // get name of default template
    std::string oldHandle = defaultAttribs->getOriginHandle();

    // verify default template is valid
    bool isTemplateValid = defaultAttribs->isValidTemplate();
    ASSERT_EQ(isTemplateValid, true);

    // if illegal values are possible
    if (nullptr != illegalVal) {
      // modify template value used by primitive constructor (will change name)
      // illegal modification
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
    // rebuild origin handle to reflect new parameters
    defaultAttribs->buildOriginHandle();

    // get synthesized handle
    std::string newHandle = defaultAttribs->getOriginHandle();
    LOG(INFO) << "Modified Template Handle : " << newHandle;
    // register modified template
    assetAttributesManager_->registerAttributesTemplate(defaultAttribs);

    // verify new handle is in template library
    // get template by handle
    ASSERT(assetAttributesManager_->getTemplateLibHasHandle(newHandle));
    // verify old template is still present as well
    ASSERT(assetAttributesManager_->getTemplateLibHasHandle(oldHandle));

    // get new template
    std::shared_ptr<T> newAttribs =
        assetAttributesManager_->getTemplateCopyByHandle<T>(newHandle);
    // verify template has modified values
    U newValue = newAttribs->template get<U>(ctorModField);
    ASSERT_EQ(legalVal, newValue);
    // remove modified template via handle
    auto oldTemplate2 =
        assetAttributesManager_->removeTemplateByHandle(newHandle);
    // verify deleted template  exists
    ASSERT_NE(nullptr, oldTemplate2);

    // verify there are same number of templates as when we started
    ASSERT_EQ(orignNumTemplates, assetAttributesManager_->getNumTemplates());

  }  // AttributesManagersTest::testAssetAttributesModRegRemove

  ResourceManager resourceManager_;

  AttrMgrs::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;
  AttrMgrs::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;
  AttrMgrs::SceneAttributesManager::ptr sceneAttributesManager_ = nullptr;

};  // class AttributesManagersTest

TEST_F(AttributesManagersTest, AttributesManagersCreate) {
  LOG(INFO) << "Starting AttributesManagersTest::AttributesManagersCreate";
  std::string sceneFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/scenes/simple_room.glb");

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/chair.phys_properties.json");

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "PhysicsAttributesManager @ "
            << physicsConfigFile;
  // physics attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::PhysicsAttributesManager>(
      physicsAttributesManager_, physicsConfigFile);
  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "SceneAttributesManager @ "
            << sceneFile;

  // scene attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::SceneAttributesManager>(sceneAttributesManager_,
                                                        sceneFile);

  LOG(INFO) << "Start Test : Create, Edit, Remove Attributes for "
               "ObjectAttributesManager @ "
            << objectFile;
  // object attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::ObjectAttributesManager>(
      objectAttributesManager_, objectFile);

}  // AttributesManagersTest::AttributesManagersCreate test

TEST_F(AttributesManagersTest, PrimitiveAssetAttributesTest) {
  LOG(INFO) << "Starting "
               "AttributesManagersTest::PrimitiveAssetAttributesTest";
  /**
   * primitive asset attributes require slightly different testing since a
   * default set of attributes are created on program load and are always
   * present.  User modification of asset attributes always starts by modifying
   * an existing default template - users will never create an attributes
   * template from scratch.
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
    esp::assets::CapsulePrimitiveAttributes::ptr dfltCapsAttribs =
        assetAttributesManager_->getDefaultCapsuleTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<esp::assets::CapsulePrimitiveAttributes,
                                    int>(dfltCapsAttribs, "segments",
                                         legalModValSolid, &illegalModValSolid);

    // test wireframe version
    dfltCapsAttribs = assetAttributesManager_->getDefaultCapsuleTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCapsAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<esp::assets::CapsulePrimitiveAttributes,
                                    int>(dfltCapsAttribs, "segments",
                                         legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid cone
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::ConePrimitiveAttributes";

    esp::assets::ConePrimitiveAttributes::ptr dfltConeAttribs =
        assetAttributesManager_->getDefaultConeTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<esp::assets::ConePrimitiveAttributes, int>(
        dfltConeAttribs, "segments", legalModValSolid, &illegalModValSolid);

    // test wireframe version
    dfltConeAttribs = assetAttributesManager_->getDefaultConeTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltConeAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<esp::assets::ConePrimitiveAttributes, int>(
        dfltConeAttribs, "segments", legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid cylinder
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::CylinderPrimitiveAttributes";

    esp::assets::CylinderPrimitiveAttributes::ptr dfltCylAttribs =
        assetAttributesManager_->getDefaultCylinderTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<esp::assets::CylinderPrimitiveAttributes,
                                    int>(dfltCylAttribs, "segments", 5,
                                         &illegalModValSolid);

    // test wireframe version
    dfltCylAttribs = assetAttributesManager_->getDefaultCylinderTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltCylAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<esp::assets::CylinderPrimitiveAttributes,
                                    int>(dfltCylAttribs, "segments",
                                         legalModValWF, &illegalModValWF);
  }
  //////////////////////////
  // get default template for solid UV Sphere
  {
    LOG(INFO) << "Starting "
                 "AttributesManagersTest::UVSpherePrimitiveAttributes";

    esp::assets::UVSpherePrimitiveAttributes::ptr dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(false);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);

    // for solid primitives, and value > 2 for segments is legal
    testAssetAttributesModRegRemove<esp::assets::UVSpherePrimitiveAttributes,
                                    int>(dfltUVSphereAttribs, "segments", 5,
                                         &illegalModValSolid);

    // test wireframe version
    dfltUVSphereAttribs =
        assetAttributesManager_->getDefaultUVSphereTemplate(true);
    // verify it exists
    ASSERT_NE(nullptr, dfltUVSphereAttribs);
    // segments must be mult of 4 for wireframe primtives
    testAssetAttributesModRegRemove<esp::assets::UVSpherePrimitiveAttributes,
                                    int>(dfltUVSphereAttribs, "segments",
                                         legalModValWF, &illegalModValWF);
  }
}  // AttributesManagersTest::AsssetAttributesManagerGetAndModify test