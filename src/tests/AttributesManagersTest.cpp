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

using esp::assets::ResourceManager;
using esp::assets::managers::AttributesManager;
const std::string dataDir = Cr::Utility::Directory::join(SCENE_DATASETS, "../");
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "../default.phys_scene_config.json");

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
   * @param mgr the Attributes Manager being tested,
   * @param handle the handle of the desired attributes template to work with
   */
  template <class T>
  void testCreateAndRemove(std::shared_ptr<T> mgr, const std::string& handle) {
    std::string keyStr = "tempKey";
    // get starting number of templates
    int orignNumTemplates = mgr->getNumTemplates();
    // verify templates is not present - should not be
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
    // change a field in each template, verify the templates are not now the
    // same
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

  ResourceManager resourceManager_;

  AttrMgrs::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;
  AttrMgrs::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;
  AttrMgrs::SceneAttributesManager::ptr sceneAttributesManager_ = nullptr;

};  // class AttributesManagersTest

TEST_F(AttributesManagersTest, AttributesManagersCreate) {
  LOG(INFO) << "Starting  AttributesManagersTest:: AttributesManagersCreate";
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
  // scene attributes manager attributes verifcation
  testCreateAndRemove<AttrMgrs::ObjectAttributesManager>(
      objectAttributesManager_, objectFile);

}  // AttributesManagersCreate test