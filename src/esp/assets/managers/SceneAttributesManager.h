// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "ObjectAttributesManager.h"

namespace esp {
namespace assets {

namespace managers {
class SceneAttributesManager
    : public AttributesManager<PhysicsSceneAttributes::ptr> {
 public:
  SceneAttributesManager(assets::ResourceManager& resourceManager,
                         ObjectAttributesManager::ptr objectAttributesMgr)
      : AttributesManager<PhysicsSceneAttributes::ptr>::AttributesManager(
            resourceManager),
        objectAttributesMgr_(objectAttributesMgr) {
    buildCtorFuncPtrMaps();
  }
  /**
   * @brief Creates an instance of a scene template described by passed string.
   * For scene templates, this a file name.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   *
   * @param sceneAttributesHandle the origin of the desired template to be
   * created, in this case, a file name.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  PhysicsSceneAttributes::ptr createAttributesTemplate(
      const std::string& sceneAttributesHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Creates a an instance of scene attributes template populated with
   * default values. Assigns the @ref templateName as the template's handle,
   * and render and collision handles.
   *
   * If a template exists with this handle, the existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   * This method is specifically intended to directly construct an attributes
   * template for editing, and so defaults to false for @ref registerTemplate
   *
   * @param templateName Name to use for the attributes handle.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to false. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template, or nullptr if fails.
   */
  PhysicsSceneAttributes::ptr createDefaultAttributesTemplate(
      const std::string& templateName,
      bool registerTemplate = false) override;

  /**
   * @brief Sets all relevant attributes to all scenes based on passed @ref
   * physicsManagerAttributes.
   *
   * @param physicsManagerAttributes The attributes describing the physics world
   * this scene lives in.
   */
  void setSceneValsFromPhysicsAttributes(
      const PhysicsManagerAttributes::cptr physicsManagerAttributes) {
    for (auto sceneAttrPair : this->templateLibrary_) {
      auto sceneAttr = this->getTemplateCopyByHandle(sceneAttrPair.first);
      sceneAttr->setFrictionCoefficient(
          physicsManagerAttributes->getFrictionCoefficient());
      sceneAttr->setRestitutionCoefficient(
          physicsManagerAttributes->getRestitutionCoefficient());
      this->addTemplateToLibrary(sceneAttr, sceneAttrPair.first);
    }
  }  // SceneAttributesManager::setSceneValsFromPhysicsAttributes

 protected:
  /**
   * @brief Read and parse the json file @ref sceneFilename and populate a
   * returned scene attributes with appropriate data.
   *
   * @param sceneFilename The configuration file to parse.
   * @param registerTemplate whether to add this template to the library or not.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file, or nullptr if fails.
   */
  PhysicsSceneAttributes::ptr createFileBasedAttributesTemplate(
      const std::string& sceneFilename,
      bool registerTemplate);

  /**
   * @brief Add a @ref std::shared_ptr<attributesType> object to the
   * @ref templateLibrary_.
   *
   * @param sceneAttributesTemplate The attributes template.
   * @param sceneAttributesHandle The key for referencing the template in the
   * @ref templateLibrary_.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplateFinalize(
      PhysicsSceneAttributes::ptr sceneAttributesTemplate,
      const std::string& sceneAttributesHandle) override {
    // adds template to library, and returns either the ID of the existing
    // template referenced by sceneAttributesHandle, or the next available ID
    // if not found.
    int sceneTemplateID = this->addTemplateToLibrary(sceneAttributesTemplate,
                                                     sceneAttributesHandle);
    return sceneTemplateID;
  }  // SceneAttributesManager::registerAttributesTemplate

  /**
   * @brief Whether template described by passed handle is read only, or can be
   * deleted. All SceneAttributes templates are removable, by default
   */
  bool isTemplateReadOnly(const std::string&) override { return false; };

  /**
   * @brief Any scene-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["PhysicsSceneAttributes"] =
        &SceneAttributesManager::createAttributesCopy<
            assets::PhysicsSceneAttributes>;
  }  // SceneAttributesManager::buildCtorFuncPtrMaps

  // instance vars

  /**
   * @brief Reference to ObjectAttributesManager to give access to setting
   * object template library using paths specified in SceneAttributes json
   */
  ObjectAttributesManager::ptr objectAttributesMgr_ = nullptr;

 public:
  ESP_SMART_POINTERS(SceneAttributesManager)

};  // SceneAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_