// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

namespace esp {
namespace assets {

namespace managers {
class SceneAttributesManager
    : public AttributesManager<PhysicsSceneAttributes::ptr> {
 public:
  SceneAttributesManager(assets::ResourceManager& resourceManager)
      : AttributesManager<PhysicsSceneAttributes::ptr>::AttributesManager(
            resourceManager) {
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
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the desired template.
   */

  PhysicsSceneAttributes::ptr createAttributesTemplate(
      const std::string& sceneAttributesHandle,
      bool registerTemplate = true) override;

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
   * deleted.  All SceneAttributes templates are removable, by default
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
  }

  // instance vars

 public:
  ESP_SMART_POINTERS(SceneAttributesManager)

};  // SceneAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_