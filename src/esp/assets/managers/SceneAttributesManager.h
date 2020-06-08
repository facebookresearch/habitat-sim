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
  using AttributesManager<PhysicsSceneAttributes::ptr>::AttributesManager;
  /**
   * @brief Creates an instance of a scene template described by passed
   string.
   * For scene templates, this a file name.
   *
   * @param sceneAttributesHandle the origin of the desired template to be
   * created, in this case, a file name.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the desired template.
   */

  PhysicsSceneAttributes::ptr createAttributesTemplate(
      const std::string& sceneAttributesHandle,
      bool registerTemplate = true);
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
  int registerAttributesTemplate(
      PhysicsSceneAttributes::ptr sceneAttributesTemplate,
      const std::string& sceneAttributesHandle) {
    // return either the ID of the existing template referenced by
    // sceneAttributesHandle, or the next available ID if not found.
    int sceneTemplateID = this->addTemplateToLibrary(sceneAttributesTemplate,
                                                     sceneAttributesHandle);
    return sceneTemplateID;
  }  // SceneAttributesManager::registerAttributesTemplate

  /**
   * @brief Sets all relevant attributes to all scenes based on passed @ref
   * physicsManagerAttributes.
   *
   * @param physicsManagerAttributes The attributes describing the physics world
   * this scene lives in.
   */
  void setSceneValsFromPhysicsAttributes(
      PhysicsManagerAttributes::ptr physicsManagerAttributes) {
    for (auto sceneAttr : this->templateLibrary_) {
      sceneAttr.second->setFrictionCoefficient(
          physicsManagerAttributes->getFrictionCoefficient());
      sceneAttr.second->setRestitutionCoefficient(
          physicsManagerAttributes->getRestitutionCoefficient());
    }
  }  // SceneAttributesManager::setSceneValsFromPhysicsAttributes

 protected:
  // instance vars

 public:
  ESP_SMART_POINTERS(SceneAttributesManager)

};  // SceneAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_SCENEATTRIBUTEMANAGER_H_