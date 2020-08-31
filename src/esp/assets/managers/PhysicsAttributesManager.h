// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "ObjectAttributesManager.h"

#include "esp/assets/attributes/PhysicsManagerAttributes.h"
#include "esp/physics/configure.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {

namespace managers {
class PhysicsAttributesManager
    : public AttributesManager<Attrs::PhysicsManagerAttributes::ptr> {
 public:
  PhysicsAttributesManager(assets::ResourceManager& resourceManager,
                           ObjectAttributesManager::ptr objectAttributesMgr)
      : AttributesManager<Attrs::PhysicsManagerAttributes::ptr>::
            AttributesManager(resourceManager, "Physics Manager"),
        objectAttributesMgr_(objectAttributesMgr) {
    buildCtorFuncPtrMaps();
  }

  /**
   * @brief Creates an instance of a physics world template described by passed
   * string. For physics templates, this a file name. Parses global physics
   * simulation parameters (such as timestep, gravity, simulator implementation)
   * from the specified configuration file.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   *
   * @param physicsFilename The configuration file to parse. Defaults to the
   * file location @ref ESP_DEFAULT_PHYS_SCENE_CONFIG_REL_PATH set by cmake.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file.
   */
  Attrs::PhysicsManagerAttributes::ptr createAttributesTemplate(
      const std::string& physicsFilename =
          ESP_DEFAULT_PHYS_SCENE_CONFIG_REL_PATH,
      bool registerTemplate = true) override;

  /**
   * @brief Creates a an instance of physics manager attributes template
   * populated with default values. Assigns the @ref templateName as the
   * template's handle.
   *
   * If a template exists with this handle, the existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   * This method is specifically intended to directly construct an attributes
   * template for editing, and so defaults to false for @ref registerTemplate

   * @param templateName Name to use for the attributes handle.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to false. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template, or nullptr if fails.
   */
  Attrs::PhysicsManagerAttributes::ptr createDefaultAttributesTemplate(
      const std::string& templateName,
      bool registerTemplate = false) override;

  /**
   * @brief Parse passed JSON Document specifically for @ref
   * PhysicsManagerAttributes object. It always returns a valid @ref
   * PhysicsManagerAttributes::ptr object.
   *
   * @param templateName the desired handle of the @ref
   * PhysicsManagerAttributes.
   * @param jsonConfig json document to parse
   * @return a reference to the desired template.
   */
  Attrs::PhysicsManagerAttributes::ptr loadAttributesFromJSONDoc(
      const std::string& templateName,
      const io::JsonDocument& jsonConfig) override;

 protected:
  /**
   * @brief Not used by PhysicsManagerAttributes.
   */
  void setDefaultFileNameBasedAttributes(
      CORRADE_UNUSED Attrs::PhysicsManagerAttributes::ptr attributes,
      CORRADE_UNUSED bool setFrame,
      CORRADE_UNUSED const std::string& meshHandle,
      CORRADE_UNUSED std::function<void(int)> meshTypeSetter) override {}

  /**
   * @brief Used Internally.  Configure newly-created attributes with any
   * default values, before any specific values are set.
   *
   * @param newAttributes Newly created attributes.
   */
  Attrs::PhysicsManagerAttributes::ptr initNewAttribsInternal(
      Attrs::PhysicsManagerAttributes::ptr newAttributes) override {
    this->setFileDirectoryFromHandle(newAttributes);
    return newAttributes;
  }
  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void updateTemplateHandleLists(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a @ref PhysicsManagerAttributes::ptr object to the @ref
   * templateLibrary_.
   *
   * @param physicsAttributesTemplate The attributes template.
   * @param physicsAttributesHandle The key for referencing the template in the
   * @ref templateLibrary_.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplateFinalize(
      Attrs::PhysicsManagerAttributes::ptr physicsAttributesTemplate,
      const std::string& physicsAttributesHandle) override {
    // adds template to library, and returns either the ID of the existing
    // template referenced by physicsAttributesHandle, or the next available ID
    // if not found.
    int physicsTemplateID = this->addTemplateToLibrary(
        physicsAttributesTemplate, physicsAttributesHandle);
    return physicsTemplateID;
  }  // PhysicsAttributesManager::registerAttributesTemplate

  /**
   * @brief Any physics-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["PhysicsManagerAttributes"] =
        &PhysicsAttributesManager::createAttributesCopy<
            Attrs::PhysicsManagerAttributes>;
  }  // PhysicsAttributesManager::buildCtorFuncPtrMaps

  // instance vars

  /**
   * @brief Reference to ObjectAttributesManager to give access to setting
   * object template library using paths specified in PhysicsAttributes json
   */
  ObjectAttributesManager::ptr objectAttributesMgr_ = nullptr;

 public:
  ESP_SMART_POINTERS(PhysicsAttributesManager)

};  // PhysicsAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_
