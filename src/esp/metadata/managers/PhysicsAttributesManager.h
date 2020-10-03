// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "ObjectAttributesManager.h"

#include "esp/metadata/attributes/PhysicsManagerAttributes.h"
#include "esp/physics/configure.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {
class PhysicsAttributesManager
    : public AttributesManager<Attrs::PhysicsManagerAttributes> {
 public:
  PhysicsAttributesManager(ObjectAttributesManager::ptr objectAttributesMgr)
      : AttributesManager<Attrs::PhysicsManagerAttributes>::AttributesManager(
            "Physics Manager",
            "phys_scene_config.json"),
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
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param physicsFilename The configuration file to parse. Defaults to the
   * file location `ESP_DEFAULT_PHYS_SCENE_CONFIG_REL_PATH` set by cmake.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file.
   */
  Attrs::PhysicsManagerAttributes::ptr createObject(
      const std::string& physicsFilename =
          ESP_DEFAULT_PHYS_SCENE_CONFIG_REL_PATH,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(Attrs::PhysicsManagerAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

 protected:
  /**
   * @brief Check if currently configured primitive asset template library has
   * passed handle.
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  bool isValidPrimitiveAttributes(const std::string& handle) override {
    return objectAttributesMgr_->getObjectLibHasHandle(handle);
  }

  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   */
  Attrs::PhysicsManagerAttributes::ptr initNewObjectInternal(
      const std::string& handleName) override {
    Attrs::PhysicsManagerAttributes::ptr newAttributes =
        this->constructFromDefault();
    if (nullptr == newAttributes) {
      newAttributes = Attrs::PhysicsManagerAttributes::create(handleName);
    }
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
  void updateObjectHandleLists(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param physicsAttributesTemplate The attributes template.
   * @param physicsAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      Attrs::PhysicsManagerAttributes::ptr physicsAttributesTemplate,
      const std::string& physicsAttributesHandle) override {
    // adds template to library, and returns either the ID of the existing
    // template referenced by physicsAttributesHandle, or the next available ID
    // if not found.
    int physicsTemplateID = this->addObjectToLibrary(physicsAttributesTemplate,
                                                     physicsAttributesHandle);
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
        &PhysicsAttributesManager::createObjectCopy<
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
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_
