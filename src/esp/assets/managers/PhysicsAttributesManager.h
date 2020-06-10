// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "AttributesManagerBase.h"

#include "esp/gfx/configure.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {

namespace managers {

class PhysicsAttributesManager
    : public AttributesManager<PhysicsManagerAttributes::ptr> {
 public:
  PhysicsAttributesManager()
      : AttributesManager<PhysicsManagerAttributes::ptr>::AttributesManager() {
    buildCtorFuncPtrMaps();
  }

  /**
   * @brief Creates an instance of a physics world template described by passed
   * string. For physics templates, this a file name. Parses global physics
   * simulation parameters (such as timestep, gravity, simulator implementation)
   * from the specified configuration file.
   *
   * @param physicsFilename The configuration file to parse. Defaults to the
   * file location @ref ESP_DEFAULT_PHYS_SCENE_CONFIG set by cmake.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file.
   */
  const PhysicsManagerAttributes::ptr createAttributesTemplate(
      const std::string& physicsFilename = ESP_DEFAULT_PHYS_SCENE_CONFIG,
      bool registerTemplate = true);

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
  int registerAttributesTemplate(
      const PhysicsManagerAttributes::ptr physicsAttributesTemplate,
      const std::string& physicsAttributesHandle) {
    // return either the ID of the existing template referenced by
    // physicsAttributesHandle, or the next available ID if not found.
    int physicsTemplateID = this->addTemplateToLibrary(
        physicsAttributesTemplate, physicsAttributesHandle);
    return physicsTemplateID;
  }  // PhysicsAttributesManager::registerAttributesTemplate

  /**
   * @brief Build all "*.phys_properties.json" files from the provided file or
   * directory path.
   *
   * @param path A global path to a physics property file or directory
   * @return A list of valid global paths to "*.phys_properties.json" files.
   */
  std::vector<std::string> buildObjectConfigPaths(const std::string& path);

 protected:
  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["PhysicsManagerAttributes"] =
        &AttributesManager<PhysicsManagerAttributes::ptr>::createAttributesCopy<
            assets::PhysicsManagerAttributes>;
  }
  // instance vars

 public:
  ESP_SMART_POINTERS(PhysicsAttributesManager)

};  // PhysicsAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_PHYSICSATTRIBUTEMANAGER_H_