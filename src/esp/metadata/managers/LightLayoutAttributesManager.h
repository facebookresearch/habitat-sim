// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "esp/gfx/LightSetup.h"
#include "esp/metadata/attributes/LightLayoutAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

class LightLayoutAttributesManager
    : public AttributesManager<attributes::LightLayoutAttributes,
                               ManagedObjectAccess::Copy> {
 public:
  LightLayoutAttributesManager()
      : AttributesManager<attributes::LightLayoutAttributes,
                          ManagedObjectAccess::Copy>::
            AttributesManager("Lighting Layout", "lighting_config.json") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["LightLayoutAttributes"] =
        &LightLayoutAttributesManager::createObjectCopy<
            attributes::LightLayoutAttributes>;
  }

  /**
   * @brief Creates one or more instances of LightLayoutAttributes based on the
   * whether @p lightConfigName is a file or a not.  If it is a file it will
   * consider the contents of that file a layout and will use the file name as
   * the layout name and load all the attributes described and assign them to
   * that layout.  File-based loads will automatically register, regardless of
   * what @p registerTemplate is.
   *
   * If a template/layout exists with this handle, this existing template/layout
   * will be overwritten with the newly created one if registerTemplate is true.
   *
   * @param lightConfigName The configuration file to parse, or the name of the
   * single light's attributes to create.
   * @param registerTemplate whether to add this template to the library.
   * Defaults to false - overridden if @p lightConfigName is a JSON file.
   * @return a reference to the created light attributes.
   */
  attributes::LightLayoutAttributes::ptr createObject(
      const std::string& lightConfigName,
      bool registerTemplate = false) override;

  /**
   * @brief Function to take an existing LightLayoutAttributes and set its
   * values from passed json config file.
   * @param lightAttribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::LightLayoutAttributes::ptr lightAttribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief Function to take an existing LightInstanceAttributes and set its
   * values from passed json config file
   * @param lightInstAttribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setLightInstanceValsFromJSONDoc(
      const attributes::LightInstanceAttributes::ptr& lightInstAttribs,
      const io::JsonGenericValue& jsonConfig);

  /**
   * @brief This will create a @ref gfx::LightSetup object based on the
   * LightLayoutAttributes referenced by the passed name.
   * @param lightConfigName the name of the LightLayoutAttributes to be used to
   * create the LightSetup.
   * @return The lightSetup defined by the attributes, or an empty LightSetup.
   */
  gfx::LightSetup createLightSetupFromAttributes(
      const std::string& lightConfigName);

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created attributes
   * with any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes\
   * @param builtFromConfig whether this LightLayoutAttributes is being
   * built from a config file, or from some other source (i.e. handleName
   * contains config file name)
   * @return Newly created but unregistered LightLayoutAttributes pointer,
   * with only default values set.
   */
  attributes::LightLayoutAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal.  This should only be
   * called from @ref esp::core::managedContainers::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::LightLayoutAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param LightLayoutAttributesTemplate The attributes template.
   * @param LightLayoutAttributesHandle The key for referencing the template in
   * the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::LightLayoutAttributes::ptr LightLayoutAttributesTemplate,
      const std::string& LightLayoutAttributesHandle,
      CORRADE_UNUSED bool forceRegistration) override;

  /**
   * @brief Any lights-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

 public:
  ESP_SMART_POINTERS(LightLayoutAttributesManager)

};  // LightLayoutAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_
