// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_

#include "AbstractAttributesManager.h"

#include "esp/gfx/LightSetup.h"
#include "esp/metadata/attributes/LightLayoutAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

class LightLayoutAttributesManager
    : public AbstractAttributesManager<attributes::LightLayoutAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  LightLayoutAttributesManager()
      : AbstractAttributesManager<attributes::LightLayoutAttributes,
                                  ManagedObjectAccess::Copy>::
            AbstractAttributesManager("Lighting Layout",
                                      "lighting_config.json") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["LightLayoutAttributes"] =
        &LightLayoutAttributesManager::createObjCopyCtorMapEntry<
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

  /**
   * @brief Not required for this manager.
   *
   * This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   * @param attributes The attributes to be filtered.
   */
  void finalizeAttrPathsBeforeRegister(
      CORRADE_UNUSED const attributes::LightLayoutAttributes::ptr& attributes)
      const override {}

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
   * AbstractAttributesManager-specific upon template removal.  This should only
   * be
   * called from @ref esp::core::managedContainers::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any essential updating to the managed object
   * before registration is performed. If this updating fails, registration will
   * also fail.
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Should register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(
      CORRADE_UNUSED attributes::LightLayoutAttributes::ptr object,
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool forceRegistration) override {
    // No pre-registration conditioning performed
    return core::managedContainers::ManagedObjectPreregistration::Success;
  }

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any final manager-related handling after
   * successfully registering an object.
   *
   * See @ref esp::attributes::managers::ObjectAttributesManager for an example.
   *
   * @param objectID the ID of the successfully registered managed object
   * @param objectHandle The name of the managed object
   */
  void postRegisterObjectHandling(
      CORRADE_UNUSED int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {}

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
