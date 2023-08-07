// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_PBRSHADERATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_PBRSHADERATTRIBUTESMANAGER_H_

#include <utility>

#include "AttributesManagerBase.h"

#include "esp/gfx/configure.h"
#include "esp/metadata/attributes/PbrShaderAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

class PbrShaderAttributesManager
    : public AttributesManager<attributes::PbrShaderAttributes,
                               ManagedObjectAccess::Copy> {
 public:
  PbrShaderAttributesManager()
      : AttributesManager<
            attributes::PbrShaderAttributes,
            ManagedObjectAccess::Copy>::AttributesManager("PBR Rendering",
                                                          "pbr_config.json") {
    this->copyConstructorMap_["PbrShaderAttributes"] =
        &PbrShaderAttributesManager::createObjectCopy<
            attributes::PbrShaderAttributes>;
  }  // ctor

  /**
   * @brief Creates an instance of a PBR shader configuration file used to
   * govern which calculations are performed by the PBR shader (such as
   * Image-based Lighting and/or specific PBR layer calculations), as well as to
   * provide some user-configurable shader-wide control values.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param pbrConfigFilename The location of the PBR shader configuration file
   * to parse. Defaults to the file location
   * `ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH` set by cmake.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the physics simulation meta data object parsed from
   * the specified configuration file.
   */
  attributes::PbrShaderAttributes::ptr createObject(
      const std::string& pbrConfigFilename =
          ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::PbrShaderAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This will set all the @ref metadata::attributes::PbrShaderAttributes
   * to have IBL either on or off.
   */
  void setAllIBLEnabled(bool isIblEnabled) {
    for (const auto& val : this->objectLibrary_) {
      // Don't change system default
      if (val.first.find(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH) ==
          std::string::npos) {
        this->getObjectByHandle(val.first)->setEnableIBL(isIblEnabled);
      }
    }
  }  // PbrShaderAttributesManager::setAllIBLEnabled

  /**
   * @brief This will set all the @ref metadata::attributes::PbrShaderAttributes
   * to have Direct Ligthing either on or off.
   */
  void setAllDirectLightsEnabled(bool isDirLightEnabled) {
    for (const auto& val : this->objectLibrary_) {
      // Don't change system default
      if (val.first.find(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH) ==
          std::string::npos) {
        this->getObjectByHandle(val.first)->setEnableDirectLighting(
            isDirLightEnabled);
      }
    }
  }  // PbrShaderAttributesManager::setAllDirectLightsEnabled

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig Whether this PbrShaderAttributes is being built
   * from a config file (i.e. handleName is the name of a configuration file) or
   * from some other source.
   * @return Newly created but unregistered PbrShaderAttributes pointer,
   * with only default values set.
   */
  attributes::PbrShaderAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      CORRADE_UNUSED bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called @ref
   * esp::core::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::PbrShaderAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param pbrShaderConfigurationTemplate The attributes template.
   * @param pbrShaderConfigurationHandle The key for referencing the template in
   * the
   * @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditionalE
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::PbrShaderAttributes::ptr pbrShaderConfigurationTemplate,
      const std::string& pbrShaderConfigurationHandle,
      CORRADE_UNUSED bool forceRegistration) override {
    // adds template to library, and returns either the ID of the existing
    // template referenced by pbrShaderConfigurationHandle, or the next
    // available ID if not found.
    int pbrConfigId =
        this->addObjectToLibrary(std::move(pbrShaderConfigurationTemplate),
                                 pbrShaderConfigurationHandle);
    return pbrConfigId;
  }  // PbrShaderAttributesManager::registerObjectFinalize

  /**
   * @brief Any physics-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  // instance vars

 public:
  ESP_SMART_POINTERS(PbrShaderAttributesManager)

};  // PbrShaderAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_PBRSHADERATTRIBUTESMANAGER_H_
