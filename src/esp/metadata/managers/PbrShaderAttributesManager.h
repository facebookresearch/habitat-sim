// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_PBRSHADERATTRIBUTESMANAGER_H_
#define ESP_METADATA_MANAGERS_PBRSHADERATTRIBUTESMANAGER_H_

#include <utility>

#include "AbstractAttributesManager.h"

#include "esp/gfx/configure.h"
#include "esp/metadata/attributes/PbrShaderAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {

class PbrShaderAttributesManager
    : public AbstractAttributesManager<attributes::PbrShaderAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  PbrShaderAttributesManager()
      : AbstractAttributesManager<attributes::PbrShaderAttributes,
                                  ManagedObjectAccess::Copy>::
            AbstractAttributesManager("PBR Rendering", "pbr_config.json") {
    this->copyConstructorMap_["PbrShaderAttributes"] =
        &PbrShaderAttributesManager::createObjCopyCtorMapEntry<
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
    auto objIterPair = this->getObjectLibIterator();
    for (auto& objIter = objIterPair.first; objIter != objIterPair.second;
         ++objIter) {
      const std::string objHandle = objIter->first;
      // Don't change system default
      if (objHandle.find(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH) ==
          std::string::npos) {
        this->getObjectByHandle(objHandle)->setEnableIBL(isIblEnabled);
      }
    }
  }  // PbrShaderAttributesManager::setAllIBLEnabled

  /**
   * @brief This will set all the @ref metadata::attributes::PbrShaderAttributes
   * to have Direct Ligthing either on or off.
   */
  void setAllDirectLightsEnabled(bool isDirLightEnabled) {
    auto objIterPair = this->getObjectLibIterator();
    for (auto& objIter = objIterPair.first; objIter != objIterPair.second;
         ++objIter) {
      const std::string objHandle = objIter->first;
      // Don't change system default
      if (objHandle.find(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH) ==
          std::string::npos) {
        this->getObjectByHandle(objHandle)->setEnableDirectLighting(
            isDirLightEnabled);
      }
    }
  }  // PbrShaderAttributesManager::setAllDirectLightsEnabled

  /**
   * @brief This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   *
   * TODO : If/When we begin treating IBL filepaths like we do other paths, this
   * will need to be implemented.
   * @param pbrShaderAttribs The attributes to be filtered.
   */
  void finalizeAttrPathsBeforeRegister(
      CORRADE_UNUSED const attributes::PbrShaderAttributes::ptr&
          pbrShaderAttribs) const override;

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
   * AbstractAttributesManager-specific upon template removal, such as removing
   * a specific template handle from the list of file-based template handles in
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
   * @brief This method will perform any essential updating to the managed
   * object before registration is performed. If this updating fails,
   * registration will also fail.
   * @param pbrShaderAttribs the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Should register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(
      attributes::PbrShaderAttributes::ptr pbrShaderAttribs,
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool forceRegistration) override;

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
