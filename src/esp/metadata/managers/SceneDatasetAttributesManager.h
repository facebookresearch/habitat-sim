// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_SCENEDATASETATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_SCENEDATASETATTRIBUTEMANAGER_H_

#include <utility>

#include "PbrShaderAttributesManager.h"
#include "PhysicsAttributesManager.h"

#include "AttributesManagerBase.h"
#include "esp/metadata/attributes/SceneDatasetAttributes.h"

namespace esp {
namespace metadata {
namespace managers {
using esp::core::managedContainers::ManagedObjectAccess;

class SceneDatasetAttributesManager
    : public AttributesManager<attributes::SceneDatasetAttributes,
                               ManagedObjectAccess::Share> {
 public:
  explicit SceneDatasetAttributesManager(
      PhysicsAttributesManager::ptr physicsAttributesMgr,
      PbrShaderAttributesManager::ptr pbrShaderAttributesMgr);
  /**
   * @brief Creates an instance of a dataset template described by passed
   * string. For dataset templates, this a file name.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired dataset template
   * to be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the newly-created template.
   */
  attributes::SceneDatasetAttributes::ptr createObject(
      const std::string& attributesTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::SceneDatasetAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This will set the current physics manager attributes that is
   * governing the world that this SceneDatasetAttributesManager's datasets will
   * be created in.  This is used so that upon creation of new
   * esp::metadata::attributes::SceneDatasetAttributes, PhysicsManagerAttributes
   * defaults can be set in the
   * esp::metadata::attributes::SceneDatasetAttributes before any scene-specific
   * values are set.
   *
   * @param handle The string handle referencing the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes governing the current
   * @ref esp::physics::PhysicsManager.
   */
  void setCurrPhysicsManagerAttributesHandle(const std::string& handle) {
    physicsManagerAttributesHandle_ = handle;
    for (const auto& val : this->objectLibrary_) {
      this->getObjectByHandle(val.first)->setPhysicsManagerHandle(handle);
    }
  }  // SceneDatasetAttributesManager::setCurrPhysicsManagerAttributesHandle

  /**
   * @brief This will set the current default PBR/IBL Shader configuration
   * attributes. This is used so that upon creation of new
   * @ref esp::metadata::attributes::SceneDatasetAttributes, the default @ref
   * esp::metadata::attributes::PbrShaderAttributes can be set in the
   * @ref esp::metadata::attributes::SceneDatasetAttributes before any
   * scene-specific values are set.
   *
   * @param pbrHandle The string handle referencing the @ref
   * esp::metadata::attributes::PbrShaderAttributes used to configure the
   * current PBR/IBL shader for all objects, unless overridden in Scene
   * Instances.
   */
  void setDefaultPbrShaderAttributesHandle(const std::string& pbrHandle) {
    defaultPbrShaderAttributesHandle_ = pbrHandle;
    for (const auto& val : this->objectLibrary_) {
      this->getObjectByHandle(val.first)->setDefaultPbrShaderAttrHandle(
          pbrHandle);
    }
  }  // SceneDatasetAttributesManager::setDefaultPbrShaderAttributesHandle

 protected:
  /**
   * @brief This will load a dataset map with file location values from the
   * dataset config.  It will also attempt to either verify those locations are
   * valid files, or else prefix the given location with the dataset root
   * directory.
   * @param dsDir the dataset's root directory
   * @param jsonTag the appropriate tag for the map being read
   * @param jsonConfig the json configuration file being read
   * @param map A ref to the dataset's map that is being populated.
   */
  void loadAndValidateMap(const std::string& dsDir,
                          const std::string& jsonTag,
                          const io::JsonGenericValue& jsonConfig,
                          std::map<std::string, std::string>& map);

  /**
   * @brief Verify a particular subcell exists within the
   * dataset_config.JSON file, and if so, handle reading the possible JSON
   * sub-cells it might hold, using the passed attributesManager for the
   * dataset being processed.
   * @tparam the type of the attributes manager.
   * @param dsDir The root directory of the dataset attributes being built.
   * @param tag The name of the JSON cell being processed - corresponds to
   * what type of data is being loaded from dataset configuration (i.e.
   * stages, objects, etc)
   * @param jsonConfig The sub cell in the json document being processed.
   * @param attrMgr The dataset's attributes manager for @p tag 's data.
   */
  template <typename U>
  void readDatasetJSONCell(const std::string& dsDir,
                           const char* tag,
                           const io::JsonGenericValue& jsonConfig,
                           const U& attrMgr);

  /**
   * @brief This will parse an individual element in a "configs" cell array in
   * the dataset_config.JSON file.
   * @tparam the type of the attributes manager.
   * @param tag The name of the JSON cell being processed - corresponds to what
   * type of data is being loaded from dataset configuration (i.e. stages,
   * objects, etc)
   * @param jCell The sub cell within the "configs" array in the json
   * document being processed.
   * @param attrMgr The dataset's attributes manager for @p tag 's data.
   */
  template <typename U>
  void readDatasetConfigsJSONCell(const std::string& dsDir,
                                  const char* tag,
                                  const io::JsonGenericValue& jCell,
                                  const U& attrMgr);

  /**
   * @brief Used Internally.  Create and configure newly-created dataset
   * attributes with any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to dataset attributes
   * @param builtFromConfig Whether this SceneDatasetAttributes is being built
   * from a config file (i.e. handleName is the name of a configuration file) or
   * from some other source.
   * @return Newly created but unregistered SceneDatasetAttributes pointer, with
   * only default values set.
   */
  attributes::SceneDatasetAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

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
   * @brief Any dataset-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Add a @ref std::shared_ptr<attributesType> object to the
   * @ref objectLibrary_.  Verify that render and collision handles have been
   * set properly.  We are doing this since these values can be modified by the
   * user.
   *
   * @param SceneDatasetAttributes The attributes template.
   * @param SceneDatasetAttributesHandle The key for referencing the template in
   * the @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of the registered template.
   */
  int registerObjectFinalize(
      attributes::SceneDatasetAttributes::ptr SceneDatasetAttributes,
      const std::string& SceneDatasetAttributesHandle,
      CORRADE_UNUSED bool forceRegistration) override;

  /**
   * @brief Name of currently used physicsManagerAttributes
   */
  std::string physicsManagerAttributesHandle_ = "";

  /**
   * @brief Name of currently used default PbrShaderAttributes
   */
  std::string defaultPbrShaderAttributesHandle_ = "";
  /**
   * @brief Reference to PhysicsAttributesManager to give access to default
   * physics manager attributes settings when
   * esp::metadata::attributes::SceneDatasetAttributes are created within
   * Dataset.
   */
  PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

  /**
   * @brief Reference to the PbrShaderAttributesManager to give access to
   * various PBR/IBL Shader configuration parameters and settings.
   */
  PbrShaderAttributesManager::ptr pbrShaderAttributesManager_ = nullptr;

 public:
  ESP_SMART_POINTERS(SceneDatasetAttributesManager)
};  // class SceneDatasetAttributesManager
}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SCENEDATASETATTRIBUTEMANAGER_H_
