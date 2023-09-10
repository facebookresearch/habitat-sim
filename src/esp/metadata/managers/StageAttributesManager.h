// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_STAGEATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_STAGEATTRIBUTEMANAGER_H_

#include "AbstractObjectAttributesManager.h"
#include "esp/metadata/attributes/StageAttributes.h"

#include "PhysicsAttributesManager.h"

namespace esp {
namespace assets {
enum class AssetType;
}  // namespace assets
namespace metadata {
namespace managers {
using esp::core::managedContainers::ManagedObjectAccess;

class StageAttributesManager
    : public AbstractObjectAttributesManager<attributes::StageAttributes,
                                             ManagedObjectAccess::Copy> {
 public:
  explicit StageAttributesManager(
      PhysicsAttributesManager::ptr physicsAttributesManager);

  /**
   * @brief This will set the current physics manager attributes that is
   * governing the world that this StageAttributesManager's scenes will be
   * created in.  This is used so that upon creation of new
   * esp::metadata::attributes::StageAttributes, PhysicsManagerAttributes
   * defaults can be set in the esp::metadata::attributes::StageAttributes
   * before any scene-specific values are set.
   *
   * @param handle The string handle referencing the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes governing the current
   * @ref esp::physics::PhysicsManager.
   */
  void setCurrPhysicsManagerAttributesHandle(const std::string& handle) {
    physicsManagerAttributesHandle_ = handle;
  }

  /**
   * @brief copy current @ref esp::sim::SimulatorConfiguration driven values as
   * defaults, to make them available for stage attributes initialization.
   *
   * @param lightSetup the config-specified light setup
   * @param frustumCulling whether or not (semantic) stage should be
   * partitioned for culling.
   */
  void setCurrCfgVals(const std::string& lightSetup, bool frustumCulling) {
    // set lightsetup default from configuration
    cfgLightSetup_ = lightSetup;
    // set frustum culling default from configuration
    cfgFrustumCulling_ = frustumCulling;

  }  // StageAttributesManager::setCurrCfgVals

  /**
   * @brief Creates an instance of a stage template described by passed
   * string, which should be a reference to an existing primitive asset template
   * to be used in the construction of the stage (as render and collision
   * mesh). It returns existing instance if there is one, and nullptr if fails.
   *
   * @param primAttrTemplateHandle The handle to an existing primitive asset
   * template. Fails if does not.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true.
   * @return a reference to the desired stage template, or nullptr if fails.
   */
  attributes::StageAttributes::ptr createPrimBasedAttributesTemplate(
      const std::string& primAttrTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::StageAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

 protected:
  /**
   * @brief Create and save default primitive asset-based object templates,
   * saving their handles as non-deletable default handles.
   */
  void createDefaultPrimBasedAttributesTemplates() override;

  /**
   * @brief Perform file-name-based attributes initialization. This is to
   * take the place of the AssetInfo::fromPath functionality, and is only
   * intended to provide default values and other help if certain mistakes
   * are made by the user, such as specifying an asset handle in json but not
   * specifying the asset type corresponding to that handle.  These settings
   * should not restrict anything, only provide defaults.
   *
   * @param attributes The AbstractObjectAttributes object to be configured
   * @param setFrame whether the frame should be set or not (only for render
   * assets in scenes)
   * @param meshHandle Mesh Handle to check.
   * @param assetTypeSetter Setter for mesh type.
   */
  void setDefaultAssetNameBasedAttributes(
      attributes::StageAttributes::ptr attributes,
      bool setFrame,
      const std::string& meshHandle,
      const std::function<void(int)>& assetTypeSetter) override;
  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig whether this stage is being built from a config file
   * (i.e. handleName is file name for config) or from some other source.
   * @return Newly created but unregistered StageAttributes pointer, with only
   * default values set.
   */
  attributes::StageAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally from @ref
   * esp::core::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Add a @ref std::shared_ptr<attributesType> object to the
   * @ref objectLibrary_.  Verify that render and collision handles have been
   * set properly.  We are doing this since these values can be modified by the
   * user.
   *
   * @param StageAttributesTemplate The attributes template.
   * @param StageAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */

  int registerObjectFinalize(
      attributes::StageAttributes::ptr StageAttributesTemplate,
      const std::string& StageAttributesHandle,
      bool forceRegistration) override;

  /**
   * @brief Any scene-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  // instance vars
  /**
   * @brief Reference to PhysicsAttributesManager to give access to default
   * physics manager attributes settings when
   * esp::metadata::attributes::StageAttributes are created.
   */
  PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

  /**
   * @brief Current lighting default value based on current @ref
   * esp::sim::SimulatorConfiguration settings. Potentially overridden by
   * scene-specific json.
   */
  std::string cfgLightSetup_;

  /**
   * @brief Current frustum culling setting based on current @ref
   * esp::sim::SimulatorConfiguration settings. Potentially overridden by
   * scene-specific json.
   */
  bool cfgFrustumCulling_ = false;

  /**
   * @brief Name of currently used physicsManagerAttributes
   */
  std::string physicsManagerAttributesHandle_ = "";

 public:
  ESP_SMART_POINTERS(StageAttributesManager)

};  // StageAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_STAGEATTRIBUTEMANAGER_H_
