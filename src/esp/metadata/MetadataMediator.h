// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_METADDATAMEDIATOR_H_
#define ESP_METADATA_METADDATAMEDIATOR_H_

/** @file
 * @brief Class @ref esp::metadata::MetadataMediator
 */

#include "esp/core/Configuration.h"

#include "esp/metadata/managers/AOAttributesManager.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/LightLayoutAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PbrShaderAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/SceneDatasetAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"
#include "esp/sim/SimulatorConfiguration.h"

namespace esp {

namespace core {
class Configuration;
}

namespace sim {
struct SimulatorConfiguration;
}

namespace metadata {

class MetadataMediator {
 public:
  explicit MetadataMediator(const sim::SimulatorConfiguration& cfg);

  MetadataMediator() : MetadataMediator(sim::SimulatorConfiguration{}) {}
  ~MetadataMediator() = default;

  /**
   * @brief Set current @ref esp::sim::SimulatorConfiguration to be used.
   * @param cfg Current configuration being used by Simulator.
   * @return Whether config was set properly.
   */
  bool setSimulatorConfiguration(const sim::SimulatorConfiguration& cfg);

  /**
   * @brief Return the current @ref esp::sim::SimulatorConfiguration this
   * MetadataMediator is using.  Used to build Simulator from existing MM.
   */
  const sim::SimulatorConfiguration& getSimulatorConfiguration() const {
    return this->simConfig_;
  }

  /**
   * @brief Creates a dataset attributes using @p sceneDatasetName, and
   * registers it. NOTE If an existing dataset attributes exists with this
   * handle, then this will only overwrite this existing dataset if @p overwrite
   * is set to true.
   * @param sceneDatasetName The name of the dataset to load or create.
   * @param overwrite Whether to overwrite an existing dataset or not
   * @return Whether a dataset with @p sceneDatasetName exists (either new or
   * pre-existing).
   */
  bool createSceneDataset(const std::string& sceneDatasetName,
                          bool overwrite = false);

  /**
   * @brief Load a @ref esp::metadata::attributes::PhysicsManagerAttributes
   * defined by passed string file path.
   * @param _physicsManagerAttributesPath The path to look for the physics
   * config file.
   * @return Whether a @ref esp::metadata::attributes::PhysicsManagerAttributes
   * exists based on the requested path, either new or pre-existing.
   */
  bool createPhysicsManagerAttributes(
      const std::string& _physicsManagerAttributesPath =
          ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH);

  /**
   * @brief Load PBR/IBL shader configuration attributes defined by file at
   * passed path.
   * @param _pbrConfigPath The path to look for the PBR/IBL shader config file
   * @return Whether successfully created a new PBR/IBL shader configuration
   * attributes or not.
   */
  bool createPbrAttributes(const std::string& _pbrConfigPath =
                               ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH);
  //==================== Accessors ======================//

  /**
   * @brief Sets default dataset attributes, if it exists already.  If it does
   * not exist, it will attempt to load a dataset_config.json with the given
   * name.  If none exists it will create an "empty" dataset attributes and give
   * it the passed name.
   * @param sceneDatasetName the name of the existing dataset to use as default,
   * or a json file describing the desired dataset attributes, or some handle to
   * use for an empty dataset.
   * @return whether successful or not
   */
  bool setActiveSceneDatasetName(const std::string& sceneDatasetName);

  /**
   * @brief Returns the name of the current active dataset
   */
  std::string getActiveSceneDatasetName() const { return activeSceneDataset_; }

  /**
   * @brief Sets desired @ref esp::metadata::attributes::PhysicsManagerAttributes
   * handle.  Will load if does not exist.
   * @param physicsManagerAttributesPath The path to look for the physics
   * config file.
   * @return whether successful or not
   */
  bool setCurrPhysicsAttributesHandle(
      const std::string& physicsManagerAttributesPath);
  /**
   * @brief Returns the name of the currently used
   * @ref esp::metadata::attributes::PhysicsManagerAttributes
   */
  std::string getCurrPhysicsAttributesHandle() const {
    return currPhysicsManagerAttributes_;
  }

  /**
   * @brief Sets the handle of the desired default @ref esp::metadata::attributes::PbrShaderAttributes
   * unless overridden in scene instances.  Will load if does not exist.
   * @param pbrShaderAttributesPath The path to look for the physics
   * config file.
   * @return whether successful or not
   */
  bool setCurrDefaultPbrAttributesHandle(
      const std::string& pbrShaderAttributesPath);
  /**
   * @brief Returns the name of the currently specified default
   * @ref esp::metadata::attributes::PbrShaderAttributes.
   * This can be overridden on a per-scene instance basis.
   */
  std::string getCurrDefaultPbrAttributesHandle() const {
    return currDefaultPbrAttributesHandle_;
  }

  /**
   * @brief Return manager for construction and access to asset attributes for
   * current dataset.
   * @return The current dataset's @ref esp::metadata::managers::AssetAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::AssetAttributesManager::ptr& getAssetAttributesManager() {
    return getActiveDSAttribs()->getAssetAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   * @return The current dataset's @ref
   * managers::LightLayoutAttributesManager::ptr, or nullptr if no current
   * dataset.
   */
  const managers::LightLayoutAttributesManager::ptr&
  getLightLayoutAttributesManager() {
    return getActiveDSAttribs()->getLightLayoutAttributesManager();
  }
  /**
   * @brief Return manager for construction and access to articulated object
   * attributes for current dataset.
   * @return The current dataset's @ref esp::metadata::managers::AOAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::AOAttributesManager::ptr& getAOAttributesManager() {
    return getActiveDSAttribs()->getAOAttributesManager();
  }
  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   * @return The current dataset's @ref esp::metadata::managers::ObjectAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::ObjectAttributesManager::ptr& getObjectAttributesManager() {
    return getActiveDSAttribs()->getObjectAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to physics world
   * attributes.
   */
  const managers::PhysicsAttributesManager::ptr& getPhysicsAttributesManager()
      const {
    return physicsAttributesManager_;
  }  // getPhysicsAttributesManager

  /**
   * @brief Return manager for PBR and IBL lighting configuration settings.
   */
  const managers::PbrShaderAttributesManager::ptr&
  getPbrShaderAttributesManager() const {
    return pbrShaderAttributesManager_;
  }  // getPbrShaderAttributesManager

  /**
   * @brief Return manager for construction and access to
   * @ref esp::attributes::SceneInstanceAttributes for current dataset.
   * @return The current dataset's @ref
   * managers::SceneInstanceAttributesManager::ptr, or nullptr if no current
   * dataset.
   */
  const managers::SceneInstanceAttributesManager::ptr&
  getSceneInstanceAttributesManager() {
    return getActiveDSAttribs()->getSceneInstanceAttributesManager();
  }  // MetadataMediator::getSceneInstanceAttributesManager

  /**
   * @brief Return manager for construction and access to stage attributes for
   * current dataset.
   * @return The current dataset's @ref esp::metadata::managers::StageAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::StageAttributesManager::ptr& getStageAttributesManager() {
    return getActiveDSAttribs()->getStageAttributesManager();
  }  // MetadataMediator::getStageAttributesManager

  /**
   * @brief Return a copy of the current
   * @ref esp::metadata::attributes::PhysicsManagerAttributes.
   */
  attributes::PhysicsManagerAttributes::ptr
  getCurrentPhysicsManagerAttributes() {
    return physicsAttributesManager_->getObjectCopyByHandle(
        currPhysicsManagerAttributes_);
  }  // getCurrentPhysicsManagerAttributes

  /**
   * @brief Return a copy of the currently specified PBR/IBL Shader
   * configuration attributes.
   */
  attributes::PbrShaderAttributes::ptr getCurrentPbrConfiguration() {
    return pbrShaderAttributesManager_->getObjectCopyByHandle(
        currDefaultPbrAttributesHandle_);
  }

  /**
   * @brief Get a list of all scene instances available in the currently active
   * dataset
   */
  std::vector<std::string> getAllSceneInstanceHandles() {
    return getActiveDSAttribs()
        ->getSceneInstanceAttributesManager()
        ->getObjectHandlesBySubstring();
  }

  /**
   * @brief Get a list of all stage attributes available in the currently active
   * dataset. This is useful if current dataset does not have scene instances
   * defined, and instead builds them on the fly for each stage.
   */
  std::vector<std::string> getAllStageAttributesHandles() {
    return getActiveDSAttribs()
        ->getStageAttributesManager()
        ->getObjectHandlesBySubstring();
  }

  /**
   * @brief Return copy of map of current active dataset's navmesh handles.
   */
  std::map<std::string, std::string> getActiveNavmeshMap() {
    return std::map<std::string, std::string>(
        getActiveDSAttribs()->getNavmeshMap());
  }  // getActiveNavmeshMap

  /**
   * @brief Return the file path of the specified navmesh in the current active
   * dataset
   * @param navMeshHandle The dataset library handle of the navmesh
   * @return The file path of the navmesh.
   */
  std::string getNavmeshPathByHandle(const std::string& navMeshHandle) {
    return getFilePathForHandle(navMeshHandle,
                                getActiveDSAttribs()->getNavmeshMap(),
                                "<getNavmeshPathByHandle>");

  }  // MetadataMediator::getNavmeshPathByHandle

  /**
   * @brief Return copy of map of current active dataset's semantic scene
   * descriptor handles.
   */
  std::map<std::string, std::string> getActiveSemanticSceneDescriptorMap() {
    return std::map<std::string, std::string>(
        getActiveDSAttribs()->getSemanticSceneDescrMap());
  }  // getActiveSemanticSceneDescriptorMap

  /**
   * @brief Return the file path of the specified semantic scene descriptor in
   * the current active dataset
   * @param ssDescrHandle The dataset library handle of the semantic scene
   * descriptor
   * @return The file path of the semantic scene descriptor.
   */
  std::string getSemanticSceneDescriptorPathByHandle(
      const std::string& ssDescrHandle) {
    return getFilePathForHandle(
        ssDescrHandle, getActiveDSAttribs()->getSemanticSceneDescrMap(),
        "<getSemanticSceneDescriptorPathByHandle>");

  }  // MetadataMediator::getNavMeshPathByHandle

  /**
   * @brief Returns an appropriate @ref esp::metadata::attributes::SceneInstanceAttributes
   * corresponding to the passed sceneID/name.  For back-compat, this function
   * needs to manage various conditions pertaining to the passed name.  It will
   * always return a valid SceneInstanceAttributes for the current active
   * dataset.
   *
   * @param sceneName A string representation of the desired
   * SceneInstanceAttributes.  May only correspond to a stage on disk, in which
   * case a new SceneInstanceAttributes will be constructed and properly
   * populated with the appropriate data.
   * @return A valid SceneInstanceAttributes - registered in current dataset,
   * with all references also registered in current dataset.
   */
  attributes::SceneInstanceAttributes::ptr getSceneInstanceAttributesByName(
      const std::string& sceneName);

  /**
   * @brief Returns stage attributes corresponding to passed handle as
   * substring. Assumes stage attributes with @p stageAttrName as substring
   * exists in current dataset.
   * @param stageAttrName substring to handle of stage instance attributes that
   * exists in current active dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return smart pointer to stage attributes if exists, nullptr otherwise.
   */
  attributes::StageAttributes::ptr getNamedStageAttributesCopy(
      const std::string& stageAttrName) {
    return getActiveDSAttribs()->getNamedStageAttributesCopy(stageAttrName);
  }  // getNamedStageAttributesCopy

  /**
   * @brief Returns object attributes corresponding to passed handle as
   * substring. Assumes object attributes with @p objAttrName as substring
   * exists in current dataset.
   * @param objAttrName substring to handle of object instance attributes that
   * exists in current active dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return smart pointer to object attributes if exists, nullptr otherwise.
   */
  attributes::ObjectAttributes::ptr getNamedObjectAttributesCopy(
      const std::string& objAttrName) {
    return getActiveDSAttribs()->getNamedObjectAttributesCopy(objAttrName);
  }  // getNamedObjectAttributesCopy

  /**
   * @brief Returns a lightsetup object configured by the attributes whose
   * handle contains the passed @p lightSetupName
   * @param lightSetupName Name of the attributes to be used to build the
   * lightsetup.  The attributes will be found via substring search, so the name
   * is expected to be sufficiently restrictive to have exactly 1 match in
   * dataset.
   * @return the lightsetup corresponding to @p lightSetupName.
   */
  esp::gfx::LightSetup getNamedLightSetup(const std::string& lightSetupName) {
    return getActiveDSAttribs()->getNamedLightSetup(lightSetupName);
  }  // getNamedLightSetup

  /**
   * @brief Returns stage attributes handle in dataset corresponding to passed
   * name as substring. Assumes stage attributes with @p stageAttrName as
   * substring exists in current dataset.
   * @param stageAttrName substring to handle of stage instance attributes that
   * exists in current active dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return name of stage attributes with handle containing @p stageAttrName ,
   * or empty string if none.
   */
  std::string getStageAttrFullHandle(const std::string& stageAttrName) {
    return getActiveDSAttribs()->getStageAttrFullHandle(stageAttrName);
  }  // getStageAttrFullHandle

  /**
   * @brief Returns object attributes handle in dataset corresponding to passed
   * name as substring. Assumes object attributes with @p objAttrName as
   * substring exists in current dataset.
   * @param objAttrName substring to handle of object instance attributes that
   * exists in current active dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return name of object attributes with handle containing @p objAttrName or
   * empty string if none.
   */
  std::string getObjAttrFullHandle(const std::string& objAttrName) {
    return getActiveDSAttribs()->getObjAttrFullHandle(objAttrName);
  }  // getObjAttrFullHandle

  /**
   * @brief Returns articulated object model handle in dataset corresponding to
   * passed name as substring. Assumes articulated object model with @p
   * artObjModelName as substring exists in this dataset.
   * @param artObjModelName substring to handle of AO model that exists in this
   * dataset. The actual model name will be found via substring search in the
   * manager, so the name is expected to be sufficiently restrictive to have
   * exactly 1 match in dataset.
   * @return name of object attributes with handle containing @p artObjModelName
   * or empty string if none.
   */
  std::string getArticulatedObjModelFullHandle(
      const std::string& artObjModelName) {
    return getActiveDSAttribs()->getArticulatedObjModelFullHandle(
        artObjModelName);
  }  // getArticulatedObjModelFullHandle

  /**
   * @brief This is to be deprecated. Provide a map of the articulated object
   * model filenames (.urdf) that have been referenced in the Scene Dataset via
   * paths, either .urdf or .json. To be removed in favor of directly accessing
   * these values through the AOAttributesMaanager.
   */
  std::map<std::string, std::string> getArticulatedObjectModelFilenames() {
    return getActiveDSAttribs()->getArticulatedObjectModelFilenames();
  }

  /**
   * @brief Returns the full name of the lightsetup attributes whose
   * handle contains the passed @p lightSetupName
   * @param lightSetupName Name of the attributes desired.  The attributes will
   * be found via substring search, so the name is expected to be sufficiently
   * restrictive to have exactly 1 match in dataset.
   * @return name of light setup with handle containing @p lightSetupName or
   * empty string if none.
   */
  std::string getLightSetupFullHandle(const std::string& lightSetupName) {
    return getActiveDSAttribs()->getLightSetupFullHandle(lightSetupName);
  }  // getLightSetupFullHandle

  /**
   * @brief Allow removal of the named @ref
   * esp::metadata::attributes::SceneDatasetAttributes.  Will silently force
   * removal of locked attributes. If @p datasetName references @ref
   * activeSceneDataset_ then will fail, returning false.
   *
   * @param sceneDatasetName The name of the @ref esp::metadata::attributes::SceneDatasetAttributes
   * to remove.
   * @return whether successful or not.
   */
  bool removeSceneDataset(const std::string& sceneDatasetName);

  /**
   * @brief Checks if passed handle exists as scene dataset.
   * @param sceneDatasetName The name of the @ref esp::metadata::attributes::SceneDatasetAttributes
   * to check for.
   * @return whether @ref esp::metadata::attributes::SceneDatasetAttributes exists.
   */
  inline bool sceneDatasetExists(const std::string& sceneDatasetName) const {
    return sceneDatasetAttributesManager_->getObjectLibHasHandle(
        sceneDatasetName);
  }

  /**
   * @brief Returns the createRenderer flag that was set in the associated
   * @ref esp::sim::SimulatorConfiguration.
   * @return the boolean flag.
   */
  bool getCreateRenderer() const { return simConfig_.createRenderer; }

  /**
   * @brief This function returns a list of all the scene datasets currently
   * loaded, along with some key statistics for each, formatted as a
   * comma-separated string.
   * @return a vector of strings holding scene dataset info.
   */
  std::string getDatasetsOverview() const;

  /**
   * @brief this function will create a report of the contents of the scene
   * dataset with the passed name. If no name is provided, a report on the
   * current active dataset will be returned.
   * @param sceneDataset The name of the scene dataset to perform the report on.
   * @return Comma-separated string of data describing the desired dataset.
   */
  std::string createDatasetReport(const std::string& sceneDataset = "") const;

  /**
   * @brief Return the root-level user defined attributes configuration for the
   * specified scene instance.
   * @param sceneName The scene name in the currently loaded SceneDataset.
   * @return The scene instance user-defined configuration.
   */
  std::shared_ptr<esp::core::config::Configuration>
  getSceneInstanceUserConfiguration(const std::string& curSceneName);

  /**
   * @brief Retrieve copies of all the @ref esp::metadata::attributes::PbrShaderAttributes
   * currently defined. These will be used to pre-load/pre-derive any BLUTs and
   * IBL cubemaps used once on dataset load.
   */
  std::unordered_map<std::string,
                     esp::metadata::attributes::PbrShaderAttributes::ptr>
  getAllPbrShaderConfigs() const {
    return pbrShaderAttributesManager_->getObjectsByHandleSubstring("");
  }

  /**
   * @brief Retrieve a copy of the named @ref esp::metadata::attributes::PbrShaderAttributes.
   * @param handle the name of the config we wish to retrieve, or the default if
   * not found
   */
  esp::metadata::attributes::PbrShaderAttributes::ptr getPbrShaderConfig(
      const std::string& handle) const {
    auto results =
        pbrShaderAttributesManager_->getObjectsByHandleSubstring(handle);
    if (results.size() > 0) {
      return results.begin()->second;
    } else {
      ESP_WARNING() << "\t WARNING! No config for key :" << handle
                    << "so returning default";
      return getDefaultPbrShaderConfig();
    }
  }

  esp::metadata::attributes::PbrShaderAttributes::ptr
  getPbrShaderConfigByRegion(const std::string& region) {
    std::string pbrConfigHandle =
        getActiveDSAttribs()->getCurrPbrShaderHandleFromRegion(region);
    return getPbrShaderConfig(pbrConfigHandle);
  }

  /**
   * @brief Set the current scene's mapping from 'region' tags to
   * PbrShaderAttributes handles.
   */
  void setCurrScenePbrShaderRegionMap(
      std::map<std::string, std::string> mappings) {
    getActiveDSAttribs()->setCurrScenePbrShaderAttrMappings(
        std::move(mappings));
  }

  /**
   * @brief Retrieve a copy of the named @ref esp::metadata::attributes::PbrShaderAttributes.
   * @param handle the name of the config we wish to retrieve.
   */
  esp::metadata::attributes::PbrShaderAttributes::ptr
  getDefaultPbrShaderConfig() const {
    return pbrShaderAttributesManager_
        ->getObjectsByHandleSubstring(currDefaultPbrAttributesHandle_)
        .at(currDefaultPbrAttributesHandle_);
  }

  /**
   * @brief This will set the IBL state for every configuration in the current
   * pbrShaderAttributesManager_.
   */
  void setAllPbrShaderConfigIBLState(bool isIblEnabled) {
    pbrShaderAttributesManager_->setAllIBLEnabled(isIblEnabled);
  }

 protected:
  /**
   * @brief Return the file path corresponding to the passed handle in the
   * current active dataset
   * @param assetHandle The dataset library handle of the desired asset
   * @param assetMapping The mappings to use to get the asset file path.
   * @param msgString A message string to describe any issues encountered,
   * consisting of who called this function.
   * @return The file path of the asset.
   */
  std::string getFilePathForHandle(
      const std::string& assetHandle,
      const std::map<std::string, std::string>& assetMapping,
      const std::string& msgString);

  /**
   * @brief This will create a new, empty @ref esp::metadata::attributes::SceneInstanceAttributes
   * with the passed name, and create a SceneObjectInstance for the stage also
   * using the passed name. It is assuming that the dataset has the stage
   * registered, and that the calling function will register the created
   * SceneInstance with the dataset.  This method will also register navmesh
   * and scene descriptor file paths that are synthesized for newly made
   * SceneInstanceAttributes. TODO: get rid of these fields in
   * stageAttributes.
   *
   * @param datasetAttr The current dataset attributes
   * @param stageAttributes Readonly version of stage to use to synthesize
   * scene instance.
   * @param dsSceneAttrMgr The current dataset's
   * SceneInstanceAttributesManager
   * @param sceneName The name for the scene and also the stage within the
   * scene.
   * @return The created SceneInstanceAttributes, with the stage's
   * SceneInstanceObject to be intialized to reference the stage also named
   * with
   * @p sceneName .
   */
  attributes::SceneInstanceAttributes::ptr makeSceneAndReferenceStage(
      const attributes::SceneDatasetAttributes::ptr& datasetAttr,
      const attributes::StageAttributes::ptr& stageAttributes,
      const managers::SceneInstanceAttributesManager::ptr& dsSceneAttrMgr,
      const std::string& sceneName);

  /**
   * @brief This function will build the @ref esp::metadata::managers::PhysicsAttributesManager
   * and @ref esp::metadata::managers::SceneDatasetAttributesManager this mediator will manage.
   * This should only be called from constructor or reset (TODO).
   */
  void buildAttributesManagers();

  /**
   * @brief Retrieve the current default dataset object.  Currently only for
   * internal use.
   */
  attributes::SceneDatasetAttributes::ptr getActiveDSAttribs() {
    // do not get copy of dataset attributes
    auto datasetAttr =
        sceneDatasetAttributesManager_->getObjectByHandle(activeSceneDataset_);
    // this should never happen - there should always be a dataset with the
    // name activeSceneDataset_
    if (datasetAttr == nullptr) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Unable to set active Scene Dataset due to unknown dataset "
             "named `"
          << activeSceneDataset_
          << "` so changing Scene Dataset  to `default`.";
      activeSceneDataset_ = "default";

      datasetAttr = sceneDatasetAttributesManager_->getObjectByHandle(
          activeSceneDataset_);
    }
    return datasetAttr;
  }  // MetadataMediator::getActiveDSAttribs

  //================== Instance variables ==================//

  /**
   * @brief Current Simulator Configuration. A copy (not a ref) so that it can
   * exceed the lifespan of the source config from, for example, Simulator.
   */
  sim::SimulatorConfiguration simConfig_{};

  /**
   * @brief String name of current, default dataset.
   */
  std::string activeSceneDataset_;

  /**
   * @brief String name of current Physics Manager attributes
   */
  std::string currPhysicsManagerAttributes_;

  /**
   * @brief String name of current default PBR/IBL Shader configuration
   * attributes.
   */
  std::string currDefaultPbrAttributesHandle_;
  /**
   * @brief Manages all construction and access to all scene dataset
   * attributes. Users should never directly access this, or it could
   * inadvertently get in a broken and unrecoverable state.  All access to
   * SceneDatasetAttributes should be currated/governed by MetadataMediator.
   */
  managers::SceneDatasetAttributesManager::ptr sceneDatasetAttributesManager_ =
      nullptr;

  /**
   * @brief Manages all construction and access to physics world attributes.
   */
  managers::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

  /**
   * @brief Manages all PBR/IBL Shader configuration settings, independent of
   * loaded datasets.
   */
  managers::PbrShaderAttributesManager::ptr pbrShaderAttributesManager_ =
      nullptr;

 public:
  ESP_SMART_POINTERS(MetadataMediator)
};  // namespace metadata

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_METADDATAMEDIATOR_H_
