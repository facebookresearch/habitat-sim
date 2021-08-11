// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_METADDATAMEDIATOR_H_
#define ESP_METADATA_METADDATAMEDIATOR_H_

/** @file
 * @brief Class @ref esp::metadata::MetadataMediator
 */

#include "esp/core/Configuration.h"

#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/LightLayoutAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/SceneDatasetAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"
#include "esp/sim/SimulatorConfiguration.h"

namespace esp {

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
   * handle, then this will exit with a message unless @p overwrite is true.
   * @param sceneDatasetName The name of the dataset to load or create.
   * @param overwrite Whether to overwrite an existing dataset or not
   * @return Whether successfully created a new dataset or not.
   */
  bool createSceneDataset(const std::string& sceneDatasetName,
                          bool overwrite = false);

  /**
   * @brief Load a physics manager attributes defined by passed string file path
   * @param _physicsManagerAttributesPath The path to look for the physics
   * config file.
   * @return Whether successfully created a new physics manager attributes or
   * not.
   */
  bool createPhysicsManagerAttributes(
      const std::string& _physicsManagerAttributesPath =
          ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH);

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
   * @brief Sets desired physics manager attributes handle.  Will load if does
   * not exist.
   * @param _physicsManagerAttributesPath The path to look for the physics
   * config file.
   * @return whether successful or not
   */
  bool setCurrPhysicsAttributesHandle(
      const std::string& _physicsManagerAttributesPath);
  /**
   * @brief Returns the name of the currently used physics manager attributes
   */
  std::string getCurrPhysicsAttributesHandle() {
    return currPhysicsManagerAttributes_;
  }

  /**
   * @brief Return manager for construction and access to asset attributes for
   * current dataset.
   * @return The current dataset's @ref managers::AssetAttributesManager::ptr,
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
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   * @return The current dataset's @ref managers::ObjectAttributesManager::ptr,
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
   * @brief Return manager for construction and access to scene instance
   * attributes for current dataset.
   * @return The current dataset's @ref managers::SceneAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::SceneAttributesManager::ptr& getSceneAttributesManager() {
    return getActiveDSAttribs()->getSceneAttributesManager();
  }  // MetadataMediator::getSceneAttributesManager

  /**
   * @brief Return manager for construction and access to stage attributes for
   * current dataset.
   * @return The current dataset's @ref managers::StageAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::StageAttributesManager::ptr& getStageAttributesManager() {
    return getActiveDSAttribs()->getStageAttributesManager();
  }  // MetadataMediator::getStageAttributesManager

  /**
   * @brief Return a copy of the current physics manager attributes.
   */
  attributes::PhysicsManagerAttributes::ptr
  getCurrentPhysicsManagerAttributes() {
    return physicsAttributesManager_->getObjectCopyByHandle(
        currPhysicsManagerAttributes_);
  }  // getCurrentPhysicsManagerAttributes

  /**
   * @brief Get a list of all scene instances available in the currently active
   * dataset
   */
  std::vector<std::string> getAllSceneInstanceHandles() {
    return getActiveDSAttribs()
        ->getSceneAttributesManager()
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
   * @brief Returns an appropriate scene instance attributes corresponding to
   * the passed sceneID/name.  For back-compat, this function needs to manage
   * various conditions pertaining to the passed name.  It will always return a
   * valid SceneInstanceAttributes for the current active dataset.
   * @param sceneName A string representation of the desired
   * SceneInstanceAttributes.  May only correspond to a stage on disk, in which
   * case a new SceneInstanceAttributes will be constructed and properly
   * populated with the appropriate data.
   * @return A valid SceneInstanceAttributes - registered in current dataset,
   * with all references also registered in current dataset.
   */
  attributes::SceneAttributes::ptr getSceneAttributesByName(
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
   * @brief TEMPORARY get a constant reference to the articulated object model
   * filenames (.urdf) that have been loaded.  Once ArticulatedModelMangaer is
   * built, this will be accomplished using Managed Container functionality.
   */
  const std::map<std::string, std::string>&
  getArticulatedObjectModelFilenames() {
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
   * removal of locked attributes.  If @p datasetName references @ref
   * activeSceneDataset_ then will fail, returning false.
   *
   * @param sceneDatasetName The name of the SceneDatasetAttributes to remove.
   * @return whether successful or not.
   */
  bool removeSceneDataset(const std::string& sceneDatasetName);

  /**
   * @brief Checks if passed handle exists as scene dataset.
   * @param sceneDatasetName The name of the SceneDatasetAttributes to remove.
   * @return whether successful or not.
   */
  inline bool sceneDatasetExists(const std::string& sceneDatasetName) const {
    return sceneDatasetAttributesManager_->getObjectLibHasHandle(
        sceneDatasetName);
  }

  /**
   * @brief Returns the createRenderer flag that was set in the associated
   * SimulatorConfiguration.
   * @return the boolean flag.
   */
  bool getCreateRenderer() const;

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
      const std::string& msgString) {
    if (assetMapping.count(assetHandle) == 0) {
      ESP_WARNING() << msgString << ": Unable to find file path for"
                    << assetHandle << ".  Aborting.";
      return "";
    }
    return assetMapping.at(assetHandle);
  }  // getFilePathForHandle

  /**
   * @brief This will create a new, empty @ref SceneAttributes with the passed
   * name, and create a SceneObjectInstance for the stage also using the passed
   * name. It is assuming that the dataset has the stage registered, and that
   * the calling function will register the created SceneInstance with the
   * dataset.  This method will also register navmesh and scene descriptor file
   * paths that are synthesized for newly made SceneAttributes. TODO: get rid of
   * these fields in stageAttributes.
   *
   * @param datasetAttr The current dataset attributes
   * @param stageAttributes Readonly version of stage to use to synthesize scene
   * instance.
   * @param dsSceneAttrMgr The current dataset's SceneAttributesManager
   * @param sceneName The name for the scene and also the stage within the
   * scene.
   * @return The created SceneAttributes, with the stage's SceneInstanceObject
   * to be intialized to reference the stage also named with @p sceneName .
   */
  attributes::SceneAttributes::ptr makeSceneAndReferenceStage(
      const attributes::SceneDatasetAttributes::ptr& datasetAttr,
      const attributes::StageAttributes::ptr& stageAttributes,
      const managers::SceneAttributesManager::ptr& dsSceneAttrMgr,
      const std::string& sceneName);

  /**
   * @brief This function will build the @ref managers::PhysicsAttributesManager
   * and @ref managers::SceneDatasetAttributesManager this mediator will manage.
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
    // this should never happen - there will always be a dataset with the name
    // activeSceneDataset_
    if (datasetAttr == nullptr) {
      ESP_ERROR() << "Unable to set active dataset due to Unknown dataset named"
                  << activeSceneDataset_
                  << "so changing dataset to \"default\".";
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
   * @brief String name of current Physis Manager attributes
   */
  std::string currPhysicsManagerAttributes_;
  /**
   * @brief Manages all construction and access to all scene dataset attributes.
   */
  managers::SceneDatasetAttributesManager::ptr sceneDatasetAttributesManager_ =
      nullptr;

  /**
   * @brief Manages all construction and access to physics world attributes.
   */
  managers::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

 public:
  ESP_SMART_POINTERS(MetadataMediator)
};  // namespace metadata

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_METADDATAMEDIATOR_H_
