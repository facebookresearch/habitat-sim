// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SCENEDATASETATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SCENEDATASETATTRIBUTES_H_

/** @file
 * @brief Class @ref esp::metadata::DatasetMetadata.  This class will hold
 * relevant data and configurations for a specific dataset.
 */

#include "AbstractAttributes.h"

#include "esp/metadata/managers/AOAttributesManager.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/LightLayoutAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/SceneInstanceAttributesManager.h"
#include "esp/metadata/managers/SemanticAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace esp {
namespace metadata {
namespace attributes {
using esp::core::managedContainers::ManagedContainerBase;

class SceneDatasetAttributes : public AbstractAttributes {
 public:
  SceneDatasetAttributes(
      const std::string& datasetName,
      const managers::PhysicsAttributesManager::ptr& physAttrMgr);

  ~SceneDatasetAttributes() override {
    // These are to clear any external refs persisting after this scene dataset
    // is deleted.  Accessing these refs should result in errors instead of
    // leaks/undefined behavior.
    artObjAttributesManager_ = nullptr;
    assetAttributesManager_ = nullptr;
    lightLayoutAttributesManager_ = nullptr;
    objectAttributesManager_ = nullptr;
    sceneInstanceAttributesManager_ = nullptr;
    semanticAttributesManager_ = nullptr;
    stageAttributesManager_ = nullptr;
    navmeshMap_.clear();
  }

  /**
   * @brief Return manager for construction and access to articulated object
   * attributes.
   */
  const managers::AOAttributesManager::ptr& getAOAttributesManager() const {
    return artObjAttributesManager_;
  }

  /**
   * @brief Return manager for construction and access to asset attributes.
   */
  const managers::AssetAttributesManager::ptr& getAssetAttributesManager()
      const {
    return assetAttributesManager_;
  }

  /**
   * @brief Return manager for construction and access to light attributes.
   */
  const managers::LightLayoutAttributesManager::ptr&
  getLightLayoutAttributesManager() const {
    return lightLayoutAttributesManager_;
  }

  /**
   * @brief Return manager for construction and access to object attributes.
   */
  const managers::ObjectAttributesManager::ptr& getObjectAttributesManager()
      const {
    return objectAttributesManager_;
  }

  /**
   * @brief Return manager for construction and access to scene attributes.
   */
  const managers::SceneInstanceAttributesManager::ptr&
  getSceneInstanceAttributesManager() const {
    return sceneInstanceAttributesManager_;
  }

  /**
   * @brief Return manager for construction and access to semantic attributes
   * for current dataset.
   * @return A shared pointer to the current dataset's @ref esp::metadata::managers::SemanticAttributesManager
   */
  const managers::SemanticAttributesManager::ptr& getSemanticAttributesManager()
      const {
    return semanticAttributesManager_;
  }  // MetadataMediator::getSemantticAttributesManager

  /**
   * @brief Return manager for construction and access to stage attributes.
   */
  const managers::StageAttributesManager::ptr& getStageAttributesManager()
      const {
    return stageAttributesManager_;
  }

  /**
   * @brief Retrieve the shader type to use for the various default materials,
   * either Phong of PBR
   */
  ObjectInstanceShaderType getDefaultMaterialShaderType() const {
    return get<bool>("default_material_is_pbr")
               ? ObjectInstanceShaderType::PBR
               : ObjectInstanceShaderType::Phong;
  }

  /**
   * @brief Set whether to use PBR or Phong for the default material values
   * defined in resource Manager.
   */
  void setDefaultMaterialIsPBR(bool default_material_is_pbr) {
    set("default_material_is_pbr", default_material_is_pbr);
  }

  /**
   * @brief Return the map for navmesh file locations
   */
  const std::map<std::string, std::string>& getNavmeshMap() const {
    return navmeshMap_;
  }

  /**
   * @brief Only SceneDatasetAttributesManager should directly edit navemesh and
   * semantic scene descriptor maps. Return the map for navmesh file locations
   * for building/modification
   */
  std::map<std::string, std::string>& editNavmeshMap() { return navmeshMap_; }

  /**
   * @brief Add an entry to the @ref navmeshMap_ with the passed key.  If @p overwrite
   * then overwrite existing entry, otherwise will modify key and add value with
   * modified key.  Returns pair of added Key-Value.
   * @param key The handle for the navmesh path to add
   * @param path The path to the navmesh asset to add
   * @param overwrite Whether to overwrite existing entries or not
   * @return Key-Value pair for path being added.
   */
  std::pair<std::string, std::string> addNavmeshPathEntry(
      const std::string& key,
      const std::string& path,
      bool overwrite = false) {
    return addNewValToMap(key, path, overwrite, navmeshMap_, "<navmesh>");
  }  // addNavmeshPathEntry

  /**
   * @brief Find or create a @ref esp::metadata::attributes::SemanticAttributes
   * entry with the passed key. If
   * @p overwrite then overwrite existing entry, otherwise will modify key and
   * add values from stage attributes with modified key. Returns the string
   * handle of the semantic attributes.
   * @param semanticHandle The handle for the SemanticSceneDescr attributes to
   * reference or add.
   * @param stageAttributes The Stage attributes
   * @return Handle in the SemanticAttributesManager referencing the desired
   * @ref esp::metadata::attributes::SemanticAttributes
   */
  std::string addSemanticSceneDescrPathEntry(
      const std::string& semanticHandle,
      const attributes::StageAttributes::ptr& stageAttributes);

  /**
   * @brief Create or modify @ref esp::metadata::attributes::SemanticAttributes
   * corresponding to the keys in the passed map to hold the map's values as the
   * semantic scene descriptor file.
   * @param semanticPathnameMap Map of keys corresponding to the keys that will
   * references these semantic attributes in the scene instance and the
   * filepaths to the appropriate SSD files.
   */
  void setSemanticAttrSSDFilenames(
      const std::map<std::string, std::string>& semanticPathnameMap);

  /**
   * @brief copy current @ref esp::sim::SimulatorConfiguration driven
   * values, such as file paths, to make them available for stage attributes
   * defaults.
   *
   * @param lightSetup the config-specified light setup
   * @param frustumCulling whether or not (semantic) stage should be
   * partitioned for culling.
   */
  void setCurrCfgVals(const std::string& lightSetup, bool frustumCulling) {
    stageAttributesManager_->setCurrCfgVals(lightSetup, frustumCulling);
  }

  /**
   * @brief Set the name of the attributes used for the physics manager that
   * governs this Dataset
   */
  void setPhysicsManagerHandle(const std::string& physMgrAttrHandle) {
    set("physMgrAttrHandle", physMgrAttrHandle);
    stageAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        physMgrAttrHandle);
  }
  std::string getPhysicsManagerHandle() const {
    return get<std::string>("physMgrAttrHandle");
  }

  /**
   * @brief Set the name of the attributes used for the default Pbr/Ibl shader
   * configuration.
   */
  void setDefaultPbrShaderAttrHandle(
      const std::string& dfltPbrShaderAttrHandle) {
    set("defaultPbrShaderAttrHandle", dfltPbrShaderAttrHandle);
    sceneInstanceAttributesManager_->setDefaultPbrShaderAttrHandle(
        dfltPbrShaderAttrHandle);
  }
  /**
   * @brief Set the current scene's mapping from 'region' tags to
   * PbrShaderAttributes handles.
   */
  void setCurrScenePbrShaderAttrMappings(
      std::map<std::string, std::string> mappings) {
    currPbrShaderAttrRegionMap_ = std::move(mappings);
  }

  /**
   * @brief retrieve the handle to the PbrShaderAttributes that corresponds to
   * the passed region handle defined in the SceneInstanceAttributes.
   */
  std::string getCurrPbrShaderHandleFromRegion(const std::string& region) {
    if (currPbrShaderAttrRegionMap_.count(region) > 0) {
      return currPbrShaderAttrRegionMap_.at(region);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "No region with handle `" << region
          << "` so returning empty string";
      return "";
    }
  }

  /**
   * @brief Get the name of the attributes used for the default Pbr/Ibl shader
   * configuration.
   */
  std::string getDefaultPbrShaderAttrHandle() const {
    return get<std::string>("defaultPbrShaderAttrHandle");
  }

  /**
   * @brief Add the passed @p sceneInstance to the dataset, verifying that all
   * the attributes and assets references in the scene instance exist, and if
   * so adding them.  This is to handle the addition of an existing
   * sceneInstance that might reference stages, objects, navmeshes, etc. that
   * do not exist in the dataset.
   * @param sceneInstance A pointer to a @ref metadata::attributes::SceneInstanceAttributes.  It is assumed the @p
   * sceneInstance at least references a valid stage.
   * @return whether this sceneInstance was successfully added to the dataset.
   */
  bool addNewSceneInstanceToDataset(
      const attributes::SceneInstanceAttributes::ptr& sceneInstance);

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
      const std::string& stageAttrName);

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
      const std::string& objAttrName);

  /**
   * @brief Returns articulated object attributes corresponding to passed handle
   * as substring. Assumes articulated object attributes with @p artObjAttrName
   * as substring exists in current dataset.
   * @param artObjAttrName substring to handle of articulated object instance
   * attributes that exists in current active dataset. The attributes will be
   * found via substring search, so the name is expected to be sufficiently
   * restrictive to have exactly 1 match in dataset.
   * @return smart pointer to articulated object attributes if exists, nullptr
   * otherwise.
   */
  attributes::ArticulatedObjectAttributes::ptr
  getNamedArticulatedObjectAttributesCopy(const std::string& artObjAttrName);

  /**
   * @brief Returns a lightsetup object configured by the attributes whose
   * handle contains the passed @p lightSetupName
   * @param lightSetupName Name of the attributes to be used to build the
   * lightsetup.  The attributes will be found via substring search, so the name
   * is expected to be sufficiently restrictive to have exactly 1 match in
   * dataset.
   * @return the lightsetup corresponding to @p lightSetupName.
   */
  esp::gfx::LightSetup getNamedLightSetup(const std::string& lightSetupName);

  /**
   * @brief Returns stage attributes handle in dataset corresponding to passed
   * name as substring. Assumes stage attributes with @p stageAttrName as
   * substring exists in this dataset.
   * @param stageAttrName substring to handle of stage instance attributes that
   * exists in this dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return name of stage attributes with handle containing @p stageAttrName ,
   * or empty string if none.
   */
  inline std::string getStageAttrFullHandle(const std::string& stageAttrName) {
    return stageAttributesManager_->getFullAttrNameFromStr(stageAttrName);
  }  // getStageAttrFullHandle

  /**
   * @brief Returns object attributes handle in dataset corresponding to passed
   * name as substring. Assumes object attributes with @p objAttrName as
   * substring exists in this dataset.
   * @param objAttrName substring to handle of object instance attributes that
   * exists in this dataset. The attributes will be found via
   * substring search, so the name is expected to be sufficiently restrictive to
   * have exactly 1 match in dataset.
   * @return name of object attributes with handle containing @p objAttrName or
   * empty string if none.
   */
  inline std::string getObjAttrFullHandle(const std::string& objAttrName) {
    return objectAttributesManager_->getFullAttrNameFromStr(objAttrName);
  }  // getObjAttrFullHandle

  /**
   * @brief Returns articulated object model file handle in dataset
   * corresponding to passed name as substring. Assumes articulated object model
   * with @p artObjModelName as substring exists in this dataset.
   * @param artObjModelName substring to handle of AO model that exists in this
   * dataset. The actual model name will be found via substring search in the
   * manager, so the name is expected to be sufficiently restrictive to have
   * exactly 1 match in dataset.
   * @return name of AO model with handle containing @p artObjModelName or
   * empty string if none.
   */
  inline std::string getArticulatedObjModelFullHandle(
      const std::string& artObjModelName) {
    return artObjAttributesManager_->getFullAttrNameFromStr(artObjModelName);
    // std::map<std::string, std::string> articulatedObjPaths;
  }

  /**
   * @brief This is to be deprecated. Provide a map of the articulated object
   * model filenames (.urdf) that have been referenced in the Scene Dataset via
   * paths, either .urdf or .json. To be removed in favor of directly accessing
   * these values through the AOAttributesMaanager.
   */
  std::map<std::string, std::string> getArticulatedObjectModelFilenames()
      const {
    return artObjAttributesManager_->getArticulatedObjectModelFilenames();
  }

  /**
   * @brief Returns the full name of the lightsetup attributes whose
   * handle contains the passed @p lightSetupName
   * @param lightSetupName Name of the attributes desired.  The attributes will
   * be found via substring search, so the name is expected to be sufficiently
   * restrictive to have exactly 1 match in dataset.
   * @return the full attributes name corresponding to @p lightSetupName , or
   * predefined No-Light and Default_lighting key strings.
   */
  inline std::string getLightSetupFullHandle(
      const std::string& lightSetupName) {
    if ((lightSetupName == DEFAULT_LIGHTING_KEY) ||
        (lightSetupName == NO_LIGHT_KEY)) {
      return lightSetupName;
    }
    return lightLayoutAttributesManager_->getFullAttrNameFromStr(
        lightSetupName);
  }  // getLightSetupFullHandle

  /**
   * @brief return a summary of this dataset
   */
  std::string getDatasetSummary() const;

  /**
   * @brief returns the header row of the summary string.
   */
  static std::string getDatasetSummaryHeader();

 protected:
  /**
   * @brief will create a new @ref esp::metadata::attributes::SemanticAttributes
   * with the given handle if one does not exist.
   * @param semanticHandle The name of the attributes to create.
   * @param dbgSourceAttribs The name of the caller, for debug purposes should
   * this fail.
   */
  void createSemanticAttribsFromDS(const std::string& semanticHandle,
                                   const std::string& dbgSourceAttribs);

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.  Individual
   * components handle this.
   */

  std::string getObjectInfoHeaderInternal() const override { return ","; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

  /**
   * @brief This will add a navmesh entry or a semantic scene descriptor entry
   * to the appropriate map.  It checks if a value already exists at the
   * specified @p key , and if not adds the @p path to the map at the given key
   * value.  If a value does exist at the specified key, it checks if it is the
   * same value as @p path . If so nothing is done, but if not, then it is
   * either overwritten or the key is modified until an available key is found,
   * which is then used to add the new entry, depending on @p overwrite .
   *
   * @param key The key to attempt to add the new value at.  This key may be
   * modified if collisions are found and @p overwrite is false.
   * @param path The location of the desired asset being added to the path.
   * @param overwrite Whether to overwrite a found, existing entry at @p key .
   * @param map The map to modify
   * @param descString The calling method, to provide context for log messages.
   * @return the key-value pair that is actually added to the map.
   */
  std::pair<std::string, std::string> addNewValToMap(
      const std::string& key,
      const std::string& path,
      bool overwrite,
      std::map<std::string, std::string>& map,
      const std::string& descString);

  /**
   * @brief Manages all construction and access to articulated object attributes
   * from this dataset.
   */
  managers::AOAttributesManager::ptr artObjAttributesManager_ = nullptr;

  /**
   * @brief Reference to AssetAttributesManager to give access to primitive
   * attributes for object construction
   */
  managers::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to light attributes from this
   * dataset.
   */
  managers::LightLayoutAttributesManager::ptr lightLayoutAttributesManager_ =
      nullptr;

  /**
   * @brief Manages all construction and access to object attributes from this
   * dataset.
   */
  managers::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to @ref metadata::attributes::SceneInstanceAttributes
   * from this dataset.
   */
  managers::SceneInstanceAttributesManager::ptr
      sceneInstanceAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to @ref metadata::attributes::SemanticAttributes
   * from this dataset.
   */
  managers::SemanticAttributesManager::ptr semanticAttributesManager_ = nullptr;
  /**
   * @brief Manages all construction and access to stage attributes from this
   * dataset.
   */
  managers::StageAttributesManager::ptr stageAttributesManager_ = nullptr;
  /**
   * @brief Maps names specified in dataset_config file to paths for
   * navmeshes.
   */
  std::map<std::string, std::string> navmeshMap_;

  /**
   * list of key-value pairs of region names to PbrShaderConfigs
   */
  std::map<std::string, std::string> currPbrShaderAttrRegionMap_;

 public:
  ESP_SMART_POINTERS(SceneDatasetAttributes)
};  // class SceneDatasetAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SCENEDATASETATTRIBUTES_H_
