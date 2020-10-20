// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_DATASETATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_DATASETATTRIBUTES_H_

/** @file
 * @brief Class @ref esp::metadata::DatasetMetadata.  This class will hold
 * relevant data and configurations for a specific dataset.
 */

#include "AttributesBase.h"

#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/LightAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace esp {
namespace metadata {
namespace attributes {
class DatasetAttributes : public AbstractAttributes {
 public:
  DatasetAttributes(const std::string& datasetName,
                    const managers::PhysicsAttributesManager::ptr physAttrMgr);
  /**
   * @brief Return manager for construction and access to asset attributes.
   */
  const managers::AssetAttributesManager::ptr getAssetAttributesManager()
      const {
    return assetAttributesManager_;
  }
  /**
   * @brief Return manager for construction and access to object attributes.
   */
  const managers::ObjectAttributesManager::ptr getObjectAttributesManager()
      const {
    return objectAttributesManager_;
  }
  /**
   * @brief Return manager for construction and access to light attributes.
   */
  const managers::LightAttributesManager::ptr getLightAttributesManager()
      const {
    return lightAttributesManager_;
  }
  /**
   * @brief Return manager for construction and access to scene attributes.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    return stageAttributesManager_;
  }

  /**
   * @brief Return the map for navmesh file locations
   */
  const std::map<std::string, std::string>& getNavmeshMap() const {
    return navmeshMap_;
  }

  /**
   * @brief Return the map for semantic scene descriptor file locations
   */
  const std::map<std::string, std::string>& getSemanticSceneDescrMap() const {
    return semanticSceneDescrMap_;
  }

  /**
   * @brief copy current @ref esp::sim::SimulatorConfiguration driven values,
   * such as file paths, to make them available for stage attributes defaults.
   *
   * @param filepaths the map of file paths from the configuration object
   * @param lightSetup the config-specified light setup
   * @param frustrumCulling whether or not (semantic) stage should be
   * partitioned for culling.
   */
  void setCurrCfgVals(const std::map<std::string, std::string>& filepaths,
                      const std::string& lightSetup,
                      bool frustrumCulling) {
    stageAttributesManager_->setCurrCfgVals(filepaths, lightSetup,
                                            frustrumCulling);
  }

  /**
   * @brief Set the name of the attributes used for the physics manager that
   * governs this Dataset
   */
  void setPhysicsManagerHandle(const std::string& physMgrAttrHandle) {
    setString("physMgrAttrHandle", physMgrAttrHandle);
    stageAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        physMgrAttrHandle);
  }
  std::string getPhysicsManagerHandle() const {
    return getString("physMgrAttrHandle");
  }

 protected:
  /**
   * @brief Reference to AssetAttributesManager to give access to primitive
   * attributes for object construction
   */
  managers::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to light attributes from this
   * dataset.
   */
  managers::LightAttributesManager::ptr lightAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to object attributes from this
   * dataset.
   */
  managers::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;
  /**
   * @brief Manages all construction and access to scene attributes from this
   * dataset.
   */
  managers::StageAttributesManager::ptr stageAttributesManager_ = nullptr;
  /**
   * @brief Maps names specified in dataset_config file to paths for navmeshes.
   */
  std::map<std::string, std::string> navmeshMap_;
  /**
   * @brief Maps names specified in dataset_config file to paths for semantic
   * scene descriptor files
   */
  std::map<std::string, std::string> semanticSceneDescrMap_;

 public:
  ESP_SMART_POINTERS(DatasetAttributes)
};  // class DatasetAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_DATASETATTRIBUTES_H_
