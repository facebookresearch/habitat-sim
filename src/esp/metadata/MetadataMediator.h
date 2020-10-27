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

namespace esp {
namespace metadata {
class MetadataMediator {
 public:
  MetadataMediator(const std::string& _defaultSceneDataset = "default")
      : activeSceneDataset_(_defaultSceneDataset) {
    buildAttributesManagers();
  }  // namespace metadata
  ~MetadataMediator() {}

  /**
   * @brief This function will build the @ref managers::PhysicsAttributesManager
   * and @ref managers::DatasetAttributeManager this mediator will manage.
   *
   * This will also attempt to build templates for the default physics manager
   * and dataset.
   */
  void buildAttributesManagers();

  /**
   * @brief Creates a dataset attributes using @p sceneDatasetName, and
   * registers it. NOTE If an existing dataset attributes exists with this
   * handle, then this will exit with a message unless @p overwrite is true.
   * @param sceneDatasetName The name of the dataset to load or create.
   * @param overwrite Whether to overwrite an existing dataset or not
   * @return Whether successfully created a new dataset or not.
   */
  bool createDataset(const std::string& sceneDatasetName,
                     bool overwrite = true);

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
   * @brief Returns the name of the current default dataset
   */
  std::string getActiveSceneDatasetName() const { return activeSceneDataset_; }

  /**
   * @brief Return manager for construction and access to asset attributes for
   * current dataset.
   * @return The current dataset's @ref managers::AssetAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::AssetAttributesManager::ptr getAssetAttributesManager()
      const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getAssetAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   * @return The current dataset's @ref
   * managers::LightLayoutAttributesManager::ptr, or nullptr if no current
   * dataset.
   */
  const managers::LightLayoutAttributesManager::ptr
  getLightLayoutAttributesManager() const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr)
               ? nullptr
               : datasetAttr->getLightLayoutAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   * @return The current dataset's @ref managers::ObjectAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::ObjectAttributesManager::ptr getObjectAttributesManager()
      const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getObjectAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to scene instance
   * attributes for current dataset.
   * @return The current dataset's @ref managers::SceneAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::SceneAttributesManager::ptr getSceneAttributesManager()
      const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getSceneAttributesManager();
  }  // MetadataMediator::getStageAttributesManager

  /**
   * @brief Return manager for construction and access to stage attributes for
   * current dataset.
   * @return The current dataset's @ref managers::StageAttributesManager::ptr,
   * or nullptr if no current dataset.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getStageAttributesManager();
  }  // MetadataMediator::getStageAttributesManager

  /**
   * @brief Return manager for construction and access to physics world
   * attributes.
   */
  const managers::PhysicsAttributesManager::ptr getPhysicsAttributesManager()
      const {
    return physicsAttributesManager_;
  }

  /**
   * @brief Return copy of map of current active dataset's navmesh handles.
   */
  const std::map<std::string, std::string> getActiveNavmeshMap() const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();

    if (nullptr == datasetAttr) {
      return std::map<std::string, std::string>();
    }
    return std::map<std::string, std::string>(datasetAttr->getNavmeshMap());
  }

  /**
   * @brief Return copy of map of current active dataset's semantic scene
   * descriptor handles.
   */
  const std::map<std::string, std::string> getActiveSemanticSceneDescriptorMap()
      const {
    attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();

    if (nullptr == datasetAttr) {
      return std::map<std::string, std::string>();
    }
    return std::map<std::string, std::string>(
        datasetAttr->getSemanticSceneDescrMap());
  }

  //==================== Accessors ======================//

 protected:
  /**
   * @brief Retrieve the current default dataset object.  Currently only for
   * internal use.
   */
  attributes::SceneDatasetAttributes::ptr getActiveDSAttribs() const {
    // do not get copy of dataset attributes until SceneDatasetAttributes deep
    // copy ctor implemented
    auto datasetAttr =
        sceneDatasetAttributesManager_->getObjectByHandle(activeSceneDataset_);
    if (nullptr == datasetAttr) {
      LOG(ERROR)
          << "MetadataMediator::getActiveDSAttribs : Unknown dataset named "
          << activeSceneDataset_ << ". Aborting";
      return nullptr;
    }
    return datasetAttr;
  }  // MetadataMediator::getActiveDSAttribs

  /**
   * @brief String name of current, default dataset.
   */
  std::string activeSceneDataset_;
  /**
   * @brief Manages all construction and access to asset attributes.
   */
  managers::SceneDatasetAttributesManager::ptr sceneDatasetAttributesManager_ =
      nullptr;

  /**
   * @brief Manages all construction and access to physics world attributes.
   */
  managers::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

 public:
  ESP_SMART_POINTERS(MetadataMediator)
};  // class MetadataMediator

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_METADDATAMEDIATOR_H_
