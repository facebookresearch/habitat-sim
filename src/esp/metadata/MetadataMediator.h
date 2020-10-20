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
#include "esp/metadata/managers/DatasetAttributesManager.h"
#include "esp/metadata/managers/LightAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace esp {
namespace metadata {
namespace Attrs = esp::metadata::attributes;
class MetadataMediator {
 public:
  MetadataMediator(const std::string& _defaultDataset = "default")
      : activeDataset_(_defaultDataset) {
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
   * @brief Creates a dataset attributes using @p datasetName, and registers it.
   * NOTE If an existing dataset attributes exists with this handle, then this
   * will exit with a message unless @p overwrite is true.
   * @param datasetName The name of the dataset to load or create.
   * @param overwrite Whether to overwrite an existing dataset or not
   * @return Whether successfully created a new dataset or not.
   */
  bool createDataset(const std::string& datasetName, bool overwrite = true);

  /**
   * @brief Sets default dataset attributes, if it exists already.  If it does
   * not exist, it will attempt to load a dataset_config.json with the given
   * name.  If none exists it will create an "empty" dataset attributes and give
   * it the passed name.
   * @param datasetName the name of the existing dataset to use as default, or a
   * json file describing the desired dataset attributes, or some handle to use
   * for an empty dataset.
   * @return whether successful or not
   */
  bool setActiveDatasetName(const std::string& datasetName);
  /**
   * @brief Returns the name of the current default dataset
   */
  std::string getActiveDatasetName() const { return activeDataset_; }

  /**
   * @brief Return manager for construction and access to asset attributes for
   * current dataset.
   */
  const managers::AssetAttributesManager::ptr getAssetAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getAssetAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   */
  const managers::LightAttributesManager::ptr getLightAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getLightAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   */
  const managers::ObjectAttributesManager::ptr getObjectAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getObjectAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to stage attributes for
   * current dataset.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
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

  //==================== Accessors ======================//

 protected:
  /**
   * @brief Retrieve the current default dataset object.  Currently only for
   * internal use.
   */
  Attrs::DatasetAttributes::ptr getActiveDSAttribs() const {
    // do not get copy of dataset attributes until datasetAttributes deep copy
    // ctor implemented
    auto datasetAttr =
        datasetAttributesManager_->getObjectByHandle(activeDataset_);
    if (nullptr == datasetAttr) {
      LOG(ERROR)
          << "MetadataMediator::getActiveDSAttribs : Unknown dataset named "
          << activeDataset_ << ". Aborting";
      return nullptr;
    }
    return datasetAttr;
  }  // MetadataMediator::getActiveDSAttribs

  /**
   * @brief String name of current, default dataset.
   */
  std::string activeDataset_;
  /**
   * @brief Manages all construction and access to asset attributes.
   */
  managers::DatasetAttributesManager::ptr datasetAttributesManager_ = nullptr;

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
