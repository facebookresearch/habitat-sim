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
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace esp {
namespace metadata {
namespace Attrs = esp::metadata::attributes;
class MetadataMediator {
 public:
  MetadataMediator();
  ~MetadataMediator() {}

  /**
   * @brief This function will build the various @ref
   * esp::metadata::managers::AttributesManager s this mediator will manage.
   */
  void buildAttributesManagers();

  /**
   * @brief Return manager for construction and access to asset attributes for
   * current dataset.
   */
  const managers::AssetAttributesManager::ptr getAssetAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getDefaultDataset();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getAssetAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to object attributes for
   * current dataset.
   */
  const managers::ObjectAttributesManager::ptr getObjectAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getDefaultDataset();
    return (nullptr == datasetAttr) ? nullptr
                                    : datasetAttr->getObjectAttributesManager();
  }

  /**
   * @brief Return manager for construction and access to stage attributes for
   * current dataset.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    Attrs::DatasetAttributes::ptr datasetAttr = getDefaultDataset();
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
  Attrs::DatasetAttributes::ptr getDefaultDataset() const {
    // do not get copy of dataset attributes until datasetAttributes deep copy
    // ctor implemented
    auto datasetAttr =
        datasetAttributesManager_->getObjectByHandle(defaultDataset_);
    if (nullptr == datasetAttr) {
      LOG(ERROR)
          << "MetadataMediator::getDefaultDataset : Unknown dataset named "
          << defaultDataset_ << ". Aborting";
      return nullptr;
    }
    return datasetAttr;
  }  // MetadataMediator::getDefaultDataset

  /**
   * @brief String name of current, default dataset.
   */
  std::string defaultDataset_;
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
