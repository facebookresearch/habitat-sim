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

class MetadataMediator {
 public:
  MetadataMediator() { defaultDataset_ = "default"; }
  ~MetadataMediator() {}

  /**
   * @brief This function will build the various @ref
   * esp::metadata::managers::AttributesManager s this mediator will manage.
   */
  void buildAttributesManagers();

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
   * @brief Return manager for construction and access to physics world
   * attributes.
   */
  const managers::PhysicsAttributesManager::ptr getPhysicsAttributesManager()
      const {
    return physicsAttributesManager_;
  }
  /**
   * @brief Return manager for construction and access to scene attributes.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    return stageAttributesManager_;
  }

 protected:
  /**
   * @brief String name of current, default dataset.
   */
  std::string defaultDataset_;
  /**
   * @brief Manages all construction and access to asset attributes.
   */
  managers::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to object attributes.
   */
  managers::ObjectAttributesManager::ptr objectAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to physics world attributes.
   */
  managers::PhysicsAttributesManager::ptr physicsAttributesManager_ = nullptr;

  /**
   * @brief Manages all construction and access to scene attributes.
   */
  managers::StageAttributesManager::ptr stageAttributesManager_ = nullptr;

 public:
  ESP_SMART_POINTERS(MetadataMediator)
};  // class MetadataMediator

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_METADDATAMEDIATOR_H_
