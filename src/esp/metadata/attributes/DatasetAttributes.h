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
   * @brief Return manager for construction and access to scene attributes.
   */
  const managers::StageAttributesManager::ptr getStageAttributesManager()
      const {
    return stageAttributesManager_;
  }

 protected:
  /**
   * @brief Reference to AssetAttributesManager to give access to primitive
   * attributes for object construction
   */
  managers::AssetAttributesManager::ptr assetAttributesManager_ = nullptr;
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

 public:
  ESP_SMART_POINTERS(DatasetAttributes)
};  // class DatasetAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_DATASETATTRIBUTES_H_