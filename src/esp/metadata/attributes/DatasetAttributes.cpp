// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

DatasetAttributes::DatasetAttributes(
    const std::string& datasetName,
    const managers::PhysicsAttributesManager::ptr physAttrMgr)
    : AbstractAttributes("DatasetAttributes", datasetName) {
  assetAttributesManager_ = managers::AssetAttributesManager::create();
  objectAttributesManager_ = managers::ObjectAttributesManager::create();
  objectAttributesManager_->setAssetAttributesManager(assetAttributesManager_);
  stageAttributesManager_ = managers::StageAttributesManager::create(
      objectAttributesManager_, physAttrMgr);
}  // ctor

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
