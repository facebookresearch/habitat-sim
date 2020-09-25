// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataMediator.h"

namespace esp {
namespace metadata {

MetadataMediator::MetadataMediator() {
  defaultDataset_ = "default";
  buildAttributesManagers();
}
void MetadataMediator::buildAttributesManagers() {
  physicsAttributesManager_ = managers::PhysicsAttributesManager::create();
  datasetAttributesManager_ =
      managers::DatasetAttributesManager::create(physicsAttributesManager_);
  // create blank default attributes manager
}

}  // namespace metadata
}  // namespace esp
