// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_UTILS_DATATOOL_SCENELOADER_H_
#define ESP_UTILS_DATATOOL_SCENELOADER_H_

#include <string>

#include <Corrade/PluginManager/Manager.h>
#include <Magnum/Trade/Trade.h>

#include "esp/assets/Asset.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/MeshData.h"

namespace esp {
namespace assets {

class SceneLoader {
  using Importer = Magnum::Trade::AbstractImporter;

 public:
  SceneLoader();
  MeshData load(const AssetInfo& info);

 private:
  Corrade::PluginManager::Manager<Importer> importerManager_;
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_UTILS_DATATOOL_SCENELOADER_H_
