// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttributesBase.h"
#include "esp/physics/PhysicsObjectBase.h"
namespace esp {

namespace metadata {
namespace attributes {

// All keys must be lowercase
const std::map<std::string, esp::assets::AssetType> AssetTypeNamesMap = {
    {"unknown", esp::assets::AssetType::UNKNOWN},
    {"mp3d", esp::assets::AssetType::MP3D_MESH},
    {"semantic", esp::assets::AssetType::INSTANCE_MESH},
    {"navmesh", esp::assets::AssetType::NAVMESH},
};

std::string getMeshTypeName(esp::assets::AssetType meshTypeEnum) {
  // Must always be valid value
  for (const auto& it : AssetTypeNamesMap) {
    if (it.second == meshTypeEnum) {
      return it.first;
    }
  }
  return "unknown";
}

AbstractAttributes::AbstractAttributes(const std::string& attributesClassKey,
                                       const std::string& handle)
    : Configuration() {
  // set up an existing subgroup for user_defined attributes
  addSubgroup("user_defined");
  AbstractAttributes::setClassKey(attributesClassKey);
  AbstractAttributes::setHandle(handle);
  // set initial vals, will be overwritten when registered
  set("ID", 0);
  set("fileDirectory", "");
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
