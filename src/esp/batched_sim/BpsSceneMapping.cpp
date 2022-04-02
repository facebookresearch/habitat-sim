// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BpsSceneMapping.h"

#include "esp/io/json.h"
#include "esp/core/Check.h"

namespace esp {
namespace batched_sim {

BpsSceneMapping BpsSceneMapping::loadFromFile(const std::string& filepath) {
  BpsSceneMapping mapping;
  auto newDoc = esp::io::parseJsonFile(filepath);
  esp::io::readMember(newDoc, "mapping", mapping);
  return mapping;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping::MeshMapping& x) {
  esp::io::readMember(obj, "name", x.name);
  esp::io::readMember(obj, "meshIdx", x.meshIdx);
  esp::io::readMember(obj, "mtrlIdx", x.mtrlIdx);
  return true;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping& x) {
  esp::io::readMember(obj, "meshMappings", x.meshMappings);
  return true;
}

BpsSceneMapping::InstanceBlueprint BpsSceneMapping::findInstanceBlueprint(
    const std::string& nodeName) const {
  
  ESP_CHECK(!nodeName.empty(), "findInstanceBlueprint: empty input name. Check your episode data."); 

  for (const auto& meshMapping : meshMappings) {
    if (meshMapping.name == nodeName) {
      constexpr float scale = 1.f;
      return InstanceBlueprint{meshMapping.meshIdx, meshMapping.mtrlIdx};
    }
  }

  ESP_CHECK(false, "findInstanceBlueprint: no mapping found for render asset "
    << nodeName << ". For every free object and stage, we expect to find a "
    << "corresponding render asset with the same name. See meshMappings names in your "
    << ".bps.mapping.json file for valid render asset names.");

  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

}  // namespace batched_sim
}  // namespace esp
