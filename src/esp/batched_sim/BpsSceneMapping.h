// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_BPSSCENEMAPPING_H_
#define ESP_BATCHEDSIM_BPSSCENEMAPPING_H_

#include "esp/io/JsonAllTypes.h"

namespace esp {
namespace batched_sim {

class BpsSceneMapping {
 public:
  static BpsSceneMapping loadFromFile(const std::string& filepath);

  std::tuple<int, int, float> findMeshIndexMaterialIndexScale(
      const std::string& nodeName);

  struct MeshMapping {
      std::string name;
      int meshIdx;
      int mtrlIdx;
  };
  std::vector<MeshMapping> meshMappings;
};

bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping::MeshMapping& x);
bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping& x);

}  // namespace batched_sim
}  // namespace esp

#endif
