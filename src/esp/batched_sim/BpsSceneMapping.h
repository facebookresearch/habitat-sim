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

  std::pair<int, int> findMeshIndexMaterialIndex(const std::string& meshName,
                                                 const std::string& mtrlName);

  std::tuple<int, int, float> findMeshIndexMaterialIndexScale(
      const std::string& nodeName);

  std::vector<std::string> materials;
  std::vector<std::string> meshes;
};

esp::io::JsonGenericValue toJsonValue(const BpsSceneMapping& x,
                                      esp::io::JsonAllocator& allocator);

bool fromJsonValue(const esp::io::JsonGenericValue& obj, BpsSceneMapping& x);

}  // namespace batched_sim
}  // namespace esp

#endif
