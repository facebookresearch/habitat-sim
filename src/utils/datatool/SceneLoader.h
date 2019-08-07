// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <string>

#include "esp/assets/Asset.h"
#include "esp/assets/MeshData.h"

namespace esp {
namespace assets {

class SceneLoader {
 public:
  MeshData load(const AssetInfo& info);
};

}  // namespace assets
}  // namespace esp
