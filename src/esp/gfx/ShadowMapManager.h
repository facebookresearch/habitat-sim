// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#ifndef ESP_GFX_SHADOW_MANAGER_H_
#define ESP_GFX_SHADOW_MANAGER_H_

#include <Magnum/ResourceManager.h>
#include "esp/gfx/CubeMap.h"

namespace esp {
namespace gfx {
using ShadowMapManager = Magnum::ResourceManager<CubeMap>;
using ShadowMapKeys = std::vector<Magnum::ResourceKey>;
}  // namespace gfx
}  // namespace esp
#endif
