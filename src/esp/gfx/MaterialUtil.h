// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/assets/Asset.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/MaterialData.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

// todo: pass resourceKey by value?
gfx::PhongMaterialInfo getPhongMaterialInfo(ShaderManager& shaderManager,
                                            Magnum::ResourceKey key);

void updatePhongMaterialInfo(ShaderManager& shaderManager,
                             Magnum::ResourceKey key,
                             const gfx::PhongMaterialInfo& materialInfo);

void overrideMaterialForSubTree(scene::SceneNode& root,
                                const Magnum::ResourceKey& originalMaterial,
                                const Magnum::ResourceKey& overrideMaterial);
}  // namespace gfx
}  // namespace esp
