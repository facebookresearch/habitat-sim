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

/**
 * @brief get PhongMaterialInfo for a material
 */
gfx::PhongMaterialInfo getPhongMaterialInfo(ShaderManager& shaderManager,
                                            const Magnum::ResourceKey& key);

/**
 * @brief update PhongMaterialInfo for a material
 */
void updatePhongMaterialInfo(ShaderManager& shaderManager,
                             const Magnum::ResourceKey& key,
                             const gfx::PhongMaterialInfo& materialInfo);

/**
 * @brief Find-and-replace all usages of the original material with the override
 * material, for all Drawables in the entire subtree at root.
 */
void overrideMaterialForSubTree(scene::SceneNode& root,
                                const Magnum::ResourceKey& originalMaterial,
                                const Magnum::ResourceKey& overrideMaterial);
}  // namespace gfx
}  // namespace esp
