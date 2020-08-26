// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialUtil.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

gfx::PhongMaterialInfo getPhongMaterialInfo(ShaderManager& shaderManager,
                                            const Mn::ResourceKey& key) {
  const auto& phongMaterial =
      *shaderManager.get<MaterialData, PhongMaterialData>(key);
  return phongMaterial.info;
}

void updatePhongMaterialInfo(ShaderManager& shaderManager,
                             const Mn::ResourceKey& key,
                             const gfx::PhongMaterialInfo& materialInfo) {
  auto& phongMaterial =
      *shaderManager.get<MaterialData, PhongMaterialData>(key);
  phongMaterial.info = materialInfo;
}

void overrideMaterialForSubTree(scene::SceneNode& root,
                                const Mn::ResourceKey& originalMaterial,
                                const Mn::ResourceKey& overrideMaterial) {
  scene::preOrderFeatureTraversalWithCallback<Drawable>(
      root, [&](Drawable& drawable) {
        auto orig = drawable.getOriginalMaterial();
        if (drawable.getOriginalMaterial() == originalMaterial) {
          drawable.setMaterial(overrideMaterial);
        }
      });
}

}  // namespace gfx
}  // namespace esp
