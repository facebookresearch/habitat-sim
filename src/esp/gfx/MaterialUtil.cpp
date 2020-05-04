// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialUtil.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

// todo: investigate coding standard for private free functions
namespace {

gfx::PythonMaterial toPythonMaterial(const PhongMaterialData& phongMaterial) {
  return gfx::PythonMaterial{.shininess = phongMaterial.shininess,
                             .ambientColor = phongMaterial.ambientColor,
                             .diffuseColor = phongMaterial.diffuseColor,
                             .specularColor = phongMaterial.specularColor,
                             .importName = phongMaterial.importName};
}

// todo: investigate ref vs pointer for in/out params
void updatePhongMaterialFromPythonMaterial(
    PhongMaterialData& phongMaterial,
    const gfx::PythonMaterial& pythonMaterial) {
  phongMaterial.shininess = pythonMaterial.shininess;
  phongMaterial.ambientColor = pythonMaterial.ambientColor;
  phongMaterial.diffuseColor = pythonMaterial.diffuseColor;
  phongMaterial.specularColor = pythonMaterial.specularColor;
  phongMaterial.importName = pythonMaterial.importName;
}

}  // namespace

// todo: rename PythonMaterial? to convey that it's a partial material
// todo: consistency "material" vs "key"
gfx::PythonMaterial getPythonMaterial(ShaderManager& shaderManager,
                                      Mn::ResourceKey key) {
  const auto& phongMaterial =
      *shaderManager.get<MaterialData, PhongMaterialData>(key);
  return toPythonMaterial(phongMaterial);
}

void updateMaterialFromPythonMaterial(
    ShaderManager& shaderManager,
    Mn::ResourceKey key,
    const gfx::PythonMaterial& pythonMaterial) {
  auto& phongMaterial =
      *shaderManager.get<MaterialData, PhongMaterialData>(key);
  updatePhongMaterialFromPythonMaterial(phongMaterial, pythonMaterial);
}

void overrideMaterialForSubTree(scene::SceneNode& root,
                                const Magnum::ResourceKey& originalMaterial,
                                const Magnum::ResourceKey& overrideMaterial) {
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
