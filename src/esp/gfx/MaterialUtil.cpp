// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialUtil.h"

#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

gfx::PhongMaterialData::uptr buildPhongFromPbrMetallicRoughness(
    const Mn::Trade::PbrMetallicRoughnessMaterialData& material,
    int textureBaseIndex,
    const std::map<int, std::shared_ptr<Magnum::GL::Texture2D>>& textures) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  auto finalMaterial = gfx::PhongMaterialData::create_unique();

  // If there's a roughness texture, we have no way to use it here. The safest
  // fallback is to assume roughness == 1, thus producing no spec highlights.
  const float roughness =
      material.hasRoughnessTexture() ? 1 : material.roughness();

  // If there's a metalness texture, we have no way to use it here. The safest
  // fallback is to assume non-metal.
  const float metalness =
      material.hasMetalnessTexture() ? 0 : material.metalness();

  // Heuristic to map roughness to spec power.
  // Higher exponent makes the spec highlight larger (lower power)
  // https://www.wolframalpha.com/input/?i=5+%2B+%281+-+x%29%5E1.5+*+75+for+x+from+0+to+1
  // lower power for metal
  const float maxShininess = Magnum::Math::lerp(250, 120, metalness);
  finalMaterial->shininess = 1.1 + powf(1 - roughness, 4.5) * maxShininess;

  // Heuristic to map roughness to spec intensity.
  // higher exponent decreases intensity.
  // https://www.wolframalpha.com/input/?i=%281-x%29%5E3+from+0+to+1
  float specIntensity = powf(1 - roughness, 2.5) * 1.4;
  // increase spec intensity for metal
  specIntensity *= (1 + 10 * powf(metalness, 1.7));

  // texture transform, if there's none the matrix is an identity
  finalMaterial->textureMatrix = material.commonTextureMatrix();

  // another heuristic: reduce diffuse intensity for metal
  float diffuseScale = Magnum::Math::lerp(1.0, 0.2, material.metalness());
  finalMaterial->diffuseColor = material.baseColor() * diffuseScale;
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::BaseColorTexture)) {
    finalMaterial->diffuseTexture =
        textures.at(textureBaseIndex + material.baseColorTexture()).get();
  }

  // Set spec base color to white or material base color, depending on
  // metalness.
  Magnum::Color4 specBaseColor =
      lerp(0xffffffff_rgbaf, material.baseColor(), powf(metalness, 0.5));
  finalMaterial->specularColor = specBaseColor * specIntensity;
  if (metalness >= 0.5) {
    finalMaterial->specularTexture = finalMaterial->diffuseTexture;
  }

  // set ambient color to match base color
  finalMaterial->ambientColor = material.baseColor();
  finalMaterial->ambientTexture = finalMaterial->diffuseTexture;

  // normal mapping
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::NormalTexture)) {
    finalMaterial->normalTexture =
        textures.at(textureBaseIndex + material.normalTexture()).get();
  }

  return finalMaterial;
}

}  // namespace gfx
}  // namespace esp
