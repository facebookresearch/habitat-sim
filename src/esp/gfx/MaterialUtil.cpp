// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MaterialUtil.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/Trade/FlatMaterialData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/Trade.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

using Mn::Trade::MaterialAttribute;

namespace {

/**
 * @brief Make sure we do not add an attribute that already exists in source
 * material - doing this will cause an assertion.
 * @tparam the type of the value to be added
 * @param material Source material for new attributes array
 * @param newAttributes The array of attributes to be used to build a new
 * Mn::Trade::MaterialData.
 * @param matAttr The Mn::Trade::MaterialAttribute tag describing the attribute
 * to add
 * @param value A reference to the value to add.
 */
template <typename T>
void appendIfNotPresent(
    const Mn::Trade::MaterialData& material,
    Cr::Containers::Array<Mn::Trade::MaterialAttributeData>& newAttributes,
    const MaterialAttribute& matAttr,
    const T& value) {
  if (!material.hasAttribute(matAttr)) {
    arrayAppend(newAttributes, {matAttr, value});
  }
}

}  // namespace

Mn::Trade::MaterialData createUniversalMaterial(
    const Mn::Trade::MaterialData& origMaterialData) {
  // create a material, based on the passed material, that will have reasonable
  // attributes to support any possible shader type. should only be called if
  // we are not using the Material's natively specified shaderType

  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  // get source attributes from original material
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> newAttributes;
  arrayAppend(newAttributes, origMaterialData.attributeData());

  const auto origMatTypes = origMaterialData.types();
  // add appropriate attributes based on what is missing
  // flat material already recognizes Phong and pbr, so don't have to do
  // anything

  // multiplicative magic number for scaling from PBR to phong,
  // hacky method to attempt to balance Phong and PBR light intensity reactions
  const float magicPhongScaling = 1.5f;

  // whether the MaterialAttribute::TextureMatrix has been set already
  bool setTexMatrix = false;
  if (!(origMatTypes & Mn::Trade::MaterialType::Phong)) {
    // add appropriate values for expected attributes to support Phong from
    // PbrMetallicRoughnessMaterialData

    const auto& pbrMaterial =
        origMaterialData.as<Mn::Trade::PbrMetallicRoughnessMaterialData>();

    /////////////////
    // calculate Phong values from PBR material values
    ////////////////

    // derive ambient color from pbr baseColor
    const Mn::Color4 ambientColor = pbrMaterial.baseColor();

    // If there's a roughness texture, we have no way to use it here. The safest
    // fallback is to assume roughness == 1, thus producing no spec highlights.
    // If pbrMaterial does not have a roughness value, it returns a 1 by
    // default.
    const float roughness =
        pbrMaterial.hasRoughnessTexture() ? 1.0f : pbrMaterial.roughness();

    // If there's a metalness texture, we have no way to use it here.
    // The safest fallback is to assume non-metal.
    const float metalness =
        pbrMaterial.hasMetalnessTexture() ? 0.0f : pbrMaterial.metalness();

    // Heuristic to map roughness to spec power.
    // Higher exponent makes the spec highlight larger (lower power)
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.1+%2B+%281+-+x%29%5E4.5+*+180.0+for+x+from+0+to+1
    // lower power for metal
    const float maxShininess = Mn::Math::lerp(250.0f, 120.0f, metalness);
    const float shininess =
        1.1f + Mn::Math::pow(1.0f - roughness, 4.5f) * maxShininess;

    // increase spec intensity for metal
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.0+%2B+10*%28x%5E1.7%29++from+0+to+1
    const float specIntensityScale =
        (metalness > 0.0f
             ? (metalness < 1.0f
                    ? (1.0f + 10.0f * Mn::Math::pow(metalness, 1.7f))
                    : 11.0f)
             : 1.0f);
    // Heuristic to map roughness to spec intensity.
    // higher exponent decreases intensity.
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.4+*+%281-x%29%5E2.5++from+0+to+1
    const float specIntensity =
        Mn::Math::pow(1.0f - roughness, 2.5f) * 1.4f * specIntensityScale;

    // NOTE: The magic-number multiplication at the end is a hack to
    // roughly balance the Phong and PBR light intensity reactions
    const Mn::Color4 diffuseColor = ambientColor * magicPhongScaling;

    // Set spec base color to white or material base color, depending on
    // metalness.
    const Mn::Color4 specBaseColor =
        (metalness > 0.0f
             ? (metalness < 1.0f
                    ? Mn::Math::lerp(0xffffffff_rgbaf, ambientColor,
                                     Mn::Math::pow(metalness, 0.5f))
                    : ambientColor)
             : 0xffffffff_rgbaf);
    // NOTE: The magic-number multiplication at the end is a hack to
    // roughly balance the Phong and PBR light intensity reactions
    const Mn::Color4 specColor =
        specBaseColor * specIntensity * magicPhongScaling;

    /////////////////
    // set Phong attributes appropriately from precalculated value
    ////////////////
    // normal mapping is already present in copied array if present in
    // original material.

    // make sure not to re-add values that already exist in array, or
    // new color creation will assert.
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::Shininess, shininess);
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::AmbientColor, ambientColor);
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::DiffuseColor, diffuseColor);
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::SpecularColor, specColor);

    // texture transforms, if there's none the returned matrix is an
    // identity only copy if we don't already have TextureMatrix
    // attribute (if original pbrMaterial does not have that specific
    // matrix)
    if (!setTexMatrix &&
        (!pbrMaterial.hasAttribute(MaterialAttribute::TextureMatrix))) {
      arrayAppend(newAttributes, {MaterialAttribute::TextureMatrix,
                                  pbrMaterial.commonTextureMatrix()});
      setTexMatrix = true;
    }

    if (pbrMaterial.hasAttribute(MaterialAttribute::BaseColorTexture)) {
      // only provide texture indices if BaseColorTexture attribute
      // exists
      const Mn::UnsignedInt BCTexture = pbrMaterial.baseColorTexture();
      appendIfNotPresent(origMaterialData, newAttributes,
                         MaterialAttribute::AmbientTexture, BCTexture);
      appendIfNotPresent(origMaterialData, newAttributes,
                         MaterialAttribute::DiffuseTexture, BCTexture);
      if (metalness >= 0.5) {
        appendIfNotPresent(origMaterialData, newAttributes,
                           MaterialAttribute::SpecularTexture, BCTexture);
      }
    }
  }  // if no phong material support exists in material

  if (!(origMatTypes & Mn::Trade::MaterialType::PbrMetallicRoughness)) {
    // add appropriate values for expected attributes for PbrMetallicRoughness
    // derived from Phong attributes
    const auto& phongMaterial =
        origMaterialData.as<Mn::Trade::PhongMaterialData>();

    /////////////////
    // calculate PBR values from Phong material values
    ////////////////

    // derive base color from Phong diffuse or ambient color, depending on which
    // is present.  set to white if neither is present
    const Mn::Color4 baseColor = phongMaterial.diffuseColor();

    // Experimental metalness heuristic using saturation of spec color
    // to derive approximation of metalness
    const Mn::Color4 specColor = phongMaterial.specularColor();

    // if specColor alpha == 0 then no metalness
    float metalness = 0.0f;
    // otherwise, this hacky heuristic will derive a value for metalness based
    // on how non-grayscale the specular color is (HSV Saturation).
    if (specColor.a() != 0.0f) {
      metalness = specColor.saturation();
    }

    /////////////////
    // set PbrMetallicRoughness attributes appropriately from precalculated
    // values
    ////////////////

    // normal mapping is already present in copied array if present in
    // original material.
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::BaseColor, baseColor);
    appendIfNotPresent(origMaterialData, newAttributes,
                       MaterialAttribute::Metalness, metalness);

    // if diffuse texture is present, use as base color texture in pbr.
    if (phongMaterial.hasAttribute(MaterialAttribute::DiffuseTexture)) {
      uint32_t bcTextureVal = phongMaterial.diffuseTexture();
      appendIfNotPresent(origMaterialData, newAttributes,
                         MaterialAttribute::BaseColorTexture, bcTextureVal);
    }
    // texture transforms, if there's none the returned matrix is an
    // identity Only copy if we don't already have TextureMatrix attribute
    // (if original phongMaterial does not have that specific attribute)
    if (!setTexMatrix &&
        (!phongMaterial.hasAttribute(MaterialAttribute::TextureMatrix))) {
      arrayAppend(newAttributes, {MaterialAttribute::TextureMatrix,
                                  phongMaterial.commonTextureMatrix()});
      // setTexMatrix = true;
    }

    // base texture

  }  // if no PbrMetallicRoughness material support exists in material

  // build flags to support all materials
  constexpr auto flags = Mn::Trade::MaterialType::Flat |
                         Mn::Trade::MaterialType::Phong |
                         Mn::Trade::MaterialType::PbrMetallicRoughness;

  // create new material from attributes array
  Mn::Trade::MaterialData newMaterialData{flags, std::move(newAttributes)};

  return newMaterialData;
}  // ResourceManager::createUniversalMaterial

}  // namespace gfx
}  // namespace esp
