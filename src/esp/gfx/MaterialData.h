// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_MATERIALDATA_H_
#define ESP_GFX_MATERIALDATA_H_

#include <Magnum/GL/Texture.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

struct MaterialData {
  bool perVertexObjectId = false;

  // construct it using the default constructor. NO initial values, such as
  // identity matrix
  Magnum::Matrix3 textureMatrix;
};

struct PhongMaterialData : public MaterialData {
  Magnum::Float shininess = 80.f;
  Magnum::Color4 ambientColor{0.1};
  Magnum::Color4 diffuseColor{0.7};
  Magnum::Color4 specularColor{0.2};
  Magnum::GL::Texture2D *ambientTexture = nullptr, *diffuseTexture = nullptr,
                        *specularTexture = nullptr, *normalTexture = nullptr;
  bool vertexColored = false;

  ESP_SMART_POINTERS(PhongMaterialData)
};

struct PBRMaterialData : public MaterialData {
  // use the value if albedoTexture does not exist
  Magnum::Color4 baseColor{0.7};
  // use the value if roughnessTexture does not exist
  Magnum::Float roughness = 0.9f;
  // use the value if metallicTexture does not exist
  Magnum::Float metallic = 0.1f;
  Magnum::GL::Texture2D* albedoTexture = nullptr;
  Magnum::GL::Texture2D* normalTexture = nullptr;
  Magnum::GL::Texture2D* metallicTexture = nullptr;
  Magnum::GL::Texture2D* roughnessTexture = nullptr;
  // TODO:
  // AO texture, emisive texture
  ESP_SMART_POINTERS(PBRMaterialData)
};

// TODO: Ptex material data

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_MATERIALDATA_H_
