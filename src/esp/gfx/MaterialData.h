// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/GL/Texture.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

struct MaterialData {};

// This struct contains a subset of phong material fields (with the rest in
// PhongMaterialData). We expose this struct to python so that scripts can
// modify these fields at runtime.
struct PhongMaterialInfo {
  Magnum::Float shininess = 80.f;
  Magnum::Color4 ambientColor{0.1};
  Magnum::Color4 diffuseColor{0.7};
  Magnum::Color4 specularColor{0.2};
  std::string importName;
};

struct PhongMaterialData : public MaterialData {
  PhongMaterialInfo info;
  Magnum::Matrix3 textureMatrix;
  Magnum::GL::Texture2D *ambientTexture = nullptr, *diffuseTexture = nullptr,
                        *specularTexture = nullptr, *normalTexture = nullptr;
  bool perVertexObjectId = false, vertexColored = false;

  ESP_SMART_POINTERS(PhongMaterialData)
};

bool operator==(const PhongMaterialInfo& a, const PhongMaterialInfo& b);
bool operator!=(const PhongMaterialInfo& a, const PhongMaterialInfo& b);

}  // namespace gfx
}  // namespace esp
