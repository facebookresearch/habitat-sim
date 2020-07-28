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

struct PhongMaterialData : public MaterialData {
  Magnum::Float shininess = 80.f;
  Magnum::Color4 ambientColor{0.1};
  Magnum::Color4 diffuseColor{0.7};
  Magnum::Color4 specularColor{0.2};
  Magnum::Matrix3 textureMatrix;
  Magnum::GL::Texture2D *ambientTexture = nullptr, *diffuseTexture = nullptr,
                        *specularTexture = nullptr, *normalTexture = nullptr;
  bool perVertexObjectId = false, vertexColored = false;

  ESP_SMART_POINTERS(PhongMaterialData)
};

// TODO: Ptex material data

}  // namespace gfx
}  // namespace esp
