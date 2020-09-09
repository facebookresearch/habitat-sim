
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/gfx/MaterialData.h"

// forward declarations
namespace Magnum {
namespace Trade {
class AbstractImporter;
class AbstractShaderProgram;
class PhongMaterialData;
class PbrMetallicRoughnessMaterialData;
}  // namespace Trade
}  // namespace Magnum

namespace esp {
namespace gfx {

/**
 * @brief Build a @ref PhongMaterialData from a PBR source material, using
 * some heuristics. This function is temporary and should go away once we have
 * a PBR shader.
 */
gfx::PhongMaterialData::uptr buildPhongFromPbrMetallicRoughness(
    const Magnum::Trade::PbrMetallicRoughnessMaterialData& material,
    int textureBaseIndex,
    const std::vector<std::shared_ptr<Magnum::GL::Texture2D>>& textures);

}  // namespace gfx
}  // namespace esp
