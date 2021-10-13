
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_MATERIALUTIL_H_
#define ESP_GFX_MATERIALUTIL_H_

#include <Magnum/Trade/Trade.h>
#include "esp/gfx/MaterialData.h"

namespace Mn = Magnum;
// forward declarations
namespace Magnum {
namespace Trade {
class AbstractImporter;
class PhongMaterialData;
class PbrMetallicRoughnessMaterialData;
}  // namespace Trade
}  // namespace Magnum

namespace esp {
namespace gfx {

/**
 * @brief This function will take an existing Mn::Trade::MaterialData and add
 * the missing attributes for the types it does not support, so that it will
 * have attributes for all habitat-supported types. This should only be called
 * if the user has specified a desired shader type that the material does not
 * natively support.
 * @param origMaterialData The original material from the importer
 * @return The new material with attribute support for all supported shader
 * types.
 */
Mn::Trade::MaterialData createUniversalMaterial(
    const Mn::Trade::MaterialData& origMaterialData);

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_MATERIALUTIL_H_
