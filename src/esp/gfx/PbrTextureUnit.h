
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_TEXTURE_UNIT_H_
#define ESP_GFX_PBR_TEXTURE_UNIT_H_

#include <Corrade/Containers/EnumSet.h>

namespace esp {
namespace gfx {

namespace pbrTextureUnitSpace {
enum TextureUnit : uint8_t {
  BaseColor = 0,
  MetallicRoughness = 1,
  Normal = 2,
  Emissive = 3,
  EnvironmentMap = 4,
  IrradianceMap = 5,
  BrdfLUT = 6,
  PrefilteredMap = 7,
  ShadowMap0 = 8,
};
}  // namespace pbrTextureUnitSpace
}  // namespace gfx
}  // namespace esp

#endif
