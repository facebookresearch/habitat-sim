
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_TEXTURE_UNIT_H_
#define ESP_GFX_PBR_TEXTURE_UNIT_H_

#include <Corrade/Containers/EnumSet.h>

namespace esp {
typedef unsigned char uint8_t;
namespace gfx {
namespace pbrTextureUnitSpace {

// The order of the samplers in the frag shader that specific textures should be
// mapped to
enum TextureUnit : uint8_t {
  BaseColor = 0,
  MetallicRoughness = 1,
  Normal = 2,
  Emissive = 3,
  ClearCoatFactor = 4,
  ClearCoatRoughenss = 5,
  ClearCoatNormal = 6,
  EnvironmentMap = 7,
  IrradianceMap = 8,
  BrdfLUT = 9,
  PrefilteredMap = 10,
  ShadowMap0 = 11,
  ShadowMap1 = 12,
  ShadowMap2 = 13,  // We support at most 3 shadows currently
};
}  // namespace pbrTextureUnitSpace
}  // namespace gfx
}  // namespace esp

#endif
