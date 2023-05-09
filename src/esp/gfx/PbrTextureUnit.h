
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
  SpecularLayer = 7,
  SpecularLayerColor = 8,
  EnvironmentMap = 9,
  IrradianceMap = 10,
  BrdfLUT = 11,
  PrefilteredMap = 12,
  ShadowMap0 = 13,
  ShadowMap1 = 14,
  ShadowMap2 = 15,  // We support at most 3 shadows currently
};
}  // namespace pbrTextureUnitSpace
}  // namespace gfx
}  // namespace esp

#endif
