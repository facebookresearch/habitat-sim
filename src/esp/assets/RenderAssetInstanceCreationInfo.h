// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_RENDERASSETINSTANCECREATIONINFO_H_
#define ESP_ASSETS_RENDERASSETINSTANCECREATIONINFO_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Containers/Optional.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>

#include <memory>
#include <string>
#include "esp/core/Esp.h"

namespace esp {
namespace assets {

// parameters to control how a render asset instance is created
struct RenderAssetInstanceCreationInfo {
  enum class Flag : unsigned int {
    IsStatic = 1 << 0,  // inform the renderer that this instance won't be moved
    IsRGBD = 1 << 1,    // use in RGB and depth observations
    IsSemantic = 1 << 2,             // use in semantic observations
    IsTextureBasedSemantic = 1 << 3  // semantic asset using annotated textures
                                     // (as opposed to vertex annotations)
  };
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  RenderAssetInstanceCreationInfo() = default;

  RenderAssetInstanceCreationInfo(
      const std::string& _filepath,
      const Corrade::Containers::Optional<Magnum::Vector3>& _scale,
      const Flags& _flags,
      const std::string& _lightSetupKey,
      int _rigId = ID_UNDEFINED);

  bool isStatic() const { return bool(flags & Flag::IsStatic); }
  bool isRGBD() const { return bool(flags & Flag::IsRGBD); }
  bool isSemantic() const { return bool(flags & Flag::IsSemantic); }
  bool isTextureBasedSemantic() const {
    return bool(flags & Flag::IsTextureBasedSemantic);
  }

  std::string filepath;  // see also AssetInfo::filepath
  Corrade::Containers::Optional<Magnum::Vector3> scale;
  // NOTE: the following allows for a predefined offset transformation for a
  // visual shape from its parent frame.
  Corrade::Containers::Optional<Magnum::Vector3> translation;
  Corrade::Containers::Optional<Magnum::Quaternion> rotation;
  Flags flags;
  std::string lightSetupKey;
  int rigId = ID_UNDEFINED;
};

}  // namespace assets
}  // namespace esp

#endif
