// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderAssetInstanceCreationInfo.h"

namespace esp {
namespace assets {

RenderAssetInstanceCreationInfo::RenderAssetInstanceCreationInfo(
    const std::string& _filepath,
    const Corrade::Containers::Optional<Magnum::Vector3>& _scale,
    bool isStatic,
    bool isRGBD,
    bool isSemantic,
    const std::string& _lightSetupKey)
    : filepath(_filepath), scale(_scale), lightSetupKey(_lightSetupKey) {
  flags |= isStatic ? Flag::IsStatic : Flags();
  flags |= isRGBD ? Flag::IsRGBD : Flags();
  flags |= isSemantic ? Flag::IsSemantic : Flags();
}

}  // namespace assets
}  // namespace esp
