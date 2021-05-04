// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderAssetInstanceCreationInfo.h"

#include <utility>

namespace esp {
namespace assets {

RenderAssetInstanceCreationInfo::RenderAssetInstanceCreationInfo(
    std::string _filepath,
    Corrade::Containers::Optional<Magnum::Vector3> _scale,
    const Flags& _flags,
    std::string _lightSetupKey)
    : filepath(std::move(_filepath)),
      scale(std::move(_scale)),
      flags(_flags),
      lightSetupKey(std::move(_lightSetupKey)) {}

}  // namespace assets
}  // namespace esp
