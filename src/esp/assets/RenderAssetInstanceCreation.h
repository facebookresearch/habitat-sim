// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Asset.h"

#include "esp/scene/SceneNode.h"

#include "Magnum/Resource.h"

#include <Corrade/Containers/Optional.h>

namespace esp {
namespace assets {

// parameters to control how a render asset instance is created
struct RenderAssetInstanceCreation {
  std::string filepath;  // see also AssetInfo::filepath
  Corrade::Containers::Optional<Magnum::Vector3> scale;
  bool isStatic =
      false;            // inform the renderer that this instance won't be moved
  bool isRGBD = false;  // use in RGB and depth observations
  bool isSemantic = false;  // use in semantic observations
  std::string lightSetupKey;
};

}  // namespace assets
}  // namespace esp
