// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace gfx {
namespace replay {

using RenderAssetInstanceKey = uint32_t;

// serialization/replay-friendly Transform class
struct Transform {
  Magnum::Vector3 translation;
  Magnum::Quaternion rotation;
  bool operator==(const Transform& rhs) const {
    return translation == rhs.translation && rotation == rhs.rotation;
  }
};

struct RenderAssetInstanceState {
  Transform absTransform;  // localToWorld
  // note we currently only support semanticId per instance, not per drawable
  int semanticId = -1;
  // note we don't currently support runtime changes to lightSetupKey
  bool operator==(const RenderAssetInstanceState& rhs) const {
    return absTransform == rhs.absTransform && semanticId == rhs.semanticId;
  }
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp
