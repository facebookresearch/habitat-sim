// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_REPLAY_KEYFRAME_H_
#define ESP_GFX_REPLAY_KEYFRAME_H_

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"

namespace esp {
namespace gfx {
namespace replay {

using RenderAssetInstanceKey = int32_t;

/**
 * @brief serialization/replay-friendly Transform class
 */
struct Transform {
  Magnum::Vector3 translation;
  Magnum::Quaternion rotation;
  bool operator==(const Transform& rhs) const {
    return translation == rhs.translation && rotation == rhs.rotation;
  }
};

/**
 * @brief The dynamic state of a render instance that is tracked by the replay
 * system every frame.
 */
struct RenderAssetInstanceState {
  Transform absTransform;  // localToWorld
  // note we currently only support semanticId per instance, not per drawable
  int semanticId = ID_UNDEFINED;
  // note we don't currently support runtime changes to lightSetupKey
  bool operator==(const RenderAssetInstanceState& rhs) const {
    return absTransform == rhs.absTransform && semanticId == rhs.semanticId;
  }
};

/**
 * @brief A serialization/replay-friendly render keyframe class. See @ref
 * Recorder.
 */
struct Keyframe {
  std::vector<esp::assets::AssetInfo> loads;
  std::vector<std::pair<RenderAssetInstanceKey,
                        esp::assets::RenderAssetInstanceCreationInfo>>
      creations;
  std::vector<RenderAssetInstanceKey> deletions;
  std::vector<std::pair<RenderAssetInstanceKey, RenderAssetInstanceState>>
      stateUpdates;
  std::unordered_map<std::string, Transform> userTransforms;
  std::vector<LightInfo> lights;
  bool lightsChanged = false;

  ESP_SMART_POINTERS(Keyframe)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif
