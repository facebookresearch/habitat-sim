// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_REPLAY_PLAYER_H_
#define ESP_GFX_REPLAY_PLAYER_H_

#include "Keyframe.h"

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"

#include <rapidjson/document.h>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace esp {
namespace scene {
class SceneNode;
}
namespace gfx {
namespace replay {

/**
 * @brief Playback for "render replay".
 *
 * This class is work in progress (readKeyframesFromFile isn't implemented
 * yet).
 *
 * This class reads render keyframes from a file so that observations can be
 * reproduced (from the same camera perspective or a different one). A loaded
 * keyframe can be set (applied to the scene) with setKeyframeIndex; render
 * asset instances are added to the scene as needed and new observations can be
 * rendered. Render assets are loaded as needed. See also @ref Recorder. See
 * examples/replay_tutorial.py for usage of this class through bindings (coming
 * soon).
 */
class Player {
 public:
  using LoadAndCreateRenderAssetInstanceCallback =
      std::function<esp::scene::SceneNode*(
          const esp::assets::AssetInfo&,
          const esp::assets::RenderAssetInstanceCreationInfo&)>;

  /**
   * @brief Construct a Player.
   * @param callback A function to load and create a render asset instance.
   */
  explicit Player(const LoadAndCreateRenderAssetInstanceCallback& callback);

  ~Player();

  /**
   * @brief Read keyframes. See also @ref Recorder::writeSavedKeyframesToFile.
   * After calling this, use @ref setKeyframeIndex to set a keyframe.
   * @param filepath
   */
  void readKeyframesFromFile(const std::string& filepath);

  /**
   * @brief Get the currently-set keyframe, or -1 if no keyframe is set.
   */
  int getKeyframeIndex() const;

  /**
   * @brief Get the number of keyframes read from file.
   */
  int getNumKeyframes() const;

  /**
   * @brief Set a keyframe by index, or pass -1 to clear the currently-set
   * keyframe.
   */
  void setKeyframeIndex(int frameIndex);

  /**
   * @brief Get a user transform. See @ref Recorder::addUserTransformToKeyframe
   * for usage tips.
   */
  bool getUserTransform(const std::string& name,
                        Magnum::Vector3* translation,
                        Magnum::Quaternion* rotation) const;

  /**
   * @brief Unload all keyframes.
   */
  void close();

  /**
   * @brief Reserved for unit-testing.
   */
  void debugSetKeyframes(std::vector<Keyframe>&& keyframes) {
    keyframes_ = std::move(keyframes);
  }

 private:
  void readKeyframesFromJsonDocument(const rapidjson::Document& d);
  void clearFrame();
  void applyKeyframe(const Keyframe& keyframe);
  static void setSemanticIdForSubtree(esp::scene::SceneNode* rootNode,
                                      int semanticId);

  LoadAndCreateRenderAssetInstanceCallback
      loadAndCreateRenderAssetInstanceCallback;
  int frameIndex_ = -1;
  std::vector<Keyframe> keyframes_;
  std::map<std::string, esp::assets::AssetInfo> assetInfos_;
  std::map<RenderAssetInstanceKey, scene::SceneNode*> createdInstances_;
  std::set<std::string> failedFilepaths_;

  ESP_SMART_POINTERS(Player)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif
