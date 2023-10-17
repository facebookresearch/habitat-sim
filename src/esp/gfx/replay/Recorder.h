// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_REPLAY_RECORDER_H_
#define ESP_GFX_REPLAY_RECORDER_H_

#include "Keyframe.h"

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace assets {
struct AssetInfo;
struct RenderAssetInstanceCreationInfo;
}  // namespace assets
namespace scene {
class SceneNode;
class SceneGraph;
}  // namespace scene
namespace gfx {
struct Rig;
namespace replay {

class NodeDeletionHelper;

/**
 * @brief Recording for "render replay".
 *
 * This class saves and serializes render keyframes. A render keyframe is a
 * visual snapshot of a scene. It includes the visual parts of a scene, such
 * that observations can be reproduced (from the same camera perspective or a
 * different one). The render keyframe includes support for named "user
 * transforms" which can be used to store cameras, agents, or other
 * application-specific objects. See also @ref Player. See
 * examples/replay_tutorial.py for usage of this class through bindings.
 */
class Recorder {
 public:
  ~Recorder();

  /**
   * @brief User code should call this upon creating a render asset instance so
   * that Recorder can track the instance.
   * @param node The root node of the instance
   * @param creation How the instance was created. Recorder will save this so
   * that the instance can be re-created later.
   */
  void onCreateRenderAssetInstance(
      scene::SceneNode* node,
      const esp::assets::RenderAssetInstanceCreationInfo& creation);

  /**
   * @brief User code should call this upon loading a render asset to inform
   * Recorder about the asset.
   * @param assetInfo The asset that was loaded.
   */
  void onLoadRenderAsset(const esp::assets::AssetInfo& assetInfo);

  /**
   * @brief User code should call this upon instantiating a skinned asset rig to
   * inform Recorder about it.
   * @param rigId Id of the rig that was instantiated.
   * @param rig Rig that was instantiated.
   */
  void onCreateRigInstance(int rigId, const Rig& rig);

  /**
   * @brief Record deletion of all render instances in a scene graph.
   * Because scene graphs are currently leaked when the active scene changes, we
   * cannot rely on node deletion to issue gfx-replay deletion entries. This
   * function allows to circumvent this issue.
   * The scene graph leak occurs in createSceneInstance(), in Simulator.cpp.
   * @param sceneGraph The scene graph being hidden.
   */
  void onHideSceneGraph(const esp::scene::SceneGraph& sceneGraph);

  /**
   * @brief Save/capture a render keyframe (a visual snapshot of the scene).
   *
   * User code can call this any time, but the intended usage is to save a
   * keyframe after stepping the environment and/or when drawing observations.
   * See also writeSavedKeyframesToFile.
   */
  void saveKeyframe();

  Keyframe extractKeyframe();

  /**
   * @brief Returns the last saved keyframe.
   */
  const Keyframe& getLatestKeyframe();

  /**
   * @brief Add a named "user transform" which can be used to store cameras,
   * agents, or other application-specific objects
   *
   * The user transforms can be retrieved by name during replay playback. See
   * @ref Player (coming soon).
   *
   * The user transform gets added to the "current" keyframe and will get saved
   * on the next call to saveKeyframe (but not later keyframes). For
   * "persistent" user objects, the expected usage is to call this every frame.
   *
   * @param name A name, to be used to retrieve the transform later.
   * @param translation
   * @param rotation
   */
  void addUserTransformToKeyframe(const std::string& name,
                                  const Magnum::Vector3& translation,
                                  const Magnum::Quaternion& rotation);

  /**
   * @brief Add a light to the current keyframe.
   *
   * @param lightInfo Parameters of the light to be added to the keyframe.
   */
  void addLightToKeyframe(const LightInfo& lightInfo);

  /**
   * @brief Delete all lights from the current keyframe.
   */
  void clearLightsFromKeyframe();

  /**
   * @brief write saved keyframes to file.
   * @param filepath
   *
   * If you prefer more readable json, set usePrettyWriter to true, but beware
   * larger filesize.
   */
  void writeSavedKeyframesToFile(const std::string& filepath,
                                 bool usePrettyWriter = false);

  /**
   * @brief write saved keyframes to string. '{"keyframes": [{...},{...},...]}'
   */
  std::string writeSavedKeyframesToString();

  /**
   * @brief write saved keyframes as individual strings ['{"keyframe": ...}',
   * '{"keyframe": ...}', ...]
   *
   * Use this function if you are using keyframes incrementally, e.g.
   * repeated calls to this function and feeding them to a renderer. Contrast
   * with writeSavedKeyframesToFile, which "consolidates" before discarding old
   * keyframes to avoid losing state information.
   */
  std::vector<std::string> writeIncrementalSavedKeyframesToStringArray();

  /**
   * @brief returns JSONized version of given keyframe.
   */
  static std::string keyframeToString(const Keyframe& keyframe);

  /**
   * @brief Reserved for unit-testing.
   */
  const std::vector<Keyframe>& debugGetSavedKeyframes() const {
    return savedKeyframes_;
  }

 private:
  // NodeDeletionHelper calls onDeleteRenderAssetInstance
  friend class NodeDeletionHelper;

  // Helper for tracking render asset instances
  struct InstanceRecord {
    scene::SceneNode* node = nullptr;
    RenderAssetInstanceKey instanceKey = ID_UNDEFINED;
    Corrade::Containers::Optional<RenderAssetInstanceState> recentState;
    NodeDeletionHelper* deletionHelper = nullptr;
    int rigId = ID_UNDEFINED;
  };

  using KeyframeIterator = std::vector<Keyframe>::const_iterator;

  rapidjson::Document writeKeyframesToJsonDocument();
  void onDeleteRenderAssetInstance(const scene::SceneNode* node);
  Keyframe& getKeyframe();
  void advanceKeyframe();
  RenderAssetInstanceKey getNewInstanceKey();
  int findInstance(const scene::SceneNode* queryNode);
  RenderAssetInstanceState getInstanceState(const scene::SceneNode* node);
  void updateStates();
  void updateInstanceStates();
  void updateRigInstanceStates();
  void checkAndAddDeletion(Keyframe* keyframe,
                           RenderAssetInstanceKey instanceKey);
  void addLoadsCreationsDeletions(KeyframeIterator begin,
                                  KeyframeIterator end,
                                  Keyframe* dest);
  void consolidateSavedKeyframes();

  std::vector<InstanceRecord> instanceRecords_;
  Keyframe currKeyframe_;
  std::vector<Keyframe> savedKeyframes_;
  RenderAssetInstanceKey nextInstanceKey_ = 0;
  std::unordered_map<int, std::vector<scene::SceneNode*>> rigNodes_;
  std::unordered_map<int, std::vector<Magnum::Matrix4>> rigNodeTransformCache_;

  ESP_SMART_POINTERS(Recorder)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif
