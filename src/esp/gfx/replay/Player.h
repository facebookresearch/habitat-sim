// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_REPLAY_PLAYER_H_
#define ESP_GFX_REPLAY_PLAYER_H_

#include "Keyframe.h"

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"

#include <rapidjson/document.h>

namespace esp {
namespace gfx {
namespace replay {

class Player;

/**
@brief Node handle

Represents either a @ref scene::SceneNode "scene::SceneNode*" or an arbitrary
handle value. A @cpp nullptr @ce value is treated as an empty / invalid handle.
*/
typedef class NodeHandle_* NodeHandle;

/**
@brief Backend implementation for @ref Player

Intended to be used via subclassing and implementing at least all pure virtual
functions, optionally also @ref setNodeSematicId() and @ref changeLightSetup()
which are no-op by default.
*/
class AbstractPlayerImplementation {
 public:
  explicit AbstractPlayerImplementation() = default;
  virtual ~AbstractPlayerImplementation() = default;

  /* Deliberately non-copyable to avoid accidents */
  AbstractPlayerImplementation(const AbstractPlayerImplementation&) = delete;
  AbstractPlayerImplementation(AbstractPlayerImplementation&&) noexcept =
      default;
  AbstractPlayerImplementation& operator=(const AbstractPlayerImplementation&) =
      delete;
  AbstractPlayerImplementation& operator=(
      AbstractPlayerImplementation&&) noexcept = default;

 private:
  /* The interfaces are private, i.e. not meant to be called from subclasses */
  friend Player;

  /**
   * @brief Load and create a render asset instance
   *
   * Returns a handle that references the newly added node containing the
   * loaded asset --- i.e., it's meant to appear in the scene. Returns
   * @cpp nullptr @ce in case of a failure.
   */
  virtual NodeHandle loadAndCreateRenderAssetInstance(
      const esp::assets::AssetInfo& assetInfo,
      const esp::assets::RenderAssetInstanceCreationInfo& creation) = 0;

  /**
   * @brief Delete asset instance
   *
   * The @p handle is expected to be returned from an earlier call to
   * @ref loadAndCreateRenderAssetInstance() on the same instance. Use
   * @ref clearAssetInstances() for deleting everything instead.
   */
  virtual void deleteAssetInstance(NodeHandle node) = 0;

  /**
   * @brief Clear all asset instances
   *
   * The player behaves as if @ref loadAndCreateRenderAssetInstance() was not
   * called at all. It *may* still contain the resources like meshes and
   * textures loaded by the previous calls though. The @p instances contain
   * all node handles returned from @ref loadAndCreateRenderAssetInstance() and
   * can be ignored if the implementation keeps track of these on its own.
   */
  virtual void deleteAssetInstances(
      const std::unordered_map<RenderAssetInstanceKey, NodeHandle>&
          instances) = 0;

  /**
   * @brief Set node transform from translation and rotation components.
   *
   * The @p handle is expected to be returned from an earlier call to
   * @ref loadAndCreateRenderAssetInstance() on the same instance.
   */
  virtual void setNodeTransform(NodeHandle node,
                                const Magnum::Vector3& translation,
                                const Magnum::Quaternion& rotation) = 0;

  /**
   * @brief Set node transform.
   *
   * The @p handle is expected to be returned from an earlier call to
   * @ref loadAndCreateRenderAssetInstance() on the same instance.
   */
  virtual void setNodeTransform(NodeHandle node,
                                const Mn::Matrix4& transform) = 0;

  /**
   * @brief Get node transform.
   *
   * The @p handle is expected to be returned from an earlier call to
   * @ref loadAndCreateRenderAssetInstance() on the same instance.
   */
  virtual Mn::Matrix4 hackGetNodeTransform(NodeHandle node) const = 0;

  /**
   * @brief Set node semantic ID
   *
   * The @p handle is expected to be returned from an earlier call to
   * @ref loadAndCreateRenderAssetInstance() on the same instance. Default
   * implementation does nothing.
   */
  virtual void setNodeSemanticId(NodeHandle node, unsigned id);

  /**
   * @brief Change light setup
   *
   * Default implementation does nothing.
   */
  virtual void changeLightSetup(const LightSetup& lights);
};

/**
@brief Classic scene graph backend implementation for @ref Player

Intended to be used via subclassing and implementing
@ref loadAndCreateRenderAssetInstance() and @ref changeLightSetup().
*/
class AbstractSceneGraphPlayerImplementation
    : public AbstractPlayerImplementation {
  /* The interfaces are private, i.e. not meant to be called from subclasses */

  void deleteAssetInstance(NodeHandle node) override;

  void deleteAssetInstances(
      const std::unordered_map<RenderAssetInstanceKey, NodeHandle>& instances)
      override;

  void setNodeTransform(NodeHandle node,
                        const Magnum::Vector3& translation,
                        const Magnum::Quaternion& rotation) override;

  void setNodeTransform(NodeHandle node, const Mn::Matrix4& transform) override;

  Mn::Matrix4 hackGetNodeTransform(NodeHandle node) const override;

  void setNodeSemanticId(NodeHandle node, unsigned id) override;
};

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
  /**
   * @brief Construct a Player.
   *
   * The @p implementation is assumed to be owned by the caller for the whole
   * lifetime of the @ref Player instance.
   */
  explicit Player(std::shared_ptr<AbstractPlayerImplementation> implementation);

  /* Deliberately move-only, it's heavy */
  Player(const Player&&) = delete;
  Player(Player&&) noexcept = default;
  Player& operator=(const Player&) = delete;
  Player& operator=(Player&&) noexcept = default;

  ~Player();

  /**
   * @brief Read keyframes. See also @ref Recorder::writeSavedKeyframesToFile.
   * After calling this, use @ref setKeyframeIndex to set a keyframe.
   * @param filepath
   */
  void readKeyframesFromFile(const std::string& filepath);

  /**
   * @brief Given a JSON string encoding a wrapped keyframe, returns the
   * keyframe itself.
   *
   * The JSON string is expected to be an object with a single `keyframe` key
   * containing data for the one keyframe. Use @ref keyframeFromStringUnwrapped()
   * to consume directly the data.
   */
  static Keyframe keyframeFromString(const std::string& keyframe);

  /**
   * @brief Given a JSON string encoding a keyframe, returns the keyframe
   * itself.
   *
   * The JSON string is expected to directly contain the keyframe object,
   * (with `loads`, `creations`, etc.). Use @ref keyframeFromString() to
   * consume a keyframe wrapped in an additional object.
   */
  static Keyframe keyframeFromStringUnwrapped(
      Corrade::Containers::StringView keyframe);

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

  /**
   * @brief Reserved for unit-testing.
   */
  const std::vector<Keyframe>& debugGetKeyframes() const { return keyframes_; }

  /**
   * @brief Appends a Keyframe to the keyframe list.
   */
  void appendKeyframe(Keyframe&& keyframe);

  void setSingleKeyframe(Keyframe&& keyframe);

  /**
   * @brief Appends a JSON keyframe to the keyframe list.
   */
  void appendJSONKeyframe(const std::string& keyframe);

 private:
  void applyKeyframe(const Keyframe& keyframe);
  void readKeyframesFromJsonDocument(const rapidjson::Document& d);
  void clearFrame();
  void hackProcessDeletions(const Keyframe& keyframe);

  std::shared_ptr<AbstractPlayerImplementation> implementation_;

  int frameIndex_ = -1;
  std::vector<Keyframe> keyframes_;
  std::unordered_map<std::string, esp::assets::AssetInfo> assetInfos_;
  std::unordered_map<RenderAssetInstanceKey, NodeHandle> createdInstances_;
  std::unordered_map<RenderAssetInstanceKey,
                     assets::RenderAssetInstanceCreationInfo>
      creationInfos_;
  std::unordered_map<RenderAssetInstanceKey, Mn::Matrix4>
      latestTransformCache_{};
  std::set<std::string> failedFilepaths_;

  ESP_SMART_POINTERS(Player)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif
