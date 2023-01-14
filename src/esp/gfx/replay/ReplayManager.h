// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_REPLAY_REPLAYMANAGER_H_
#define ESP_GFX_REPLAY_REPLAYMANAGER_H_

#include "Player.h"
#include "Recorder.h"

namespace esp {
namespace gfx {
namespace replay {

/**
 * @brief Helper class for the replay python bindings (GfxReplayBindings.cpp)
 *
 * This class is intended to be used as a singleton. It optionally keeps a
 * pointer to a Recorder instance, and it helps construct Player instances.
 */
class ReplayManager {
 public:
  /**
   * @brief Optionally make a Recorder instance available to python, or pass
   * nullptr.
   */
  void setRecorder(std::shared_ptr<Recorder> writer) {
    recorder_ = std::move(writer);
  }

  /**
   * @brief Get a Recorder instance, or nullptr if it isn't enabled.
   */
  std::shared_ptr<Recorder> getRecorder() const { return recorder_; }

  /**
   * @brief Set callbacks that are invoked when simulation state changes upon
   * playing a new keyframe. This is required to construct Player instances.
   */
  void setPlayerImplementation(
      std::shared_ptr<AbstractPlayerImplementation>&& implementation) {
    playerImplementation_ = std::move(implementation);
  }

  /**
   * @brief Read keyframes from a file and construct a Player. Returns nullptr
   * if no keyframes could be read.
   */
  std::shared_ptr<Player> readKeyframesFromFile(const std::string& filepath);

  /**
   * @brief Returns an empty Player object. This can be used if you want to add
   * keyframes later on.
   */
  std::shared_ptr<Player> createEmptyPlayer();

 private:
  std::shared_ptr<Recorder> recorder_;
  std::shared_ptr<AbstractPlayerImplementation> playerImplementation_;

  ESP_SMART_POINTERS(ReplayManager)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif
