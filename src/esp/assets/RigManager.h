// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_RIGMANAGER_H_
#define ESP_ASSETS_RIGMANAGER_H_

#include "esp/gfx/SkinData.h"

namespace esp {
namespace assets {

// Tracks the rig instances in a simulator (skinned articulated objects).
class RigManager {
 public:
  /**
   * @brief Registers a rig instance. This gives ownership of the rig to the rig manager. Use @ref deleteRigInstance to dispose of the rig.
   *
   * @param rig Instantiated rig to register.
   * @return Unique id of the rig.
   */
  int registerRigInstance(gfx::Rig&& rig);

  /**
   * @brief Registers a rig instance. This gives ownership of the rig to the rig manager. Use @ref deleteRigInstance to dispose of the rig.
   * This variant assumes that the rig id comes from gfx-replay, so id
   * management can be skipped.
   *
   * @param rigId Unique id for the rig.
   * @param rig Instantiated rig to register.
   */
  void registerRigInstance(int rigId, gfx::Rig&& rig);

  /**
   * @brief Unregisters a rig instance and deletes its bone nodes.
   *
   * @param rigId ID of the rig.
   */
  void deleteRigInstance(int rigId);

  /**
   * @brief Checks if the specified rig ID has been registered to the rig
   * manager.
   *
   * @param rigId ID of the rig.
   * @return Whether the rig is registered to the rig manager.
   */
  bool rigInstanceExists(int rigId) const;

  /**
   * @brief Get a reference to a registered rig instance.
   *
   * @param rigId ID of the rig.
   * @return Reference to a registered rig instance.
   */
  gfx::Rig& getRigInstance(int rigId);

 private:
  /**
   * @brief The rigs for instantiated skinned assets.
   */
  std::unordered_map<int, gfx::Rig> _rigInstances;

  /**
   * @brief The next available unique ID for instantiated rigs.
   */
  int _nextRigInstanceID = 0;
};

}  // namespace assets
}  // namespace esp

#endif
