// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigManager.h"

#include <esp/core/Check.h>

namespace esp {
namespace assets {

int RigManager::registerRigInstance(gfx::Rig&& rig) {
  int rigId = _nextRigInstanceID++;
  _rigInstances[rigId] = std::move(rig);
  return rigId;
}

void RigManager::registerRigInstance(int rigId, gfx::Rig&& rig) {
  ESP_CHECK(!rigInstanceExists(rigId),
            "A rig instance was already registered with the specified id.");
  _rigInstances[rigId] = std::move(rig);
  _nextRigInstanceID = std::max(_nextRigInstanceID, rigId + 1);
}

void RigManager::deleteRigInstance(int rigId) {
  const auto& rigIt = _rigInstances.find(rigId);
  ESP_CHECK(rigIt != _rigInstances.end(),
            "The specified rig instance id isn't known by rig manager or "
            "was already deleted.");
  for (auto* bone : getRigInstance(rigId).bones) {
    delete bone;
  }
  _rigInstances.erase(rigIt);
}

bool RigManager::rigInstanceExists(int rigId) const {
  return _rigInstances.find(rigId) != _rigInstances.end();
}

gfx::Rig& RigManager::getRigInstance(int rigId) {
  const auto& rigIt = _rigInstances.find(rigId);
  ESP_CHECK(rigIt != _rigInstances.end(),
            "The specified rig instance id isn't known by rig manager.");
  return rigIt->second;
}

}  // namespace assets
}  // namespace esp
