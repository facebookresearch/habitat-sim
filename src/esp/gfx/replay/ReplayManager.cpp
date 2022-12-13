// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayManager.h"

namespace esp {
namespace gfx {
namespace replay {

std::shared_ptr<Player> ReplayManager::readKeyframesFromFile(
    const std::string& filepath) {
  auto player = std::make_shared<Player>(playerImplementation_);
  player->readKeyframesFromFile(filepath);
  if (player->getNumKeyframes() == 0) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Failed to load any keyframes from [" << filepath << "]";
    return nullptr;
  }
  return player;
}

std::shared_ptr<Player> ReplayManager::createEmptyPlayer() {
  auto player = std::make_shared<Player>(playerImplementation_);
  return player;
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
