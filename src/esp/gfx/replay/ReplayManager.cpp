// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayManager.h"

namespace esp {
namespace gfx {
namespace replay {

std::shared_ptr<Player> ReplayManager::readKeyframesFromFile(
    const std::string& filepath) {
  auto player = std::make_shared<Player>(playerCallback_);
  player->readKeyframesFromFile(filepath);
  if (!player->getNumKeyframes()) {
    LOG(ERROR) << "ReplayManager::readKeyframesFromFile: failed to load any "
                  "keyframes from ["
               << filepath << "]";
    return nullptr;
  }
  return player;
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
