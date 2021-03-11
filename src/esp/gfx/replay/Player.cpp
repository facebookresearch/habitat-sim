// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Player.h"

#include "esp/assets/ResourceManager.h"
#include "esp/core/esp.h"
#include "esp/io/JsonAllTypes.h"

#include <rapidjson/document.h>

namespace esp {
namespace gfx {
namespace replay {

void Player::readKeyframesFromJsonDocument(const rapidjson::Document& d) {
  ASSERT(keyframes_.empty());
  esp::io::readMember(d, "keyframes", keyframes_);
}

Player::Player(const LoadAndCreateRenderAssetInstanceCallback& callback)
    : loadAndCreateRenderAssetInstanceCallback(callback) {}

void Player::readKeyframesFromFile(const std::string& filepath) {
  close();

  if (!Corrade::Utility::Directory::exists(filepath)) {
    LOG(ERROR) << "Player::readKeyframesFromFile: file " << filepath
               << " not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    readKeyframesFromJsonDocument(newDoc);
  } catch (...) {
    LOG(ERROR)
        << "Player::readKeyframesFromFile: failed to parse keyframes from "
        << filepath << ".";
  }
}

Player::~Player() {
  clearFrame();
}

int Player::getKeyframeIndex() const {
  return frameIndex_;
}

int Player::getNumKeyframes() const {
  return keyframes_.size();
}

void Player::setKeyframeIndex(int frameIndex) {
  ASSERT(frameIndex == -1 ||
         (frameIndex >= 0 && frameIndex < getNumKeyframes()));

  if (frameIndex < frameIndex_) {
    clearFrame();
  }

  while (frameIndex_ < frameIndex) {
    applyKeyframe(keyframes_[++frameIndex_]);
  }
}

bool Player::getUserTransform(const std::string& name,
                              Magnum::Vector3* translation,
                              Magnum::Quaternion* rotation) const {
  ASSERT(frameIndex_ >= 0 && frameIndex_ < getNumKeyframes());
  ASSERT(translation);
  ASSERT(rotation);
  const auto& keyframe = keyframes_[frameIndex_];
  const auto& it = keyframe.userTransforms.find(name);
  if (it != keyframe.userTransforms.end()) {
    *translation = it->second.translation;
    *rotation = it->second.rotation;
    return true;
  } else {
    return false;
  }
}

void Player::close() {
  clearFrame();
  keyframes_.clear();
}

void Player::clearFrame() {
  for (const auto& pair : createdInstances_) {
    // TODO: use NodeDeletionHelper to safely delete nodes owned by the Player.
    // the deletion here is unsafe because a Player may persist beyond the
    // lifetime of these nodes.
    delete pair.second;
  }
  createdInstances_.clear();
  assetInfos_.clear();
  frameIndex_ = -1;
}

void Player::applyKeyframe(const Keyframe& keyframe) {
  for (const auto& assetInfo : keyframe.loads) {
    ASSERT(assetInfos_.count(assetInfo.filepath) == 0);
    if (failedFilepaths_.count(assetInfo.filepath)) {
      continue;
    }
    assetInfos_[assetInfo.filepath] = assetInfo;
  }

  for (const auto& pair : keyframe.creations) {
    const auto& creation = pair.second;
    if (!assetInfos_.count(creation.filepath)) {
      if (!failedFilepaths_.count(creation.filepath)) {
        LOG(WARNING) << "Player: missing asset info for [" << creation.filepath
                     << "]";
        failedFilepaths_.insert(creation.filepath);
      }
      continue;
    }
    ASSERT(assetInfos_.count(creation.filepath));
    auto node = loadAndCreateRenderAssetInstanceCallback(
        assetInfos_[creation.filepath], creation);
    if (!node) {
      if (!failedFilepaths_.count(creation.filepath)) {
        LOG(WARNING) << "Player: load failed for asset [" << creation.filepath
                     << "]";
        failedFilepaths_.insert(creation.filepath);
      }
      continue;
    }

    const auto& instanceKey = pair.first;
    ASSERT(createdInstances_.count(instanceKey) == 0);
    createdInstances_[instanceKey] = node;
  }

  for (const auto& deletionInstanceKey : keyframe.deletions) {
    const auto& it = createdInstances_.find(deletionInstanceKey);
    if (it == createdInstances_.end()) {
      // missing instance for this key, probably due to a failed instance
      // creation
      continue;
    }

    auto node = it->second;
    delete node;
    createdInstances_.erase(deletionInstanceKey);
  }

  for (const auto& pair : keyframe.stateUpdates) {
    const auto& it = createdInstances_.find(pair.first);
    if (it == createdInstances_.end()) {
      // missing instance for this key, probably due to a failed instance
      // creation
      continue;
    }
    auto node = it->second;
    const auto& state = pair.second;
    node->setTranslation(state.absTransform.translation);
    node->setRotation(state.absTransform.rotation);
    setSemanticIdForSubtree(node, state.semanticId);
  }
}

void Player::setSemanticIdForSubtree(esp::scene::SceneNode* rootNode,
                                     int semanticId) {
  if (rootNode->getSemanticId() == semanticId) {
    // We assume the entire subtree's semanticId matches the root's, so we can
    // early out here.
    return;
  }

  // See also RigidBase setSemanticId. That function uses a prepared container
  // of visual nodes, whereas this function traverses the subtree to touch all
  // nodes (including visual nodes). The results should be the same.
  auto cb = [&](esp::scene::SceneNode& node) {
    node.setSemanticId(semanticId);
  };
  esp::scene::preOrderTraversalWithCallback(*rootNode, cb);
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
