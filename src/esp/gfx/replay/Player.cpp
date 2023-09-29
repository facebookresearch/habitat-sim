// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Player.h"

#include <Corrade/Containers/StringStl.h>
#include <Corrade/Utility/Path.h>

#include "esp/io/Json.h"

namespace esp {
namespace gfx {
namespace replay {

namespace {

// At recording time, material overrides gets stringified and appended to the
// filepath. See ResourceManager::createModifiedAssetName.
// AbstractSceneGraphPlayerImplementation doesn't support parsing this material
// info. More info at
// https://docs.google.com/document/d/1ngA73cXl3YRaPfFyICSUHONZN44C-XvieS7kwyQDbkI/edit#bookmark=id.aoe7xgsro2r7
std::string removeMaterialOverrideFromFilepathAndWarn(const std::string& src) {
  auto pos = src.find('?');
  if (pos != std::string::npos) {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "Ignoring material-override for [" << src << "]";

    return src.substr(0, pos);
  } else {
    return src;
  }
}

}  // namespace

static_assert(std::is_nothrow_move_constructible<Player>::value, "");

void AbstractPlayerImplementation::setNodeSemanticId(NodeHandle, unsigned) {}

void AbstractPlayerImplementation::changeLightSetup(const LightSetup&) {}

void AbstractSceneGraphPlayerImplementation::deleteAssetInstance(
    const NodeHandle node) {
  // TODO: use NodeDeletionHelper to safely delete nodes owned by the Player.
  // the deletion here is unsafe because a Player may persist beyond the
  // lifetime of these nodes.
  delete reinterpret_cast<scene::SceneNode*>(node);
}

void AbstractSceneGraphPlayerImplementation::deleteAssetInstances(
    const std::unordered_map<RenderAssetInstanceKey, NodeHandle>& instances) {
  for (const auto& pair : instances) {
    delete reinterpret_cast<scene::SceneNode*>(pair.second);
  }
}

void AbstractSceneGraphPlayerImplementation::setNodeTransform(
    const NodeHandle node,
    const Mn::Vector3& translation,
    const Mn::Quaternion& rotation) {
  (*reinterpret_cast<scene::SceneNode*>(node))
      .setTranslation(translation)
      .setRotation(rotation);
}

void AbstractSceneGraphPlayerImplementation::setNodeTransform(
    const NodeHandle node,
    const Mn::Matrix4& transform) {
  (*reinterpret_cast<scene::SceneNode*>(node)).setTransformation(transform);
}

Mn::Matrix4 AbstractSceneGraphPlayerImplementation::hackGetNodeTransform(
    const NodeHandle node) const {
  return (*reinterpret_cast<scene::SceneNode*>(node)).transformation();
}

void AbstractSceneGraphPlayerImplementation::setNodeSemanticId(
    const NodeHandle node,
    const unsigned id) {
  setSemanticIdForSubtree(reinterpret_cast<scene::SceneNode*>(node), id);
}

void AbstractPlayerImplementation::createRigInstance(
    int,
    const std::vector<std::string>&) {}

void AbstractPlayerImplementation::deleteRigInstance(int) {}

void AbstractPlayerImplementation::setRigPose(
    int,
    const std::vector<gfx::replay::Transform>&) {}

void Player::readKeyframesFromJsonDocument(const rapidjson::Document& d) {
  CORRADE_INTERNAL_ASSERT(keyframes_.empty());
  esp::io::readMember(d, "keyframes", keyframes_);
}

Keyframe Player::keyframeFromString(const std::string& keyframe) {
  Keyframe res;
  rapidjson::Document d;
  d.Parse<0>(keyframe.data(), keyframe.size());
  esp::io::readMember(d, "keyframe", res);
  return res;
}

Keyframe Player::keyframeFromStringUnwrapped(
    const Cr::Containers::StringView keyframe) {
  Keyframe res;
  rapidjson::Document d;
  d.Parse<0>(keyframe.data(), keyframe.size());
  esp::io::fromJsonValue(d, res);
  return res;
}

Player::Player(std::shared_ptr<AbstractPlayerImplementation> implementation)
    : implementation_{std::move(implementation)} {}

void Player::readKeyframesFromFile(const std::string& filepath) {
  close();

  if (!Corrade::Utility::Path::exists(filepath)) {
    ESP_ERROR() << "File" << filepath << "not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    readKeyframesFromJsonDocument(newDoc);
  } catch (...) {
    ESP_ERROR() << "Failed to parse keyframes from" << filepath << ".";
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
  CORRADE_INTERNAL_ASSERT(frameIndex == -1 ||
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
  CORRADE_INTERNAL_ASSERT(frameIndex_ >= 0 && frameIndex_ < getNumKeyframes());
  CORRADE_INTERNAL_ASSERT(translation);
  CORRADE_INTERNAL_ASSERT(rotation);
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
  /* In a moved-out Player the implementation_ shared_ptr becomes null for
     some reason (why, C++?), and since clearFrame() is called on destruction
     accessing it will blow up. So it's a destructive move, yes. */
  if (implementation_)
    implementation_->deleteAssetInstances(createdInstances_);
  createdInstances_.clear();
  assetInfos_.clear();
  creationInfos_.clear();
  frameIndex_ = -1;
}

void Player::applyKeyframe(const Keyframe& keyframe) {
  for (const auto& assetInfo : keyframe.loads) {
    if (failedFilepaths_.count(assetInfo.filepath) != 0u) {
      continue;
    }
    // TODO: This overwrites the previous AssetInfo. This is not ideal. Consider
    // including AssetInfo in creations rather than using keyframe loads.
    assetInfos_[assetInfo.filepath] = assetInfo;
  }

  for (const auto& rigCreation : keyframe.rigCreations) {
    implementation_->createRigInstance(rigCreation.id, rigCreation.boneNames);
  }

  for (const auto& pair : keyframe.creations) {
    const auto& creation = pair.second;

    auto adjustedFilepath =
        removeMaterialOverrideFromFilepathAndWarn(creation.filepath);

    if (assetInfos_.count(adjustedFilepath) == 0u) {
      if (failedFilepaths_.count(adjustedFilepath) == 0u) {
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Missing asset info for [" << adjustedFilepath << "]";
        failedFilepaths_.insert(adjustedFilepath);
      }
      continue;
    }
    CORRADE_INTERNAL_ASSERT(assetInfos_.count(adjustedFilepath));
    auto adjustedCreation = creation;
    adjustedCreation.filepath = adjustedFilepath;
    auto* node = implementation_->loadAndCreateRenderAssetInstance(
        assetInfos_[adjustedFilepath], adjustedCreation);
    if (!node) {
      if (failedFilepaths_.count(adjustedFilepath) == 0u) {
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Load failed for asset [" << adjustedFilepath << "]";
        failedFilepaths_.insert(adjustedFilepath);
      }
      continue;
    }

    const auto& instanceKey = pair.first;
    CORRADE_INTERNAL_ASSERT(createdInstances_.count(instanceKey) == 0);
    createdInstances_[instanceKey] = node;
    creationInfos_[instanceKey] = adjustedCreation;
  }

  hackProcessDeletions(keyframe);

  for (const auto& pair : keyframe.stateUpdates) {
    const auto& it = createdInstances_.find(pair.first);
    if (it == createdInstances_.end()) {
      // Missing instance for this key due to a failed instance creation
      continue;
    }
    auto* node = it->second;
    const auto& state = pair.second;
    implementation_->setNodeTransform(node, state.absTransform.translation,
                                      state.absTransform.rotation);
    implementation_->setNodeSemanticId(node, state.semanticId);
  }

  for (const auto& rigUpdate : keyframe.rigUpdates) {
    implementation_->setRigPose(rigUpdate.id, rigUpdate.pose);
  }

  if (keyframe.lightsChanged) {
    implementation_->changeLightSetup(keyframe.lights);
  }
}

void Player::hackProcessDeletions(const Keyframe& keyframe) {
  // HACK: Classic and batch renderers currently handle deletions differently.
  // The batch renderer can only clear the scene entirely; it cannot delete
  // individual objects. To process deletions, all instances are deleted,
  // remaining instances are re-created and latest transform updates are
  // re-applied.
  bool isClassicReplayRenderer =
      dynamic_cast<AbstractSceneGraphPlayerImplementation*>(
          implementation_.get()) != nullptr;
  if (isClassicReplayRenderer) {
    for (const auto& deletionInstanceKey : keyframe.deletions) {
      const auto& it = createdInstances_.find(deletionInstanceKey);
      if (it == createdInstances_.end()) {
        // Missing instance for this key due to a failed instance creation
        continue;
      }

      implementation_->deleteAssetInstance(it->second);
      createdInstances_.erase(deletionInstanceKey);

      int rigId = creationInfos_[deletionInstanceKey].rigId;
      if (rigId != ID_UNDEFINED) {
        implementation_->deleteRigInstance(rigId);
      }
      creationInfos_.erase(deletionInstanceKey);
    }
  } else if (keyframe.deletions.size() > 0) {
    // Cache latest transforms
    latestTransformCache_.clear();
    for (const auto& pair : this->createdInstances_) {
      const RenderAssetInstanceKey key = pair.first;
      latestTransformCache_[key] =
          implementation_->hackGetNodeTransform(pair.second);
    }

    // Delete all instances
    implementation_->deleteAssetInstances(createdInstances_);

    // Remove deleted instances from records
    for (const auto& deletion : keyframe.deletions) {
      const auto& createInstanceIt = createdInstances_.find(deletion);
      if (createInstanceIt == createdInstances_.end()) {
        // Missing instance for this key due to a failed instance creation
        continue;
      }
      createdInstances_.erase(createInstanceIt);
      creationInfos_.erase(deletion);
    }

    for (const auto& pair : createdInstances_) {
      const auto key = pair.first;
      const auto& creationInfo = creationInfos_[key];
      auto* instance = implementation_->loadAndCreateRenderAssetInstance(
          assetInfos_[creationInfo.filepath], creationInfo);

      // Replace dangling reference
      createdInstances_[key] = instance;

      // Re-apply latest transform updates
      implementation_->setNodeTransform(instance, latestTransformCache_[key]);
    }

    // TODO: Handle batch rendered rigs
  }
}

void Player::appendKeyframe(Keyframe&& keyframe) {
  keyframes_.emplace_back(std::move(keyframe));
}

void Player::appendJSONKeyframe(const std::string& keyframe) {
  appendKeyframe(keyframeFromString(keyframe));
}

void Player::setSingleKeyframe(Keyframe&& keyframe) {
  keyframes_.clear();
  frameIndex_ = -1;
  keyframes_.emplace_back(std::move(keyframe));
  setKeyframeIndex(0);
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
