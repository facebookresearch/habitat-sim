// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Recorder.h"

#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/core/Check.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/SkinData.h"
#include "esp/io/Json.h"
#include "esp/io/JsonAllTypes.h"
#include "esp/scene/SceneNode.h"

namespace {
esp::gfx::replay::Transform createReplayTransform(
    const Magnum::Matrix4& absTransformMat) {
  auto rotationShear = absTransformMat.rotationShear();
  // Remove reflection (negative scaling) from the matrix. We assume
  // constant node scaling for the node's lifetime. It is baked into
  // instance-creation so it doesn't need to be saved into
  // RenderAssetInstanceState. See also onCreateRenderAssetInstance.
  if (rotationShear.determinant() < 0.0f) {
    rotationShear[0] *= -1.f;
  }

  return esp::gfx::replay::Transform{
      absTransformMat.translation(),
      Magnum::Quaternion::fromMatrix(rotationShear)};
};
}  // namespace

namespace esp {
namespace gfx {
namespace replay {

/**
 * @brief Helper class to get notified when a SceneNode is about to be
 * destroyed.
 */
class NodeDeletionHelper : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  NodeDeletionHelper(scene::SceneNode& node_, Recorder* writer)
      : Magnum::SceneGraph::AbstractFeature3D(node_),
        recorder_(writer),
        node(&node_) {}

  ~NodeDeletionHelper() override {
    recorder_->onDeleteRenderAssetInstance(node);
  }

 private:
  Recorder* recorder_ = nullptr;
  const scene::SceneNode* node = nullptr;
};

Recorder::~Recorder() {
  // Delete NodeDeletionHelpers. This is important because they hold raw
  // pointers to this Recorder and these pointers would become dangling
  // (invalid) after this Recorder is destroyed.
  for (auto& instanceRecord : instanceRecords_) {
    delete instanceRecord.deletionHelper;
  }
}

void Recorder::onLoadRenderAsset(const esp::assets::AssetInfo& assetInfo) {
  getKeyframe().loads.push_back(assetInfo);
}

void Recorder::onCreateRenderAssetInstance(
    scene::SceneNode* node,
    const esp::assets::RenderAssetInstanceCreationInfo& creation) {
  CORRADE_INTERNAL_ASSERT(node);
  CORRADE_INTERNAL_ASSERT(findInstance(node) == ID_UNDEFINED);

  RenderAssetInstanceKey instanceKey = getNewInstanceKey();
  getKeyframe().creations.emplace_back(instanceKey, creation);

  // Constructing NodeDeletionHelper here is equivalent to calling
  // node->addFeature. We keep a pointer to deletionHelper so we can delete it
  // manually later if necessary.
  NodeDeletionHelper* deletionHelper = new NodeDeletionHelper{*node, this};

  instanceRecords_.emplace_back(InstanceRecord{node, instanceKey,
                                               Corrade::Containers::NullOpt,
                                               deletionHelper, creation.rigId});
}

void Recorder::onCreateRigInstance(int rigId, const Rig& rig) {
  rigNodes_[rigId] = rig.bones;

  RigCreation rigCreation;
  rigCreation.id = rigId;
  rigCreation.boneNames.resize(rig.bones.size());
  for (const auto& boneNamePair : rig.boneNames) {
    rigCreation.boneNames[boneNamePair.second] = boneNamePair.first;
  }
  currKeyframe_.rigCreations.emplace_back(std::move(rigCreation));
}

void Recorder::onHideSceneGraph(const esp::scene::SceneGraph& sceneGraph) {
  const auto& root = sceneGraph.getRootNode();
  scene::preOrderTraversalWithCallback(
      root, [this](const scene::SceneNode& node) {
        int index = findInstance(&node);
        if (index != ID_UNDEFINED) {
          delete instanceRecords_[index].deletionHelper;
        }
      });
}

void Recorder::saveKeyframe() {
  updateStates();
  advanceKeyframe();
}

Keyframe Recorder::extractKeyframe() {
  updateStates();
  auto retVal = std::move(currKeyframe_);
  currKeyframe_ = Keyframe{};
  return retVal;
}

const Keyframe& Recorder::getLatestKeyframe() {
  CORRADE_ASSERT(!savedKeyframes_.empty(),
                 "Recorder::getLatestKeyframe() : Trying to access latest "
                 "keyframe when there are none",
                 savedKeyframes_.back());
  return savedKeyframes_.back();
}

void Recorder::addUserTransformToKeyframe(const std::string& name,
                                          const Magnum::Vector3& translation,
                                          const Magnum::Quaternion& rotation) {
  getKeyframe().userTransforms[name] = Transform{translation, rotation};
}

void Recorder::addLightToKeyframe(const LightInfo& lightInfo) {
  getKeyframe().lightsChanged = true;
  getKeyframe().lights.emplace_back(lightInfo);
}

void Recorder::clearLightsFromKeyframe() {
  getKeyframe().lightsChanged = true;
  getKeyframe().lights.clear();
}

void Recorder::addLoadsCreationsDeletions(KeyframeIterator begin,
                                          KeyframeIterator end,
                                          Keyframe* dest) {
  CORRADE_INTERNAL_ASSERT(dest);
  for (KeyframeIterator curr = begin; curr != end; ++curr) {
    const auto& keyframe = *curr;
    dest->loads.insert(dest->loads.end(), keyframe.loads.begin(),
                       keyframe.loads.end());
    dest->creations.insert(dest->creations.end(), keyframe.creations.begin(),
                           keyframe.creations.end());
    for (const auto& deletionInstanceKey : keyframe.deletions) {
      checkAndAddDeletion(dest, deletionInstanceKey);
    }
  }
}

void Recorder::checkAndAddDeletion(Keyframe* keyframe,
                                   RenderAssetInstanceKey instanceKey) {
  auto it =
      std::find_if(keyframe->creations.begin(), keyframe->creations.end(),
                   [&](const auto& pair) { return pair.first == instanceKey; });
  if (it != keyframe->creations.end()) {
    // this deletion just cancels out with an earlier creation
    keyframe->creations.erase(it);
  } else {
    // This deletion has no matching creation so it can't be canceled out.
    // Include it in the keyframe.
    keyframe->deletions.push_back(instanceKey);
  }
}

void Recorder::onDeleteRenderAssetInstance(const scene::SceneNode* node) {
  int index = findInstance(node);
  CORRADE_INTERNAL_ASSERT(index != ID_UNDEFINED);

  const auto& instanceRecord = instanceRecords_[index];
  const auto& instanceKey = instanceRecord.instanceKey;
  const auto& rigId = instanceRecord.rigId;

  checkAndAddDeletion(&getKeyframe(), instanceKey);

  instanceRecords_.erase(instanceRecords_.begin() + index);
  rigNodes_.erase(rigId);
  rigNodeTransformCache_.erase(rigId);
}

Keyframe& Recorder::getKeyframe() {
  return currKeyframe_;
}

RenderAssetInstanceKey Recorder::getNewInstanceKey() {
  return nextInstanceKey_++;
}

int Recorder::findInstance(const scene::SceneNode* queryNode) {
  auto it = std::find_if(instanceRecords_.begin(), instanceRecords_.end(),
                         [&queryNode](const InstanceRecord& record) {
                           return record.node == queryNode;
                         });

  return it == instanceRecords_.end()
             ? ID_UNDEFINED
             : static_cast<int>(it - instanceRecords_.begin());
}

RenderAssetInstanceState Recorder::getInstanceState(
    const scene::SceneNode* node) {
  const auto absTransformMat = node->absoluteTransformation();
  const auto transform = ::createReplayTransform(absTransformMat);
  return RenderAssetInstanceState{transform, node->getSemanticId()};
}

void Recorder::updateStates() {
  updateInstanceStates();
  updateRigInstanceStates();
}

void Recorder::updateInstanceStates() {
  for (auto& instanceRecord : instanceRecords_) {
    auto state = getInstanceState(instanceRecord.node);
    if (!instanceRecord.recentState || state != instanceRecord.recentState) {
      getKeyframe().stateUpdates.emplace_back(instanceRecord.instanceKey,
                                              state);
      instanceRecord.recentState = state;
    }
  }
}

void Recorder::updateRigInstanceStates() {
  for (const auto& rigItr : rigNodes_) {
    const int rigId = rigItr.first;
    const int boneCount = rigItr.second.size();

    RigUpdate rigUpdate;
    rigUpdate.id = rigId;

    const auto cacheIt = rigNodeTransformCache_.find(rigId);
    if (cacheIt == rigNodeTransformCache_.end()) {
      rigNodeTransformCache_[rigId] = std::vector<Mn::Matrix4>(boneCount);
    }

    // If a single bone pose was updated, update the entire pose
    bool updated = false;
    for (int boneIdx = 0; boneIdx < boneCount; ++boneIdx) {
      const auto* bone = rigItr.second[boneIdx];
      const auto absTransformMat = bone->absoluteTransformation();
      if (rigNodeTransformCache_[rigId][boneIdx] != absTransformMat) {
        updated = true;
        break;
      }
    }

    // Update the pose
    if (updated) {
      rigUpdate.pose.reserve(boneCount);
      for (int boneIdx = 0; boneIdx < boneCount; ++boneIdx) {
        const auto* bone = rigItr.second[boneIdx];
        const auto absTransformMat = bone->absoluteTransformation();
        rigNodeTransformCache_[rigId][boneIdx] = absTransformMat;
        rigUpdate.pose.emplace_back(createReplayTransform(absTransformMat));
      }
      currKeyframe_.rigUpdates.emplace_back(std::move(rigUpdate));
    }
  }
}

void Recorder::advanceKeyframe() {
  savedKeyframes_.emplace_back(std::move(currKeyframe_));
  currKeyframe_ = Keyframe{};
}

void Recorder::writeSavedKeyframesToFile(const std::string& filepath,
                                         bool usePrettyWriter) {
  auto document = writeKeyframesToJsonDocument();
  // replay::Keyframes use floats (not doubles) so this is plenty of precision
  const int maxDecimalPlaces = 7;
  auto ok = esp::io::writeJsonToFile(document, filepath, usePrettyWriter,
                                     maxDecimalPlaces);
  ESP_CHECK(ok, "writeSavedKeyframesToFile: unable to write to " << filepath);

  consolidateSavedKeyframes();
}

std::string Recorder::writeSavedKeyframesToString() {
  auto document = writeKeyframesToJsonDocument();

  consolidateSavedKeyframes();

  return esp::io::jsonToString(document);
}

std::vector<std::string>
Recorder::writeIncrementalSavedKeyframesToStringArray() {
  std::vector<std::string> results;
  results.reserve(savedKeyframes_.size());

  for (const auto& keyframe : savedKeyframes_) {
    results.emplace_back(keyframeToString(keyframe));
  }

  // note we don't call consolidateSavedKeyframes. Use this function if you are
  // using keyframes incrementally, e.g. repeated calls to this function and
  // feeding them to a renderer. Contrast with writeSavedKeyframesToFile, which
  // "consolidates" before discarding old keyframes to avoid losing state
  // information.
  savedKeyframes_.clear();

  return results;
}

std::string Recorder::keyframeToString(const Keyframe& keyframe) {
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  esp::io::addMember(d, "keyframe", keyframe, allocator);
  return esp::io::jsonToString(d);
}

void Recorder::consolidateSavedKeyframes() {
  // consolidate saved keyframes into current keyframe
  addLoadsCreationsDeletions(savedKeyframes_.begin(), savedKeyframes_.end(),
                             &getKeyframe());
  // clear instanceRecord.recentState to ensure updates get included in the next
  // saved keyframe.
  for (auto& instanceRecord : instanceRecords_) {
    instanceRecord.recentState = Corrade::Containers::NullOpt;
  }
  savedKeyframes_.clear();
}

rapidjson::Document Recorder::writeKeyframesToJsonDocument() {
  if (savedKeyframes_.empty()) {
    ESP_WARNING() << "No saved keyframes to write";
    return rapidjson::Document();
  }

  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  esp::io::addMember(d, "keyframes", savedKeyframes_, allocator);
  return d;
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp
