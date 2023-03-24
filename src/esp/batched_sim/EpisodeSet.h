// Copyright (c) Meta Platforms, Inc. All Rights Reserved
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_EPISODESET_H_
#define ESP_BATCHEDSIM_EPISODESET_H_

#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/SerializeCollection.h"

#include <Magnum/Magnum.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Vector3.h>

#include <glm/gtx/transform.hpp>
#include <string>
#include <vector>

//#define EPISODESET_DISCRETE_SPAWN_ROTATIONS

namespace esp {
namespace batched_sim {

// todo: more careful namespace or nested classes

struct CollisionSphere {
  Magnum::Vector3 origin;
  int radiusIdx = -1;  // see BatchedSimulator::getCollisionRadius()
};

class RenderAsset {
 public:
  std::string name_;
  BpsSceneMapping::InstanceBlueprint instanceBlueprint_;
  bool needsPostLoadFixup_ = true;
};

class FreeObject {
 public:
  int renderAssetIndex_ = -1;
  Magnum::Range3D aabb_;
  std::vector<Magnum::Quaternion> startRotations_;
  int heldRotationIndex_;  // index into startRotations_
  std::vector<CollisionSphere> collisionSpheres_;
  bool needsPostLoadFixup_ = true;
};

class Transform {
 public:
  Magnum::Vector3 origin_;
  Magnum::Quaternion rotation_;
  Magnum::Vector3 scale_;
};

class RenderAssetInstance {
 public:
  int renderAssetIndex_;
  Transform transform_;  // for serialization
  glm::mat4x3 glMat_;    // computed from transform_
};

class StaticScene {
 public:
  std::string name_;
  std::vector<RenderAssetInstance> renderAssetInstances_;
  std::string columnGridSetName_;

  bool needsPostLoadFixup_ = true;  // todo: get rid of all these bools
  esp::batched_sim::ColumnGridSet columnGridSet_;
};

class FreeObjectSpawn {
 public:
  int16_t freeObjIndex_;
#ifdef EPISODESET_DISCRETE_SPAWN_ROTATIONS
  int16_t startRotationIndex_;
#else
  Magnum::Quaternion startRotation_;
#endif
  Magnum::Vector3 startPos_;  // discretize?
};

class Episode {
 public:
  int16_t staticSceneIndex_ = -1;
  int16_t numFreeObjectSpawns_ = 0;
  int16_t targetObjIndex_ =
      -1;  // 0..numFreeObjectSpawns - 1, see also freeObjectIndex
  int32_t firstFreeObjectSpawnIndex_ =
      -1;  // index into EpisodeSet::freeObjectSpawns_
  Magnum::Vector2 agentStartPos_ = Magnum::Vector2(Magnum::Math::ZeroInit);
  float agentStartYaw_ = 0.f;  // radians
  static constexpr int MAX_JOINT_POSITIONS = 7;
  std::array<float, MAX_JOINT_POSITIONS> robotStartJointPositions_;

  // task-specific
  Magnum::Vector3 targetObjGoalPos_ = Magnum::Vector3(Magnum::Math::ZeroInit);
  Magnum::Quaternion targetObjGoalRotation_ =
      Magnum::Quaternion(Magnum::Math::IdentityInit);
};

class EpisodeSet {
 public:
  std::vector<Episode> episodes_;          // ~50,000
  std::vector<RenderAsset> renderAssets_;  // ~1000, no max
  std::vector<StaticScene> staticScenes_;  // ~100, max 32K
  std::vector<FreeObjectSpawn>
      freeObjectSpawns_;  // num episodes * num-spawns-per-episode, ~5,000,000
  std::vector<FreeObject> freeObjects_;  // ~100, max 32K
  int maxFreeObjects_ = -1;
  Magnum::Range3D allEpisodesAABB_;
  bool needsPostLoadFixup_ = true;

  static EpisodeSet loadFromFile(const std::string& filepath);
  void saveToFile(const std::string& filepath) const;
};

class EpisodeInstance {
 public:
  int32_t episodeIndex_ = -1;
  std::vector<int32_t> staticSceneInstanceIds_;
  // free obj instance ids stored in freeObjectInstanceIds_
  CollisionBroadphaseGrid colGrid_;
  // todo: more robust storage for moved free objects
  static constexpr int MAX_MOVED_FREE_OBJECTS = 6;
  // todo: better memory management
  int firstFreeObjectInstanceId_ = -1;
  std::vector<int> persistentDebugInstanceIds_;
};

class EpisodeInstanceSet {
 public:
  std::vector<EpisodeInstance> episodeInstanceByEnv_;  // num envs, ~1,000
};

void updateFromSerializeCollection(EpisodeSet& set,
                                   const serialize::Collection& collection);

void updateFromSerializeCollection(FreeObject& freeObject,
                                   const serialize::FreeObject& serFreeObject,
                                   const serialize::Collection& collection);

void postLoadFixup(EpisodeSet& set,
                   const BpsSceneMapping& sceneMapping,
                   const serialize::Collection& collection);

}  // namespace batched_sim
}  // namespace esp

#endif
