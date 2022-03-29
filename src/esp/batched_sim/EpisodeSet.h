// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_EPISODE_H_
#define ESP_BATCHEDSIM_EPISODE_H_

#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/SerializeCollection.h"

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Range.h>

#include <vector>
#include <string>

namespace esp {
namespace batched_sim {

// todo: more careful namespace or nested classes

struct CollisionSphere {
  Magnum::Vector3 origin;
  int radiusIdx = -1; // see BatchedSimulator::getCollisionRadius()
};

class FreeObject {
 public:
  std::string name_;
  BpsSceneMapping::InstanceBlueprint instanceBlueprint_;
  Magnum::Range3D aabb_;
  std::vector<Magnum::Matrix3x3> startRotations_;
  int heldRotationIndex_; // index into startRotations_
  std::vector<CollisionSphere> collisionSpheres_;
  bool needsPostLoadFixup_ = true;
};

class FixedObject {
 public:
  std::string name_;
  bool needsPostLoadFixup_ = true;
  BpsSceneMapping::InstanceBlueprint instanceBlueprint_;
  esp::batched_sim::ColumnGridSet columnGridSet_;
};

class FreeObjectSpawn {
 public:
  int16_t freeObjIndex_;
  int16_t startRotationIndex_;
  Mn::Vector3 startPos_; // discretize?
};

class Episode {
 public:
  int32_t stageFixedObjIndex = -1; // todo: array of fixed objects
  int16_t numFreeObjectSpawns_ = 0;
  int16_t targetObjIndex_ = -1; // 0..numFreeObjectSpawns - 1, see also freeObjectIndex
  int32_t firstFreeObjectSpawnIndex_ = -1; // index into EpisodeSet::freeObjectSpawns_
  Mn::Vector2 agentStartPos_;
  float agentStartYaw_ = 0.f; // radians

  // task-specific
  Magnum::Vector3 targetObjGoalPos_;
};

class EpisodeSet {
 public:
  std::vector<Episode> episodes_; // ~50,000
  std::vector<FixedObject> fixedObjects_; // ~100, max 32K
  std::vector<FreeObjectSpawn> freeObjectSpawns_; // num episodes * num-spawns-per-episode, ~5,000,000
  std::vector<FreeObject> freeObjects_; // ~100, max 32K
  int maxFreeObjects_ = -1;
  bool needsPostLoadFixup_ = true;

  static EpisodeSet loadFromFile(const std::string& filepath);
  void saveToFile(const std::string& filepath) const;
};

class EpisodeInstance {
 public:
  int32_t episodeIndex_ = -1;
  int32_t stageFixedObjectInstanceId_ = -1;
  // free obj instance ids stored in freeObjectInstanceIds_
  CollisionBroadphaseGrid colGrid_;
  int debugNumColGridObstacleInstances_ = 0;
  // todo: more robust storage for moved free objects
  static constexpr int MAX_MOVED_FREE_OBJECTS = 6;
  // todo: better memory management
  std::vector<int16_t> movedFreeObjectIndexes_;
  int firstFreeObjectInstanceId_ = -1;
};

class EpisodeInstanceSet {
 public:
  std::vector<EpisodeInstance> episodeInstanceByEnv_; // num envs, ~1,000
};

void updateFromSerializeCollection(EpisodeSet& set, const serialize::Collection& collection);

// todo: move to separate file
EpisodeSet generateBenchmarkEpisodeSet(int numEpisodes, 
  const BpsSceneMapping& sceneMapping, const serialize::Collection& collection);

void postLoadFixup(EpisodeSet& set, const BpsSceneMapping& sceneMapping, 
  const serialize::Collection& collection);

}  // namespace batched_sim
}  // namespace esp

#endif
