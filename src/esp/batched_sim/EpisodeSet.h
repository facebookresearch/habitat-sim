// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_EPISODE_H_
#define ESP_BATCHEDSIM_EPISODE_H_

#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/batched_sim/BpsSceneMapping.h"

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Range.h>

#include <vector>
#include <string>

namespace esp {
namespace batched_sim {

// todo: more careful namespace or nested classes

class FreeObject {
 public:
  std::string name_;
  BpsSceneMapping::InstanceBlueprint instanceBlueprint_;
  Magnum::Range3D aabb_;
  // float boundingSphereRadiusSq_ = 0.f; // unused
  // std::vector<Magnum::Vector3> collisionSphereLocalOrigins_;
  std::vector<Magnum::Matrix3x3> startRotations_;
};

class FixedObject {
 public:
  std::string name_;
  BpsSceneMapping::InstanceBlueprint instanceBlueprint_;
  esp::batched_sim::ColumnGridSource columnGrid_;
  // temp: only used for stage right now
  // todo: bounding box and transform/invTransform
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
  int32_t numFreeObjectSpawns_ = 0;
  int32_t firstFreeObjectSpawnIndex_ = -1; // index into EpisodeSet::freeObjectSpawns_
};

class EpisodeSet {
 public:
  std::vector<Episode> episodes_; // ~50,000
  std::vector<FixedObject> fixedObjects_; // ~100, max 32K
  std::vector<FreeObjectSpawn> freeObjectSpawns_; // num episodes * num-spawns-per-episode, ~5,000,000
  std::vector<FreeObject> freeObjects_; // ~100, max 32K
  int maxFreeObjects_ = 0;
};

class EpisodeInstance {
 public:
  int32_t episodeIndex_ = -1;
  int32_t stageFixedObjectInstanceId_ = -1;
  // free obj instance ids stored in freeObjectInstanceIds_
  CollisionBroadphaseGrid colGrid_;
};

class EpisodeInstanceSet {
 public:
  std::vector<EpisodeInstance> episodeInstanceByEnv_; // num envs, ~1,000
  // todo: move this back into EpisodeInstance?
  std::vector<int16_t> freeObjectInstanceIds_; // num envs * maxFreeObjects, ~100,000
};

// todo: move to separate file
EpisodeSet generateBenchmarkEpisodeSet(int numEpisodes, const BpsSceneMapping& sceneMapping);

}  // namespace batched_sim
}  // namespace esp

#endif
