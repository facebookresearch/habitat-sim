// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_EPISODEGENERATOR_H_
#define ESP_BATCHEDSIM_EPISODEGENERATOR_H_

#include "esp/batched_sim/EpisodeSet.h"

namespace esp {
namespace batched_sim {

struct EpisodeGeneratorConfig {
  int numEpisodes = 100;
  int seed = 3; // this is 3 for legacy reason
  // int numStageVariations = 12; // see selectedReplicaCadBakedStages
  int minStageNumber = 0;
  int maxStageNumber = 12;
  int numObjectVariations = 6; // see selectedYCBObjects
  int minNontargetObjects = 27;
  int maxNontargetObjects = 32;
  bool useFixedRobotStartPos = true;
  bool useFixedRobotStartYaw = false;
  bool useFixedRobotJointStartPositions = true;
  ESP_SMART_POINTERS(EpisodeGeneratorConfig);
};

EpisodeSet generateBenchmarkEpisodeSet(const EpisodeGeneratorConfig& config, 
  const BpsSceneMapping& sceneMapping, const serialize::Collection& collection);

}  // namespace batched_sim
}  // namespace esp

#endif
