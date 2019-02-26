// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/agent/Agent.h"
#include "esp/core/esp.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneGraph.h"

#include <unordered_map>

namespace std {
template <>
struct hash<esp::vec4ul> {
  size_t operator()(const esp::vec4ul& k) const {
    size_t seed = 0;
    for (int i = 0; i < 4; ++i) {
      seed ^= k[i] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    return seed;
  }
};
}  // namespace std

namespace esp {
namespace nav {
// forward declare

namespace impl {

struct ActionSpaceGraph {
  ActionSpaceGraph(PathFinder::ptr pf,
                   const agent::AgentConfiguration& config,
                   const scene::ObjectControls& controls,
                   const double closeToObstacleCostMultiplier = 5.0,
                   const double paddingRadius = 0.0);

  ~ActionSpaceGraph();

  float edgeWeight(const uint64_t& fromKey, const uint64_t& toKey);
  float computeTileWeight(const uint64_t& key);

  float heuristic(const uint64_t& key);

  uint64_t toKey(const vec3f& pos, const quatf& rotation) const;

  std::tuple<vec3f, quatf> toWorld(const uint64_t& key) const;

  std::unordered_map<std::string, uint64_t> tryActionsFrom(const uint64_t& key);
  std::vector<uint64_t> edgesFrom(const uint64_t& key);

  void calibrateRotations(const quatf& initRotation);
  int getClosestCalibratedRotation(const quatf& q) const;

  std::string getActionBetween(const uint64_t& fromKey, const uint64_t& toKey);

  void takeAction(const std::string& act, scene::SceneNode& node);

  void goalNodeKey(const uint64_t& newKey);

  PathFinder::ptr finder_;
  agent::AgentConfiguration config_;
  scene::ObjectControls controls_;

  double closeToObstacleCostMultiplier_, paddingRadius_;
  std::unordered_map<uint64_t, float> tileCostCache_;
  double goalExpansionAmount_ = 0.0;

  std::vector<quatf> calibratedRotations_;

  double stepSize_, turnSize_, cellsPerMeter_, cellSize_;

 private:
  scene::SceneGraph dummyGraph_;
  scene::SceneNode dummy_;

  std::string forwardAction, lookLeftAction, lookRightAction;

  // TODO Change these to turn actions instead of look actions
  const std::string forwardActionName = "moveForward",
                    lookLeftActionName = "lookLeft",
                    lookRightActionName = "lookRight";

  uint64_t goalNodeKey_;
  std::unordered_map<uint64_t, float> heuristicCache_;

  ESP_SMART_POINTERS(ActionSpaceGraph)
};
}  // namespace impl
}  // namespace nav
}  // namespace esp
