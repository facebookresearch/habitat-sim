// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/agent/Agent.h"
#include "esp/core/esp.h"
#include "esp/nav/AStar.h"
#include "esp/nav/ActionSpaceGraph.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"

namespace esp {
namespace nav {

struct ActionSpacePathLocation {
  ActionSpacePathLocation(){};
  ActionSpacePathLocation(const vec3f& pos, const vec4f& rotation)
      : pos_{pos}, rotation_{rotation} {};
  vec3f pos_;
  vec4f rotation_;

  ESP_SMART_POINTERS(ActionSpacePathLocation);
};

struct ActionSpaceShortestPath {
  ActionSpacePathLocation::ptr requestedStart =
      ActionSpacePathLocation::create();
  ActionSpacePathLocation::ptr requestedEnd = ActionSpacePathLocation::create();

  std::vector<std::string> actions;
  std::vector<vec3f> points;
  std::vector<vec4f> rotations;

  float geodesicDistance;

  ESP_SMART_POINTERS(ActionSpaceShortestPath);
};

struct MultiGoalActionSpaceShortestPath {
  ActionSpacePathLocation::ptr requestedStart =
      ActionSpacePathLocation::create();
  std::vector<ActionSpacePathLocation::ptr> requestedEnds;

  std::vector<std::string> actions;
  std::vector<vec3f> points;
  std::vector<vec4f> rotations;

  float geodesicDistance;

  ESP_SMART_POINTERS(MultiGoalActionSpaceShortestPath);
};

class ActionSpacePathFinder {
 public:
  ActionSpacePathFinder(PathFinder::ptr& pf,
                        const agent::AgentConfiguration& config,
                        const scene::ObjectControls& controls,
                        const quatf& initRotation)
      : dummy_{dummyGraph_.getRootNode()},
        finder_{pf},
        graph_{pf, config, controls} {
    graph_.calibrateRotations(initRotation);
  };

  ActionSpacePathFinder(PathFinder::ptr& pf,
                        const agent::AgentConfiguration& config,
                        const scene::ObjectControls& controls,
                        const vec4f& initRotation)
      : ActionSpacePathFinder{pf, config, controls,
                              Eigen::Map<const quatf>(initRotation.data())} {};

  bool findNextActionAlongPath(ActionSpaceShortestPath& _path);
  bool findNextActionAlongPath(MultiGoalActionSpaceShortestPath& _path);

  bool findPath(ActionSpaceShortestPath& _path);
  bool findPath(MultiGoalActionSpaceShortestPath& _path);

  void paddingRadius(const double paddingRadius);
  inline double paddingRadius(void) const { return graph_.paddingRadius_; };

  void closeToObstacleCostMultiplier(const double newCost);
  inline double closeToObstacleCostMultiplier(void) const {
    return graph_.closeToObstacleCostMultiplier_;
  };

 private:
  scene::SceneGraph dummyGraph_;
  scene::SceneNode dummy_;

  PathFinder::ptr finder_;
  impl::ActionSpaceGraph graph_;

  AStar astar_;
  AStar::SearchState::ptr searchState_;

  std::vector<uint64_t> prevEndNodes_;

  void buildSearchState(const MultiGoalActionSpaceShortestPath& _path);
  bool findNextActionInternal(const uint64_t& startNode, std::string& act);
  bool checkIfHasPath(const MultiGoalActionSpaceShortestPath& _path);

  ESP_SMART_POINTERS(ActionSpacePathFinder)
};
}  // namespace nav
}  // namespace esp
