// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/nav/ActionSpacePath.h"
#include "esp/geo/geo.h"

#include <sophus/so3.hpp>

namespace esp {
namespace nav {

/*
 * Due to the cell size being much smaller than the step size, the exact goal
 * cell tends to be unreachable, or requires an absolutely ridiculous looking
 * path.  The way to fix this is to either expand the goal location or expand
 * the starting location to be all cells within half (for instance) a step of
 * the actual goal or starting location.  If EXPAND_GOALS is 1, the goal
 * locations will be expanded, while if EXPAND_GOALS is 0, the starting location
 * will be expanded. EXPAND_GOALS tends to give results that more align with
 * your expectations than EXPAND_START, but EXPAND_GOALS is slower/uses more
 * memory, although this tends to be negligible
 */
#define EXPAND_GOALS 1

namespace {
// Fraction of step size to expand start or goal locations by
constexpr double expandFactor = 0.5;

// Flip directions as we are going to search from end to start!
const Sophus::SO3f T_flip_rotation_dir = Sophus::SO3f::exp(M_PI * geo::ESP_UP);
}  // namespace

void ActionSpacePathFinder::buildSearchState(
    const MultiGoalActionSpaceShortestPath& _path) {
  // The exit or start node set needs to be expanded as the exact path between
  // two points won't look at all like we want it to :)

  std::vector<uint64_t> endNodes;
  for (auto& rqEnd : _path.requestedEnds) {
    Eigen::Map<quatf> endRotation(rqEnd->rotation_.data());
    const uint64_t endNode = graph_.toKey(
        rqEnd->pos_,
        (T_flip_rotation_dir * Sophus::SO3f(endRotation)).unit_quaternion());

    endNodes.emplace_back(endNode);
  }

#if EXPAND_GOALS
  const double r = graph_.stepSize_ * expandFactor;

  // Include all nodes within half a step as the start node
  {
    std::vector<uint64_t> expandedEndNodes;
    for (const auto& ed : endNodes) {
      for (double dz = -r; dz <= r; dz += graph_.cellSize_) {
        for (double dx = -r; dx <= r; dx += graph_.cellSize_) {
          if (dx * dx + dz * dz > r * r) {
            continue;
          }

          vec3f startPos;
          quatf rotation;
          std::tie(startPos, rotation) = graph_.toWorld(ed);

          startPos[0] += dx;
          startPos[2] += dz;

          expandedEndNodes.emplace_back(graph_.toKey(startPos, rotation));
        }
      }
    }
    endNodes = expandedEndNodes;
  }

  graph_.goalExpansionAmount_ = expandFactor;
#endif

  if (prevEndNodes_ != endNodes) {
    // Create the search state.  The search state takes the starting nodes,
    // which are the end nodes since we are going to search backwards.
    searchState_ = AStar::CreateSearchState(&graph_, endNodes);
    prevEndNodes_.assign(endNodes.begin(), endNodes.end());
  }
}

bool ActionSpacePathFinder::checkIfHasPath(
    const MultiGoalActionSpaceShortestPath& _path) {
  MultiGoalShortestPath path2D_;
  path2D_.requestedStart = _path.requestedStart->pos_;
  for (auto& rqEnd : _path.requestedEnds) {
    path2D_.requestedEnds.emplace_back(rqEnd->pos_);
  }

  if (!finder_->findPath(path2D_)) {
    LOG(ERROR) << "Could not find path between start and goal(s)";
    return false;
  }

  return true;
}

bool ActionSpacePathFinder::findNextActionInternal(const uint64_t& startNode,
                                                   std::string& act) {
  std::unordered_set<uint64_t> exitNodes = {startNode};

#if !EXPAND_GOALS
  const double r = graph_.stepSize_ * expandFactor;

  // Include all nodes within half a step as the start node
  for (double dz = -r; dz <= r; dz += graph_.cellSize_) {
    for (double dx = -r; dx <= r; dx += graph_.cellSize_) {
      if (dx * dx + dz * dz > r * r) {
        continue;
      }

      vec3f startPos;
      quatf rotation;
      std::tie(startPos, rotation) = graph_.toWorld(startNode);

      startPos[0] += dx;
      startPos[2] += dz;

      exitNodes.emplace(graph_.toKey(startPos, rotation));
    }
  }
#endif
  // Set the goalNode ID for heuristic computation
  graph_.goalNodeKey(startNode);
  std::vector<AStar::Node::ptr> astarPath;

  constexpr float hweight = 1.05;
  if (!astar_.compute(searchState_, exitNodes, astarPath, hweight)) {
    return false;
  }

  if (astarPath.size() > 1) {
    act = graph_.getActionBetween(astarPath.back()->key,
                                  astarPath[astarPath.size() - 2]->key);
  } else {
    act = "goalFound";
  }

  return true;
}

bool ActionSpacePathFinder::findPath(ActionSpaceShortestPath& _path) {
  MultiGoalActionSpaceShortestPath multiPath;
  multiPath.requestedStart = _path.requestedStart;
  multiPath.requestedEnds.assign({_path.requestedEnd});

  bool status = findPath(multiPath);

  _path.geodesicDistance = multiPath.geodesicDistance;
  _path.points.assign(multiPath.points.begin(), multiPath.points.end());
  _path.actions.assign(multiPath.actions.begin(), multiPath.actions.end());
  _path.rotations.assign(multiPath.rotations.begin(),
                         multiPath.rotations.end());

  return status;
}

bool ActionSpacePathFinder::findPath(MultiGoalActionSpaceShortestPath& _path) {
  _path.actions.clear();
  _path.points.clear();
  _path.rotations.clear();
  _path.geodesicDistance = std::numeric_limits<float>::infinity();

  if (!checkIfHasPath(_path))
    return false;

  buildSearchState(_path);

  dummy_.setTranslation(_path.requestedStart->pos_);
  dummy_.setRotation(
      Eigen::Map<const quatf>(_path.requestedStart->rotation_.data()));

  _path.points.emplace_back(dummy_.getAbsolutePosition());
  _path.rotations.emplace_back(dummy_.getRotation().coeffs());

  while (true) {
    const uint64_t startNode =
        graph_.toKey(dummy_.getAbsolutePosition(),
                     (T_flip_rotation_dir * Sophus::SO3f(dummy_.getRotation()))
                         .unit_quaternion());

    std::string nextAct;
    if (!findNextActionInternal(startNode, nextAct)) {
      return false;
    }

    if (nextAct == "goalFound")
      break;

    graph_.takeAction(nextAct, dummy_);

    _path.actions.emplace_back(nextAct);
    _path.points.emplace_back(dummy_.getAbsolutePosition());
    _path.rotations.emplace_back(dummy_.getRotation().coeffs());
  }

  _path.geodesicDistance = 0.0;
  for (int i = 1; i < _path.points.size(); ++i) {
    _path.geodesicDistance += (_path.points[i - 1] - _path.points[i]).norm();
  }

  return true;
}

bool ActionSpacePathFinder::findNextActionAlongPath(
    ActionSpaceShortestPath& _path) {
  MultiGoalActionSpaceShortestPath multiPath;
  multiPath.requestedStart = _path.requestedStart;
  multiPath.requestedEnds.assign({_path.requestedEnd});

  bool status = findNextActionAlongPath(multiPath);

  _path.geodesicDistance = multiPath.geodesicDistance;
  _path.points.assign(multiPath.points.begin(), multiPath.points.end());
  _path.actions.assign(multiPath.actions.begin(), multiPath.actions.end());
  _path.rotations.assign(multiPath.rotations.begin(),
                         multiPath.rotations.end());

  return status;
}

bool ActionSpacePathFinder::findNextActionAlongPath(
    MultiGoalActionSpaceShortestPath& _path) {
  _path.actions.clear();
  _path.points.clear();
  _path.rotations.clear();
  _path.geodesicDistance = std::numeric_limits<float>::infinity();

  if (!checkIfHasPath(_path))
    return false;

  buildSearchState(_path);

  // Flip directions as we are going to search from end to start!
  const Sophus::SO3f T_flip_rotation_dir =
      Sophus::SO3f::exp(M_PI * geo::ESP_UP);

  _path.geodesicDistance = 0.0;
  dummy_.setTranslation(_path.requestedStart->pos_);
  dummy_.setRotation(
      Eigen::Map<const quatf>(_path.requestedStart->rotation_.data()));

  const uint64_t startNode =
      graph_.toKey(_path.requestedStart->pos_,
                   (T_flip_rotation_dir * Sophus::SO3f(dummy_.getRotation()))
                       .unit_quaternion());

  std::string nextAct;
  if (!findNextActionInternal(startNode, nextAct)) {
    return false;
  }

  _path.actions.emplace_back(nextAct);

  return true;
}
void ActionSpacePathFinder::paddingRadius(const double paddingRadius) {
  ASSERT(paddingRadius >= 0.0);
  if (graph_.paddingRadius_ != paddingRadius) {
    graph_.paddingRadius_ = paddingRadius;
    prevEndNodes_.clear();
    graph_.tileCostCache_.clear();
  }
};
void ActionSpacePathFinder::closeToObstacleCostMultiplier(
    const double newCost) {
  ASSERT(newCost >= 1.0);
  if (graph_.closeToObstacleCostMultiplier_ != newCost) {
    graph_.closeToObstacleCostMultiplier_ = newCost;
    prevEndNodes_.clear();
    graph_.tileCostCache_.clear();
  }
};

#undef EXPAND_GOALS

}  // namespace nav
}  // namespace esp
