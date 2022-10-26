// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_NAV_GREEDYFOLLOWER_H_
#define ESP_NAV_GREEDYFOLLOWER_H_

#include "esp/core/Esp.h"
#include "esp/core/RigidState.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace nav {

/**
 * @brief Generates actions to take to reach a goal
 *
 * Choose actions by running local planning over a set of motion primitives.
 * The primitives are all sequences of action in the form `[left]*n + [forward]`
 * or `[right]*n + [forward]` for `0 <= n < turning 180 degrees`
 *
 * Primitives are selected by choosing the one that best maximizes a reward
 * function that selects the primitive that most quickly makes progress towards
 * the goal while preferring shorter primtives and avoid obstacles.
 *
 * Once a primitive is selected, the first action in that primitives is selected
 * as the next action to take and this process is repeated
 */
class GreedyGeodesicFollowerImpl {
 public:
  /**
   * @brief Ouputs from the greedy follower.
   *
   * Used to specify which action to take next
   * or that an error occurred
   */
  enum class CODES : int {
    ERROR = -2,
    STOP = -1,
    FORWARD = 0,
    LEFT = 1,
    RIGHT = 2
  };

  /**
   * @brief Helper typedef for function pointer to a function that manipulates a
   * scene node.
   *
   * These functions are used to get access to the python functions which
   * implement the control functions
   */
  typedef std::function<bool(scene::SceneNode*)> MoveFn;

  /**
   * @brief Constructor
   *
   * @param[in] pathfinder Instance of the pathfinder used for calculating the
   *                       geodesic shortest path
   * @param[in] moveForward Function that implements "move_forward" on a
   *                        SceneNode
   * @param[in] turnLeft Function that implements "turn_left" on a SceneNode
   * @param[in] turnRight Function that implements "turn_right" on a SceneNode
   * @param[in] goalDist How close the agent needs to get to the goal before
   *                     calling stop
   * @param[in] forwardAmount The amount "move_forward" moves the agent
   * @param[in] turnAmount The amount "turn_left"/"turn_right" turns the agent
   *                       in radians
   * @param[in] fixThrashing Whether or not to fix thrashing
   * @param[in] thrashingThreshold The length of left, right, left, right
   *                                actions needed to be considered thrashing
   */
  GreedyGeodesicFollowerImpl(PathFinder::ptr& pathfinder,
                             MoveFn& moveForward,
                             MoveFn& turnLeft,
                             MoveFn& turnRight,
                             double goalDist,
                             double forwardAmount,
                             double turnAmount,
                             bool fixThrashing = true,
                             int thrashingThreshold = 16);

  /**
   * @brief Calculates the next action to follow the path
   *
   * @param[in] currentRot The current rotation
   * @param[in] currentPos The current position
   * @param[in] end The end location of the path
   */
  CODES nextActionAlong(const Magnum::Quaternion& currentRot,
                        const Magnum::Vector3& currentPos,
                        const Magnum::Vector3& end);

  /**
   * @brief Finds the full path from the current agent state to the end location
   *
   * @warning Do not use this method if there is actuation noise.  Instead, use
   * @ref nextActionAlong to calculate the next action to take, actually take
   * it, then call that method again.
   *
   * @param[in] startRot The starting rotation
   * @param[in] startPos The starting position
   * @param[in] end The end location of the path
   */
  std::vector<CODES> findPath(const Magnum::Quaternion& startRot,
                              const Magnum::Vector3& startPos,
                              const Magnum::Vector3& end);

  CODES nextActionAlong(const core::RigidState& start,
                        const Magnum::Vector3& end);
  std::vector<CODES> findPath(const core::RigidState& start,
                              const Magnum::Vector3& end);

  /**
   * @brief Reset the planner.
   *
   * Should be called whenever a different goal is chosen or start state
   * differs by more than action from the last start state
   */
  void reset();

 private:
  PathFinder::ptr pathfinder_;
  MoveFn moveForward_, turnLeft_, turnRight_;
  const double forwardAmount_, goalDist_, turnAmount_;
  const bool fixThrashing_;
  const int thrashingThreshold_;
  const float closeToObsThreshold_ = 0.2f;
  const float collisionCost_ = 0.25f;

  std::vector<CODES> actions_;
  std::vector<CODES> thrashingActions_;

  scene::SceneGraph dummyScene_;
  scene::SceneNode findPathDummyNode_{dummyScene_.getRootNode()},
      leftDummyNode_{dummyScene_.getRootNode()},
      rightDummyNode_{dummyScene_.getRootNode()},
      tryStepDummyNode_{dummyScene_.getRootNode()};

  ShortestPath geoDistPath_;
  float geoDist(const Magnum::Vector3& start, const Magnum::Vector3& end);

  struct TryStepResult {
    float postGeodesicDistance, postDistanceToClosestObstacle;
    bool didCollide;
  };

  TryStepResult tryStep(const scene::SceneNode& node,
                        const Magnum::Vector3& end);

  float computeReward(const scene::SceneNode& node,
                      const nav::ShortestPath& path,
                      size_t primLen);

  bool isThrashing();

  std::vector<nav::GreedyGeodesicFollowerImpl::CODES> nextBestPrimAlong(
      const core::RigidState& state,
      const nav::ShortestPath& path);

  ESP_SMART_POINTERS(GreedyGeodesicFollowerImpl)
};

}  // namespace nav
}  // namespace esp

#endif  // ESP_NAV_GREEDYFOLLOWER_H_
