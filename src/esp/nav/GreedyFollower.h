#pragma once

#include "esp/core/esp.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace nav {

class GreedyGeodesicFollowerImpl {
 public:
  /**
   * Ouputs from the greedy follower.  Used to specify which action to take next
   *or that an error occured
   **/
  enum class CODES { ERROR = -2, STOP = -1, FORWARD = 0, LEFT = 1, RIGHT = 2 };

  /**
   * Helper typedef for function pointer to a function that manipulates a scene
   *node These functions are used to get access to the python functions which
   *implement the control functions
   **/
  typedef std::function<void(scene::SceneNode*)> MoveFn;

  /**
   * Helper typedef for a sixdof pos
   **/
  typedef std::tuple<vec3f, quatf> State;

  /**
   * Implements a follower that greedily fits actions to follow the geodesic
   *shortest path
   *
   * Params
   * @param[in] pathfinder Instance of the pathfinder used for calculating the
   *geodesic shortest path
   * @param[in] moveForward Function that implements "move_forward" on a
   *SceneNode
   * @param[in] turnLeft Function that implements "turn_left" on a SceneNode
   * @param[in] turnRight Function that implements "turn_right" on a SceneNode
   * @param[in] goalDist How close the agent needs to get to the goal before
   *calling stop
   * @param[in] forwardAmount The amount "move_forward" moves the agent
   * @param[in] turnAmount The amount "turn_left"/"turn_right" turns the agent
   *in radians
   **/
  GreedyGeodesicFollowerImpl(PathFinder::ptr& pathfinder,
                             MoveFn& moveForward,
                             MoveFn& turnLeft,
                             MoveFn& turnRight,
                             double goalDist,
                             double forwardAmount,
                             double turnAmount)
      : pathfinder_{pathfinder},
        moveForward_{moveForward},
        turnLeft_{turnLeft},
        turnRight_{turnRight},
        forwardAmount_{forwardAmount},
        goalDist_{goalDist},
        turnAmount_{turnAmount} {};

  CODES nextActionAlong(const State& start, const vec3f& end);

  /**
   * Calculates the next action to follow the path
   *
   * Params
   * @param[in] currentPos The current position
   * @param[in] currentRot The current rotation
   * @param[in] end The end location of the path
   **/
  inline CODES nextActionAlong(const vec3f& currentPos,
                               const vec4f& currentRot,
                               const vec3f& end) {
    quatf rot = Eigen::Map<const quatf>(currentRot.data());
    return nextActionAlong(std::make_tuple(currentPos, rot), end);
  }

  std::vector<CODES> findPath(const State& start, const vec3f& end);

  /**
   * Finds the full path from the current agent state to the end location
   *
   * Params
   * @param[in] startPos The starting position
   * @param[in] startRot The starting rotation
   * @param[in] end The end location of the path
   **/
  inline std::vector<CODES> findPath(const vec3f& startPos,
                                     const vec4f& startRot,
                                     const vec3f& end) {
    quatf rot = Eigen::Map<const quatf>(startRot.data());
    return findPath(std::make_tuple(startPos, rot), end);
  }

 private:
  PathFinder::ptr pathfinder_;
  MoveFn moveForward_, turnLeft_, turnRight_;
  const double forwardAmount_, goalDist_, turnAmount_;

  scene::SceneGraph dummyScene_;
  scene::SceneNode dummyNode_{dummyScene_.getRootNode()};

  CODES calcStepAlong(const State& start, const ShortestPath& path);

  inline float geoDist(const vec3f& start, const vec3f& end) {
    ShortestPath path;
    path.requestedStart = start;
    path.requestedEnd = end;
    pathfinder_->findPath(path);
    return path.geodesicDistance;
  }

  CODES checkForward(const State& state);

  ESP_SMART_POINTERS(GreedyGeodesicFollowerImpl)
};

}  // namespace nav
}  // namespace esp
