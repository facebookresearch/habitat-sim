#pragma once

#include "esp/core/esp.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace nav {

class GreedyGeodesicFollowerImpl {
 public:
  enum class CODES { ERROR = -2, STOP = -1, FORWARD = 0, LEFT = 1, RIGHT = 2 };

  typedef std::function<void(scene::SceneNode*)> MoveFn;
  typedef std::tuple<vec3f, quatf> State;

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
        goalDist_{goalDist},
        forwardAmount_{forwardAmount},
        turnAmount_{turnAmount} {};

  int nextActionAlong(const State& start, const Eigen::Ref<const vec3f> end);

  inline int nextActionAlong(const Eigen::Ref<const vec3f> startPos,
                             const Eigen::Ref<const vec4f> startRot,
                             const Eigen::Ref<const vec3f> end) {
    quatf rot = Eigen::Map<const quatf>(startRot.data());
    return nextActionAlong(std::make_tuple(startPos, rot), end);
  }

  std::vector<int> findPath(const State& start,
                            const Eigen::Ref<const vec3f> end);

  inline std::vector<int> findPath(const Eigen::Ref<const vec3f> startPos,
                                   const Eigen::Ref<const vec4f> startRot,
                                   const Eigen::Ref<const vec3f> end) {
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
