#include "esp/nav/GreedyFollower.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

#include "Sophus/sophus/so3.hpp"
#include "esp/geo/geo.h"

namespace Mn = Magnum;
using Mn::EigenIntegration::cast;

namespace esp {

nav::GreedyGeodesicFollowerImpl::GreedyGeodesicFollowerImpl(
    PathFinder::ptr& pathfinder,
    MoveFn& moveForward,
    MoveFn& turnLeft,
    MoveFn& turnRight,
    double goalDist,
    double forwardAmount,
    double turnAmount,
    bool fixThrashing,
    int thrashingThreshold)
    : pathfinder_{pathfinder},
      moveForward_{moveForward},
      turnLeft_{turnLeft},
      turnRight_{turnRight},
      forwardAmount_{forwardAmount},
      goalDist_{goalDist},
      turnAmount_{turnAmount},
      fixThrashing_{fixThrashing},
      thrashingThreshold_{thrashingThreshold} {};

float nav::GreedyGeodesicFollowerImpl::geoDist(const vec3f& start,
                                               const vec3f& end) {
  ShortestPath path;
  path.requestedStart = start;
  path.requestedEnd = end;
  pathfinder_->findPath(path);
  return path.geodesicDistance;
}

std::tuple<float, float, float> nav::GreedyGeodesicFollowerImpl::tryStep(
    const scene::SceneNode& node,
    const vec3f& end) {
  tryStepDummyNode_.MagnumObject::setTransformation(
      node.MagnumObject::transformation());

  const float didCollide = moveForward_(&tryStepDummyNode_) ? 1.0f : 0.0f;
  const vec3f newPose =
      cast<vec3f>(tryStepDummyNode_.MagnumObject::translation());

  return std::make_tuple(geoDist(newPose, end), didCollide,
                         pathfinder_->distanceToClosestObstacle(
                             newPose, 1.1 * closeToObsThreshold_));
}

float nav::GreedyGeodesicFollowerImpl::computeReward(
    const scene::SceneNode& node,
    const nav::ShortestPath& path,
    const int primLen) {
  float newGoeDist, didCollide, distToObstacle;
  std::tie(newGoeDist, didCollide, distToObstacle) =
      tryStep(node, path.requestedEnd);

  // Try to minimize geodesic distance to target
  return (path.geodesicDistance - newGoeDist) +
         // Weight these costs by the forwardAmount as (path.geodesicDistance -
         // newGoeDist) is a function of that also
         forwardAmount_ *
             (
                 // Prefer shortest primitives
                 -0.025f * primLen
                 // Avoid collisions
                 - 0.25f * didCollide
                 // Avoid being close to an obstacle
                 - 0.05f * static_cast<float>(distToObstacle <
                                              closeToObsThreshold_));
  ;
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::nextBestPrimAlong(
    const nav::GreedyGeodesicFollowerImpl::State& state,
    const nav::ShortestPath& path) {
  if (path.geodesicDistance == std::numeric_limits<float>::infinity()) {
    return {CODES::ERROR};
  }

  if (path.geodesicDistance < goalDist_) {
    return {CODES::STOP};
  }

  // Intialize bestReward to the minumum acceptable reward
  float bestReward = -0.1f * forwardAmount_;
  std::vector<CODES> bestPrim, leftPrim, rightPrim;

  leftDummyNode_.setTranslation(Magnum::Vector3{std::get<0>(state)});
  leftDummyNode_.setRotation(Magnum::Quaternion{std::get<1>(state)});

  rightDummyNode_.setTranslation(Magnum::Vector3{std::get<0>(state)});
  rightDummyNode_.setRotation(Magnum::Quaternion{std::get<1>(state)});

  // Plan over all primitives of the form [LEFT] * n + [FORWARD]
  // or [RIGHT] * n + [FORWARD]
  for (float angle = 0; angle < M_PI; angle += turnAmount_) {
    {
      const float reward = computeReward(leftDummyNode_, path, leftPrim.size());
      if (reward > bestReward) {
        bestReward = reward;
        bestPrim = leftPrim;
        bestPrim.emplace_back(CODES::FORWARD);
      }
    }

    {
      const float reward =
          computeReward(rightDummyNode_, path, rightPrim.size());
      if (reward > bestReward) {
        bestReward = reward;
        bestPrim = rightPrim;
        bestPrim.emplace_back(CODES::FORWARD);
      }
    }

    if (bestReward > 0.95 * forwardAmount_)
      break;

    leftPrim.emplace_back(CODES::LEFT);
    turnLeft_(&leftDummyNode_);

    rightPrim.emplace_back(CODES::RIGHT);
    turnRight_(&rightDummyNode_);
  }

  return bestPrim;
}

bool nav::GreedyGeodesicFollowerImpl::isThrashing() {
  if (actions_.size() < thrashingThreshold_)
    return false;

  bool thrashing =
      actions_.back() == CODES::LEFT || actions_.back() == CODES::RIGHT;
  CODES lastAct = actions_.back();
  for (int i = 2; (i < (thrashingThreshold_ + 1)) && thrashing; ++i) {
    thrashing = (actions_[actions_.size() - i] == CODES::RIGHT &&
                 lastAct == CODES::LEFT) ||
                (actions_[actions_.size() - i] == CODES::LEFT &&
                 lastAct == CODES::RIGHT);
    lastAct = actions_.back();
  }

  return thrashing;
}

nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::nextActionAlong(const State& start,
                                                 const vec3f& end) {
  nav::ShortestPath path;
  path.requestedStart = std::get<0>(start);
  path.requestedEnd = end;
  pathfinder_->findPath(path);

  const auto actions = nextBestPrimAlong(start, path);

  CODES nextAction;
  if (fixThrashing_ && thrashingActions_.size() > 0) {
    nextAction = thrashingActions_.back();
    thrashingActions_.pop_back();
  } else {
    const auto nextActions = nextBestPrimAlong(start, path);
    if (nextActions.size() == 0) {
      nextAction = CODES::ERROR;
    } else if (fixThrashing_ && isThrashing()) {
      thrashingActions_ = {nextActions.rbegin(), nextActions.rend()};
      nextAction = thrashingActions_.back();
      thrashingActions_.pop_back();
    } else {
      nextAction = nextActions[0];
    }
  }

  actions_.push_back(nextAction);

  return actions_.back();
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::findPath(const State& start,
                                          const vec3f& end) {
  constexpr int maxActions = 5e3;
  dummyNode_.setTranslation(Mn::Vector3{std::get<0>(start)});
  dummyNode_.setRotation(Mn::Quaternion{std::get<1>(start)});

  do {
    State state =
        std::make_tuple(cast<vec3f>(dummyNode_.MagnumObject::translation()),
                        quatf{dummyNode_.rotation()});
    CODES nextAction = nextActionAlong(state, end);

    switch (nextAction) {
      case CODES::FORWARD:
        moveForward_(&dummyNode_);
        break;

      case CODES::RIGHT:
        turnRight_(&dummyNode_);
        break;

      case CODES::LEFT:
        turnLeft_(&dummyNode_);
        break;

      default:
        break;
    }

  } while (actions_.back() != CODES::STOP && actions_.back() != CODES::ERROR &&
           actions_.size() < maxActions);

  if (actions_.back() == CODES::ERROR)
    return {};

  if (actions_.size() == maxActions)
    return {};

  return actions_;
}

nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::nextActionAlong(const vec3f& currentPos,
                                                 const vec4f& currentRot,
                                                 const vec3f& end) {
  quatf rot = Eigen::Map<const quatf>(currentRot.data());
  return nextActionAlong(std::make_tuple(currentPos, rot), end);
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::findPath(const vec3f& currentPos,
                                          const vec4f& currentRot,
                                          const vec3f& end) {
  quatf rot = Eigen::Map<const quatf>(currentRot.data());
  return findPath(std::make_tuple(currentPos, rot), end);
}

void nav::GreedyGeodesicFollowerImpl::reset() {
  actions_.clear();
  thrashingActions_.clear();
}

}  // namespace esp
