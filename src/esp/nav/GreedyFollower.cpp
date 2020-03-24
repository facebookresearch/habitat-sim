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
  geoDistPath_.requestedStart = start;
  geoDistPath_.requestedEnd = end;
  pathfinder_->findPath(geoDistPath_);
  return geoDistPath_.geodesicDistance;
}

nav::GreedyGeodesicFollowerImpl::TryStepResult
nav::GreedyGeodesicFollowerImpl::tryStep(const scene::SceneNode& node,
                                         const vec3f& end) {
  tryStepDummyNode_.MagnumObject::setTransformation(
      node.MagnumObject::transformation());

  const bool didCollide = moveForward_(&tryStepDummyNode_);
  const vec3f newPose =
      cast<vec3f>(tryStepDummyNode_.MagnumObject::translation());

  const float geoDistAfter = geoDist(newPose, end);
  const float distToObsAfter = pathfinder_->distanceToClosestObstacle(
      newPose, 1.1 * closeToObsThreshold_);

  return {geoDistAfter, distToObsAfter, didCollide};
}

float nav::GreedyGeodesicFollowerImpl::computeReward(
    const scene::SceneNode& node,
    const nav::ShortestPath& path,
    const size_t primLen) {
  const auto tryStepRes = tryStep(node, path.requestedEnd);

  // Try to minimize geodesic distance to target
  // Divide by forwardAmount_ to make the reward structure independent of step
  // size
  return (path.geodesicDistance - tryStepRes.postGeodesicDistance) /
             forwardAmount_ +
         (
             // Prefer shortest primitives
             -0.0125f * primLen
             // Avoid collisions
             - (tryStepRes.didCollide ? 0.25f : 0.0f)
             // Avoid being close to an obstacle
             - (tryStepRes.postDistanceToClosestObstacle < closeToObsThreshold_
                    ? 0.05f
                    : 0.0f));
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::nextBestPrimAlong(
    const nav::GreedyGeodesicFollowerImpl::SixDofPose& state,
    const nav::ShortestPath& path) {
  if (path.geodesicDistance == std::numeric_limits<float>::infinity()) {
    return {CODES::ERROR};
  }

  if (path.geodesicDistance < goalDist_) {
    return {CODES::STOP};
  }

  constexpr float minumumReward = -0.1f;
  // Intialize bestReward to the minumum acceptable reward
  float bestReward = minumumReward;
  std::vector<CODES> bestPrim, leftPrim, rightPrim;

  leftDummyNode_.setTranslation(Mn::Vector3{state.translation});
  leftDummyNode_.setRotation(Mn::Quaternion{state.rotation});

  rightDummyNode_.setTranslation(Mn::Vector3{state.translation});
  rightDummyNode_.setRotation(Mn::Quaternion{state.rotation});

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

    // If reward is within 99% of max (1.0), call it good enough and exit
    constexpr float goodEnoughRewardThresh = 0.99f;
    if (bestReward > goodEnoughRewardThresh)
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

  CODES lastAct = actions_.back();

  bool thrashing = lastAct == CODES::LEFT || lastAct == CODES::RIGHT;
  for (int i = 2; (i < (thrashingThreshold_ + 1)) && thrashing; ++i) {
    thrashing = (actions_[actions_.size() - i] == CODES::RIGHT &&
                 lastAct == CODES::LEFT) ||
                (actions_[actions_.size() - i] == CODES::LEFT &&
                 lastAct == CODES::RIGHT);
    lastAct = actions_[actions_.size() - i];
  }

  return thrashing;
}

nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::nextActionAlong(const SixDofPose& start,
                                                 const vec3f& end) {
  nav::ShortestPath path;
  path.requestedStart = start.translation;
  path.requestedEnd = end;
  pathfinder_->findPath(path);

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
nav::GreedyGeodesicFollowerImpl::findPath(const SixDofPose& start,
                                          const vec3f& end) {
  constexpr int maxActions = 5e3;
  dummyNode_.setTranslation(Mn::Vector3{start.translation});
  dummyNode_.setRotation(Mn::Quaternion{start.rotation});

  do {
    SixDofPose state{quatf{dummyNode_.rotation()},
                     cast<vec3f>(dummyNode_.MagnumObject::translation())};
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
  return nextActionAlong({rot, currentPos}, end);
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::findPath(const vec3f& currentPos,
                                          const vec4f& currentRot,
                                          const vec3f& end) {
  quatf rot = Eigen::Map<const quatf>(currentRot.data());
  return findPath({rot, currentPos}, end);
}

void nav::GreedyGeodesicFollowerImpl::reset() {
  actions_.clear();
  thrashingActions_.clear();
}

}  // namespace esp
