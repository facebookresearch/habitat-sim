/** Copyright (c) Meta Platforms, Inc. and its affiliates.
 *  This source code is licensed under the MIT license found in the
 *  LICENSE file in the root directory of this source tree.
 */

#include "esp/nav/GreedyFollower.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"

namespace Mn = Magnum;
using Mn::EigenIntegration::cast;

namespace esp {
namespace nav {

GreedyGeodesicFollowerImpl::GreedyGeodesicFollowerImpl(
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

float GreedyGeodesicFollowerImpl::geoDist(const Mn::Vector3& start,
                                          const Mn::Vector3& end) {
  geoDistPath_.requestedStart = cast<vec3f>(start);
  geoDistPath_.requestedEnd = cast<vec3f>(end);
  pathfinder_->findPath(geoDistPath_);
  return geoDistPath_.geodesicDistance;
}

GreedyGeodesicFollowerImpl::TryStepResult GreedyGeodesicFollowerImpl::tryStep(
    const scene::SceneNode& node,
    const Mn::Vector3& end) {
  tryStepDummyNode_.MagnumObject::setTransformation(
      node.MagnumObject::transformation());

  const bool didCollide = moveForward_(&tryStepDummyNode_);
  const Mn::Vector3 newPose = tryStepDummyNode_.MagnumObject::translation();

  const float geoDistAfter = geoDist(newPose, end);
  const float distToObsAfter = pathfinder_->distanceToClosestObstacle(
      cast<vec3f>(newPose), 1.1 * closeToObsThreshold_);

  return {geoDistAfter, distToObsAfter, didCollide};
}

float GreedyGeodesicFollowerImpl::computeReward(const scene::SceneNode& node,
                                                const ShortestPath& path,
                                                const size_t primLen) {
  const auto tryStepRes = tryStep(node, Mn::Vector3{path.requestedEnd});

  // Try to minimize geodesic distance to target
  // Divide by forwardAmount_ to make the reward structure independent of step
  // size
  return (path.geodesicDistance - tryStepRes.postGeodesicDistance) /
             forwardAmount_ +
         (
             // Prefer shortest primitives
             -0.0125f * primLen
             // Avoid collisions
             - (tryStepRes.didCollide ? collisionCost_ : 0.0f)
             // Avoid being close to an obstacle
             - (tryStepRes.postDistanceToClosestObstacle < closeToObsThreshold_
                    ? 0.05f
                    : 0.0f));
}

std::vector<GreedyGeodesicFollowerImpl::CODES>
GreedyGeodesicFollowerImpl::nextBestPrimAlong(const core::RigidState& state,
                                              const ShortestPath& path) {
  if (path.geodesicDistance == std::numeric_limits<float>::infinity()) {
    return {CODES::ERROR};
  }

  if (path.geodesicDistance < goalDist_) {
    return {CODES::STOP};
  }

  // Intialize bestReward to the minimum acceptable reward -- we are just
  // constantly colliding
  float bestReward = -collisionCost_;
  std::vector<CODES> bestPrim, leftPrim, rightPrim;

  leftDummyNode_.setTranslation(state.translation);
  leftDummyNode_.setRotation(state.rotation);

  rightDummyNode_.setTranslation(state.translation);
  rightDummyNode_.setRotation(state.rotation);

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

bool GreedyGeodesicFollowerImpl::isThrashing() {
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

GreedyGeodesicFollowerImpl::CODES GreedyGeodesicFollowerImpl::nextActionAlong(
    const core::RigidState& start,
    const Mn::Vector3& end) {
  ShortestPath path;
  path.requestedStart = cast<vec3f>(start.translation);
  path.requestedEnd = cast<vec3f>(end);
  pathfinder_->findPath(path);

  CODES nextAction;
  if (fixThrashing_ && thrashingActions_.size() > 0) {
    nextAction = thrashingActions_.back();
    thrashingActions_.pop_back();
  } else {
    const auto nextActions = nextBestPrimAlong(start, path);
    if (nextActions.empty()) {
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

std::vector<GreedyGeodesicFollowerImpl::CODES>
GreedyGeodesicFollowerImpl::findPath(const core::RigidState& start,
                                     const Mn::Vector3& end) {
  constexpr int maxActions = 5e3;
  findPathDummyNode_.setTranslation(Mn::Vector3{start.translation});
  findPathDummyNode_.setRotation(Mn::Quaternion{start.rotation});

  do {
    core::RigidState state{findPathDummyNode_.rotation(),
                           findPathDummyNode_.MagnumObject::translation()};
    ShortestPath path;
    path.requestedStart = cast<vec3f>(state.translation);
    path.requestedEnd = cast<vec3f>(end);
    pathfinder_->findPath(path);
    const auto nextPrim = nextBestPrimAlong(state, path);
    if (nextPrim.empty()) {
      actions_.emplace_back(CODES::ERROR);
    } else {
      for (const auto nextAction : nextPrim) {
        switch (nextAction) {
          case CODES::FORWARD:
            moveForward_(&findPathDummyNode_);
            break;

          case CODES::RIGHT:
            turnRight_(&findPathDummyNode_);
            break;

          case CODES::LEFT:
            turnLeft_(&findPathDummyNode_);
            break;

          default:
            break;
        }

        actions_.emplace_back(nextAction);
      }
    }

  } while (actions_.back() != CODES::STOP && actions_.back() != CODES::ERROR &&
           actions_.size() < maxActions);

  if (actions_.back() == CODES::ERROR)
    return {};

  if (actions_.size() == maxActions)
    return {};

  return actions_;
}

GreedyGeodesicFollowerImpl::CODES GreedyGeodesicFollowerImpl::nextActionAlong(
    const Mn::Quaternion& currentRot,
    const Mn::Vector3& currentPos,
    const Mn::Vector3& end) {
  return nextActionAlong({currentRot, currentPos}, end);
}

std::vector<GreedyGeodesicFollowerImpl::CODES>
GreedyGeodesicFollowerImpl::findPath(const Mn::Quaternion& currentRot,
                                     const Mn::Vector3& currentPos,
                                     const Mn::Vector3& end) {
  return findPath({currentRot, currentPos}, end);
}

void GreedyGeodesicFollowerImpl::reset() {
  actions_.clear();
  thrashingActions_.clear();
}

}  // namespace nav
}  // namespace esp
