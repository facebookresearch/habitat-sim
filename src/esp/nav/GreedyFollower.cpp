#include "esp/nav/GreedyFollower.h"
#include "Sophus/sophus/so3.hpp"
#include "esp/geo/geo.h"

namespace esp {

// There are some cases were we can't perfectly align along the shortest path
// and the agent will get stuck, so we need to check that forward will actually
// move us forward, if it doesn't, we will check to see if either of the turn
// directions + forward will make progress, if neither do, return an error
nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::checkForward(const State& state) {
  dummyNode_.setTranslation(std::get<0>(state));
  dummyNode_.setRotation(std::get<1>(state));

  moveForward_(&dummyNode_);
  float dist_travelled =
      (std::get<0>(state) - dummyNode_.getAbsolutePosition()).norm();

  const float minTravel = 1e-1 * forwardAmount_;

  if (dist_travelled < minTravel) {
    dummyNode_.setTranslation(std::get<0>(state));
    dummyNode_.setRotation(std::get<1>(state));

    turnLeft_(&dummyNode_);
    moveForward_(&dummyNode_);
    dist_travelled =
        (std::get<0>(state) - dummyNode_.getAbsolutePosition()).norm();
    if (dist_travelled > minTravel) {
      return CODES::LEFT;
    }

    dummyNode_.setTranslation(std::get<0>(state));
    dummyNode_.setRotation(std::get<1>(state));

    turnRight_(&dummyNode_);
    moveForward_(&dummyNode_);
    dist_travelled =
        (std::get<0>(state) - dummyNode_.getAbsolutePosition()).norm();
    if (dist_travelled > minTravel) {
      return CODES::RIGHT;
    }

    return CODES::ERROR;
  }

  return CODES::FORWARD;
}

nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::calcStepAlong(
    const std::tuple<vec3f, quatf>& state,
    const nav::ShortestPath& path) {
  VLOG(1) << path.geodesicDistance;
  if (path.geodesicDistance == std::numeric_limits<float>::infinity())
    return CODES::ERROR;

  if (path.geodesicDistance < goalDist_)
    return CODES::STOP;

  vec3f grad = path.points[1] - path.points[0];
  // Project into x-z plane
  grad[1] = 0;

  grad.normalize();
  // Life gets weird if the gradient is anti-parallel with forward
  // So add a little offset in the x direction to deal with that
  if (grad.dot(geo::ESP_FRONT) < (-1 + 1e-3))
    grad += 1e-3 * geo::ESP_UP.cross(geo::ESP_FRONT);

  const quatf gradDir = quatf::FromTwoVectors(geo::ESP_FRONT, grad);

  const float alpha = gradDir.angularDistance(std::get<1>(state));
  VLOG(1) << alpha;
  if (alpha <= turnAmount_ / 2.0 + 1e-3)
    return CODES::FORWARD;

  // There are some edge cases where the gradient doesn't line up with makes
  // progress the fastest, so we will always try forward
  dummyNode_.setTranslation(std::get<0>(state));
  dummyNode_.setRotation(std::get<1>(state));
  moveForward_(&dummyNode_);
  const float newGeoDist =
      this->geoDist(dummyNode_.getAbsolutePosition(), path.requestedEnd);
  if ((path.geodesicDistance - newGeoDist) > 0.5 * forwardAmount_) {
    return CODES::FORWARD;
  }

  dummyNode_.setTranslation(std::get<0>(state));
  dummyNode_.setRotation(std::get<1>(state));
  turnLeft_(&dummyNode_);

  if (gradDir.angularDistance(dummyNode_.getRotation()) < alpha)
    return CODES::LEFT;
  else
    return CODES::RIGHT;
}

nav::GreedyGeodesicFollowerImpl::CODES
nav::GreedyGeodesicFollowerImpl::nextActionAlong(
    const std::tuple<vec3f, quatf>& start,
    const vec3f& end) {
  nav::ShortestPath path;
  path.requestedStart = std::get<0>(start);
  path.requestedEnd = end;
  pathfinder_->findPath(path);

  CODES action = calcStepAlong(start, path);
  if (action == CODES::FORWARD)
    action = checkForward(start);

  return action;
}

std::vector<nav::GreedyGeodesicFollowerImpl::CODES>
nav::GreedyGeodesicFollowerImpl::findPath(
    const std::tuple<vec3f, quatf>& startState,
    const vec3f& end) {
  constexpr int maxActions = 1e4;
  std::vector<CODES> actions;

  std::tuple<vec3f, quatf> state = startState;
  nav::ShortestPath path;
  path.requestedStart = std::get<0>(state);
  path.requestedEnd = end;
  pathfinder_->findPath(path);

  do {
    CODES nextAction = calcStepAlong(state, path);
    if (nextAction == CODES::FORWARD)
      nextAction = checkForward(state);

    actions.push_back(nextAction);

    dummyNode_.setTranslation(std::get<0>(state));
    dummyNode_.setRotation(std::get<1>(state));

    switch (nextAction) {
      case CODES::FORWARD:
        moveForward_(&dummyNode_);

        path.requestedStart = dummyNode_.getAbsolutePosition();
        pathfinder_->findPath(path);
        break;

      case CODES::LEFT:
        turnLeft_(&dummyNode_);
        break;

      case CODES::RIGHT:
        turnRight_(&dummyNode_);
        break;

      default:
        break;
    }

    state = std::make_tuple(dummyNode_.getAbsolutePosition(),
                            dummyNode_.getRotation());

  } while ((actions.back() != CODES::STOP && actions.back() != CODES::ERROR) &&
           actions.size() < maxActions);

  if (actions.back() == CODES::ERROR)
    actions.clear();

  if (actions.size() >= maxActions)
    actions.clear();

  return actions;
}
}  // namespace esp
