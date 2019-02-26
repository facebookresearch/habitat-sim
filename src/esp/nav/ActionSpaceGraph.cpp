// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/nav/ActionSpaceGraph.h"
#include "esp/geo/geo.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "Recast.h"

#include <sophus/so3.hpp>

static_assert(sizeof(uint64_t) == 8, "uint64_t must be 64 bits");

namespace esp {
namespace nav {
namespace impl {

namespace {

constexpr uint64_t numHeadingBits = 10;
static_assert(((64 - numHeadingBits) % 3) == 0,
              "64 - numHeadingBits must be a multiple of 3");

const uint64_t numPosBits = (64 - numHeadingBits) / 3;  // = 18

constexpr uint64_t first10bits = 0x3FFul;
constexpr uint64_t first18bits = 0x3FFFFul;

const uint64_t HEADING_OFFSET = 64 - numHeadingBits;
const uint64_t HEADING_MASK = first10bits << HEADING_OFFSET;

const uint64_t X_OFFSET = numPosBits * 0;
const uint64_t X_MASK = first18bits << X_OFFSET;

const uint64_t Y_OFFSET = numPosBits * 1;
const uint64_t Y_MASK = first18bits << Y_OFFSET;

const uint64_t Z_OFFSET = numPosBits * 2;
const uint64_t Z_MASK = first18bits << Z_OFFSET;

uint64_t encodeAsBits(const vec4ul& k) {
  return (k[3] << HEADING_OFFSET) + (k[2] << Z_OFFSET) + (k[1] << Y_OFFSET) +
         (k[0] << X_OFFSET);
}

vec4ul decodeFromBits(uint64_t bits) {
  vec4ul k;
  k[0] = (bits & X_MASK) >> X_OFFSET;
  k[1] = (bits & Y_MASK) >> Y_OFFSET;
  k[2] = (bits & Z_MASK) >> Z_OFFSET;
  k[3] = (bits & HEADING_MASK) >> HEADING_OFFSET;

  return k;
}

// TODO Make this not a hard coded constant :)
constexpr double minXYZ = -25.0;
const vec3f xyzOffset = -vec3f(minXYZ, minXYZ, minXYZ);

double angularDistance(const quatf& q1, const quatf& q2) {
  return q1.angularDistance(q2) / M_PI;
}
}  // namespace

ActionSpaceGraph::ActionSpaceGraph(
    PathFinder::ptr pf,
    const agent::AgentConfiguration& config,
    const scene::ObjectControls& controls,
    const double closeToObstacleCostMultiplier /* = 5.0 */,
    const double paddingRadius /* = 0.0 */)
    : finder_{pf},
      config_{config},
      controls_{controls},
      closeToObstacleCostMultiplier_{closeToObstacleCostMultiplier},
      paddingRadius_{paddingRadius},
      dummy_{dummyGraph_.getRootNode()} {
  for (auto& it : config_.actionSpace) {
    if (it.second->name == forwardActionName) {
      forwardAction = it.first;
    } else if (it.second->name == lookLeftActionName) {
      lookLeftAction = it.first;
    } else if (it.second->name == lookRightActionName) {
      lookRightAction = it.first;
    }
  }

  ASSERT(config_.actionSpace.find(forwardAction) != config_.actionSpace.end());
  ASSERT(config_.actionSpace.find(lookRightAction) !=
         config_.actionSpace.end());
  ASSERT(config_.actionSpace.find(lookLeftAction) != config_.actionSpace.end());
  ASSERT(config_.actionSpace[lookLeftAction]->actuation["amount"] ==
         config_.actionSpace[lookRightAction]->actuation["amount"]);
  const float numTurns =
      360.0f / config_.actionSpace[lookLeftAction]->actuation["amount"];
  ASSERT(numTurns == static_cast<float>(static_cast<int>(numTurns)));

  stepSize_ = config_.actionSpace[forwardAction]->actuation["amount"];
  turnSize_ =
      config_.actionSpace[lookRightAction]->actuation["amount"] * M_PI / 180.0;
  cellsPerMeter_ = 2.0 / (stepSize_ * std::sin(turnSize_));
  cellSize_ = 1.0 / cellsPerMeter_;
};

ActionSpaceGraph::~ActionSpaceGraph(){};

void ActionSpaceGraph::goalNodeKey(const uint64_t& newKey) {
  goalNodeKey_ = newKey;
  heuristicCache_.clear();
}

void ActionSpaceGraph::calibrateRotations(const quatf& initRotation) {
  dummy_.setRotation(initRotation);
  dummy_.setTranslation(vec3f::Zero());

  calibratedRotations_.clear();
  const int numTurns = 2.0 * M_PI / turnSize_;
  for (int i = 0; i < numTurns; ++i) {
    calibratedRotations_.emplace_back(dummy_.getRotation());

    takeAction(lookLeftAction, dummy_);
  }
}

int ActionSpaceGraph::getClosestCalibratedRotation(const quatf& q) const {
  int idx = 0;
  double dist = angularDistance(q, calibratedRotations_[idx]);
  for (int i = 1; i < calibratedRotations_.size(); ++i) {
    double newDist = angularDistance(q, calibratedRotations_[i]);

    idx = newDist < dist ? i : idx;
    dist = std::min(dist, newDist);
  }

  return idx;
}

uint64_t ActionSpaceGraph::toKey(const vec3f& _pos,
                                 const quatf& rotation) const {
  // Project point onto mesh
  vec3f pos;
  dtPolyRef polyRef;
  // Size of bounding box to search in for closest polygon.
  const float polyPickExt[3] = {2, 4, 2};
  finder_->navQuery_->findNearestPoly(_pos.data(), polyPickExt,
                                      finder_->filter_, &polyRef, pos.data());

  // vec3f pos = _pos;
  pos += xyzOffset;
  vec4ul key;

  Eigen::Vector3d posd = pos.cast<double>() * cellsPerMeter_;
  key.head<3>() = posd.cast<uint64_t>();

  key[3] = getClosestCalibratedRotation(rotation);

  return encodeAsBits(key);
}

std::tuple<vec3f, quatf> ActionSpaceGraph::toWorld(const uint64_t& bits) const {
  vec4ul key = decodeFromBits(bits);
  Eigen::Vector3d posd =
      (key.head<3>().cast<double>()) / cellsPerMeter_ -
      xyzOffset.cast<double>() +
      Eigen::Vector3d(cellSize_ / 2.0, cellSize_ / 2.0, cellSize_ / 2.0);

  const quatf rotation = calibratedRotations_[key[3]];

  return std::make_tuple(posd.cast<float>(), rotation);
}

std::unordered_map<std::string, uint64_t> ActionSpaceGraph::tryActionsFrom(
    const uint64_t& key) {
  std::unordered_map<std::string, uint64_t> retMap;
  vec3f pos;
  quatf rotation;
  std::tie(pos, rotation) = toWorld(key);

  const int currentRotation = getClosestCalibratedRotation(rotation);
  if (currentRotation == 0) {
    retMap[lookLeftAction] =
        toKey(pos, calibratedRotations_[currentRotation + 1]);
    retMap[lookRightAction] =
        toKey(pos, calibratedRotations_[calibratedRotations_.size() - 1]);
  } else if (currentRotation == calibratedRotations_.size() - 1) {
    retMap[lookLeftAction] = toKey(pos, calibratedRotations_[0]);
    retMap[lookRightAction] =
        toKey(pos, calibratedRotations_[currentRotation - 1]);
  } else {
    retMap[lookLeftAction] =
        toKey(pos, calibratedRotations_[currentRotation + 1]);
    retMap[lookRightAction] =
        toKey(pos, calibratedRotations_[currentRotation - 1]);
  }

  dummy_.setTranslation(pos).setRotation(rotation);

  takeAction(forwardAction, dummy_);

  retMap.emplace(forwardAction,
                 toKey(dummy_.getAbsolutePosition(), dummy_.getRotation()));

  return retMap;
}

std::string ActionSpaceGraph::getActionBetween(const uint64_t& fromKey,
                                               const uint64_t& toKey) {
  vec3f p1, p2;
  quatf q1, q2;
  std::tie(p1, q1) = toWorld(fromKey);
  std::tie(p2, q2) = toWorld(toKey);

  const int rID1 = getClosestCalibratedRotation(q1);
  const int rID2 = getClosestCalibratedRotation(q2);

  if (rID1 == rID2) {
    return forwardAction;
  }

  if (rID1 == 0 && rID2 == (calibratedRotations_.size() - 1)) {
    return lookRightAction;
  }

  if (rID1 == (calibratedRotations_.size() - 1) && rID2 == 0) {
    return lookLeftAction;
  }

  return rID2 > rID1 ? lookLeftAction : lookRightAction;
}

std::vector<uint64_t> ActionSpaceGraph::edgesFrom(const uint64_t& key) {
  std::vector<uint64_t> edges;
  auto actionsFrom = tryActionsFrom(key);
  for (auto& it : actionsFrom) {
    if (it.second != key) {
      edges.emplace_back(it.second);
    }
  }

  return edges;
}

float ActionSpaceGraph::edgeWeight(const uint64_t& fromKey,
                                   const uint64_t& toKey) {
  return 1.0 * computeTileWeight(toKey);
}

float ActionSpaceGraph::computeTileWeight(const uint64_t& key) {
  if (paddingRadius_ > 0.0) {
    uint64_t cacheKey = key & (~HEADING_MASK);
    {
      auto it = tileCostCache_.find(cacheKey);
      if (it != tileCostCache_.end()) {
        return it->second;
      }
    }

    vec3f _pos, pos;
    std::tie(_pos, std::ignore) = toWorld(key);

    dtPolyRef polyRef;
    const float polyPickExt[3] = {2, 4, 2};
    finder_->navQuery_->findNearestPoly(_pos.data(), polyPickExt,
                                        finder_->filter_, &polyRef, pos.data());

    vec3f hitPos, hitNormal;
    float hitDist;
    finder_->navQuery_->findDistanceToWall(polyRef, pos.data(), paddingRadius_,
                                           finder_->filter_, &hitDist,
                                           hitPos.data(), hitNormal.data());

    const float tileCost =
        hitDist < paddingRadius_ ? closeToObstacleCostMultiplier_ : 1.0;

    tileCostCache_.emplace(cacheKey, tileCost);

    return tileCost;
  } else {
    return 1.0;
  }
}

float ActionSpaceGraph::heuristic(const uint64_t& key) {
  // clear the heading bits as all headings have the same heuristic
  uint64_t cacheKey = key & (~HEADING_MASK);

  {
    auto it = heuristicCache_.find(cacheKey);
    if (it != heuristicCache_.end()) {
      return it->second;
    }
  }

  ShortestPath path_;
  path_.geodesicDistance = 0.0;
  path_.requestedStart = std::get<0>(toWorld(key));
  path_.requestedEnd = std::get<0>(toWorld(goalNodeKey_));

  finder_->findPath(path_);

  const double expectedDistance =
      std::max(path_.geodesicDistance - stepSize_ * goalExpansionAmount_, 0.0);

  const float h = expectedDistance / stepSize_;

  heuristicCache_.emplace(cacheKey, h);
  return h;
}

void ActionSpaceGraph::takeAction(const std::string& act,
                                  scene::SceneNode& node) {
  const vec3f pos = node.getAbsolutePosition();
  const agent::ActionSpec::ptr& actionSpec = config_.actionSpace[act];
  controls_.getMoveFuncMap().at(actionSpec->name)(
      node, actionSpec->actuation["amount"]);

  vec3f newPos = finder_->tryStep(pos, node.getAbsolutePosition());
  node.setTranslation(newPos);
}

}  // namespace impl
}  // namespace nav
}  // namespace esp
