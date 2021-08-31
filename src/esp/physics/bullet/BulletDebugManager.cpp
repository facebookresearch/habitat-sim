// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletDebugManager.h"

#include "esp/core/logging.h"

#include <sstream>

namespace esp {
namespace physics {

void BulletDebugManager::mapCollisionObjectTo(const btCollisionObject* colObj,
                                              const std::string& debugName) {
  collisionObjectToDebugName_[colObj] = debugName;

  // reference code for debugging
  // std::cout << debugName << ": " << ((void*)(colObj)) << std::endl;
}

int BulletDebugManager::getNumActiveContactPoints(
    btMultiBodyDynamicsWorld* bWorld) {
  int count = 0;
  auto func = [&](const btCollisionObject*, const btCollisionObject*,
                  const btPersistentManifold* manifold) {
    count += manifold->getNumContacts();
  };

  processActiveManifolds(bWorld, func);

  return count;
}

int BulletDebugManager::getNumActiveOverlappingPairs(
    btMultiBodyDynamicsWorld* bWorld) {
  int count = 0;
  auto func = [&](const btCollisionObject*, const btCollisionObject*,
                  const btPersistentManifold*) { count++; };

  processActiveManifolds(bWorld, func);

  return count;
}

template <typename Func>
void BulletDebugManager::processActiveManifolds(
    btMultiBodyDynamicsWorld* bWorld,
    Func func) {
  auto* dispatcher = bWorld->getDispatcher();
  for (int i = 0; i < dispatcher->getNumManifolds(); i++) {
    auto* manifold = dispatcher->getManifoldByIndexInternal(i);
    const btCollisionObject* colObj0 =
        static_cast<const btCollisionObject*>(manifold->getBody0());
    const btCollisionObject* colObj1 =
        static_cast<const btCollisionObject*>(manifold->getBody1());

    // logic copied from btSimulationIslandManager::buildIslands. We want to
    // count manifolds only if related to non-sleeping bodies.
    if (((colObj0) && colObj0->getActivationState() != ISLAND_SLEEPING) ||
        ((colObj1) && colObj1->getActivationState() != ISLAND_SLEEPING)) {
      func(colObj0, colObj1, manifold);
    }
  }
}

std::string BulletDebugManager::getCollisionObjectName(
    const btCollisionObject* colObj) {
  return getDebugStringForCollisionObject(colObj);
#if 0
  if (collisionObjectToDebugName_.count(colObj)) {
    return collisionObjectToDebugName_[colObj];
  }
  return "Unknown";
#endif
}

std::string BulletDebugManager::getDebugStringForCollisionObject(
    const btCollisionObject* colObj) {
  std::string name = (collisionObjectToDebugName_.count(colObj))
                         ? collisionObjectToDebugName_[colObj]
                         : "unknown";

  // reference code to shorten names
  // const int maxLen = 60;
  // if (name.size() > maxLen) {
  //  name = name.substr(name.size() - maxLen, maxLen);
  //}

  /*
  #define ACTIVE_TAG 1
  #define ISLAND_SLEEPING 2
  #define WANTS_DEACTIVATION 3
  #define DISABLE_DEACTIVATION 4
  #define DISABLE_SIMULATION 5
  */
  const char* activationStateNames[] = {"invalid",
                                        "active",
                                        "sleeping",
                                        "wants_deactivation",
                                        "disable_deactivation",
                                        "disable_simulation"};
  std::stringstream result;

  result << static_cast<const void*>(colObj) << " " << name << ", "
         << activationStateNames[colObj->getActivationState()];

  const auto* broadphaseHandle = colObj->getBroadphaseHandle();
  if (broadphaseHandle) {
    result << ", group "
           << std::to_string(broadphaseHandle->m_collisionFilterGroup) +
                  ", mask "
           << std::to_string(broadphaseHandle->m_collisionFilterMask);
  } else {
    // this probably means the object hasn't been added to the bullet world
    result << ", no broadphase handle";
  }
  return result.str();
}

std::string BulletDebugManager::getStepCollisionSummary(
    btMultiBodyDynamicsWorld* bWorld) {
  std::stringstream s;
  auto func = [&](const btCollisionObject* colObj0,
                  const btCollisionObject* colObj1,
                  const btPersistentManifold* manifold) {
    if (manifold->getNumContacts() > 0) {
      s << "[" << getDebugStringForCollisionObject(colObj0) << "] vs ["
        << getDebugStringForCollisionObject(colObj1) << "], "
        << manifold->getNumContacts() << " points" << std::endl;
    }
  };

  processActiveManifolds(bWorld, func);

  const bool isEmpty = s.rdbuf()->in_avail() == 0;
  if (isEmpty) {
    s << "(no active collision manifolds)" << std::endl;
  }

  return s.str();
}

}  // namespace physics
}  // namespace esp
