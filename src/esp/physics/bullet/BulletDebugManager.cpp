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

std::string BulletDebugManager::getCollisionFilteringSummary(bool doVerbose) {
  std::stringstream s2;
  for (const auto& pair0 : collisionObjectToDebugName_) {
    const btCollisionObject* colObj0 = pair0.first;
    s2 << getDebugStringForCollisionObject(colObj0) << std::endl;
  }

  if (doVerbose) {
    for (const auto& pair0 : collisionObjectToDebugName_) {
      std::stringstream s;
      const btCollisionObject* colObj0 = pair0.first;
      s << getDebugStringForCollisionObject(colObj0)
        << " may collide with:" << std::endl;

      bool foundAny = false;
      for (const auto& pair1 : collisionObjectToDebugName_) {
        const btCollisionObject* colObj1 = pair1.first;
        const std::string& name1 = pair1.second;
        if (colObj1 == colObj0) {
          continue;
        }

        const auto* broadphaseHandle0 = colObj0->getBroadphaseHandle();
        const auto* broadphaseHandle1 = colObj1->getBroadphaseHandle();

        // logic copied from
        // btHashedOverlappingPairCache::needsBroadphaseCollision
        bool collides = (broadphaseHandle0->m_collisionFilterGroup &
                         broadphaseHandle1->m_collisionFilterMask) != 0;
        collides = collides && (broadphaseHandle1->m_collisionFilterGroup &
                                broadphaseHandle0->m_collisionFilterMask);

        if (collides) {
          s << "  " << name1 << std::endl;
          foundAny = true;
        }
      }

      if (!foundAny) {
        s << "  (none)" << std::endl;
      }

      LOG(INFO) << s.str();
    }
  }

  std::string tmp = s2.str();
  return tmp;
}

}  // namespace physics
}  // namespace esp
