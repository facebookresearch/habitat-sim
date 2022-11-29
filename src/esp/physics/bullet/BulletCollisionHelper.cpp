// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletCollisionHelper.h"

#include <sstream>
#include <utility>

namespace esp {
namespace physics {

void BulletCollisionHelper::mapCollisionObjectTo(
    const btCollisionObject* colObj,
    const std::string& debugName) {
  collisionObjectToDebugName_[colObj] = debugName;

  // reference code for debugging
  // std::cout << debugName << ": " << ((void*)(colObj)) << std::endl;
}

int BulletCollisionHelper::getNumActiveContactPoints(
    btMultiBodyDynamicsWorld* bWorld) {
  int count = 0;
  auto func = [&](const btCollisionObject*, const btCollisionObject*,
                  const btPersistentManifold* manifold) {
    count += manifold->getNumContacts();
  };

  processActiveManifolds(bWorld, func);

  return count;
}

int BulletCollisionHelper::getNumActiveOverlappingPairs(
    btMultiBodyDynamicsWorld* bWorld) {
  int count = 0;
  auto func = [&](const btCollisionObject*, const btCollisionObject*,
                  const btPersistentManifold*) { ++count; };

  processActiveManifolds(bWorld, func);

  return count;
}

template <typename Func>
void BulletCollisionHelper::processActiveManifolds(
    btMultiBodyDynamicsWorld* bWorld,
    Func func) {
  auto* dispatcher = bWorld->getDispatcher();
  for (int i = 0; i < dispatcher->getNumManifolds(); ++i) {
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

std::string BulletCollisionHelper::getDebugStringForCollisionObject(
    const btCollisionObject* colObj) {
  std::string name = "unknown";
  auto colObjToNameIter = collisionObjectToDebugName_.find(colObj);
  if (colObjToNameIter != collisionObjectToDebugName_.end()) {
    name = colObjToNameIter->second;
  }

  // reference code to shorten names
  // const int maxLen = 60;
  // if (name.size() > maxLen) {
  //  name = name.substr(name.size() - maxLen, maxLen);
  //}

#if 0  // reference code for extra debug information
  std::string result =
      name + ", act state " + std::to_string(colObj->getActivationState());

  const auto* broadphaseHandle = colObj->getBroadphaseHandle();
  if (broadphaseHandle) {
    result +=
        ", group " + std::to_string(broadphaseHandle->m_collisionFilterGroup) +
        ", mask " + std::to_string(broadphaseHandle->m_collisionFilterMask);
  } else {
    // this probably means the object hasn't been added to the bullet world
    result += ", no broadphase handle";
  }
#else
  std::string result = std::move(name);
#endif
  return result;
}

std::string BulletCollisionHelper::getStepCollisionSummary(
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
