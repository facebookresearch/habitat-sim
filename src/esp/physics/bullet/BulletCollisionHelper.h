// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <string>
#include <unordered_map>

class btCollisionObject;
class btMultiBodyDynamicsWorld;

namespace esp {
namespace physics {

class BulletRigidObject;

class BulletCollisionHelper {
 public:
  static BulletCollisionHelper& get() {
    static BulletCollisionHelper mgr;
    return mgr;
  }

  /**
   * @brief Associate a human-readable debug name with the collision object. See
   * also getStepCollisionSummary.
   */
  void mapCollisionObjectTo(const btCollisionObject* colObj,
                            const std::string& debugName);

  /**
   * @brief see BulletPhysicsManager.h getNumActiveContactPoints
   */
  int getNumActiveContactPoints(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief see BulletPhysicsManager.h getNumActiveOverlappingPairs
   */
  int getNumActiveOverlappingPairs(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief see BulletPhysicsManager.h getStepCollisionSummary
   */
  std::string getStepCollisionSummary(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief Get the object's debug name plus some useful Bullet collision state.
   */
  std::string getDebugStringForCollisionObject(const btCollisionObject* colObj);

 private:
  template <typename Func>
  void processActiveManifolds(btMultiBodyDynamicsWorld* bWorld, Func func);

  std::unordered_map<const btCollisionObject*, std::string>
      collisionObjectToDebugName_;
};

}  // end namespace physics
}  // end namespace esp
