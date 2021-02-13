// Copyright (c) Facebook, Inc. and its affiliates.
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

class BulletDebugManager {
 public:
  static BulletDebugManager& get() {
    static BulletDebugManager mgr;
    return mgr;
  }

  /**
   * @brief Associate a human-readable debug name with the collision object. See
   * also getStepCollisionSummary.
   */
  void mapCollisionObjectTo(const btCollisionObject* colObj,
                            const std::string& debugName);

  /**
   * @brief The number of contact points that were active during the last step.
   * An object resting on another object will involve several active contact
   * points. Once both objects are asleep, the contact points are inactive. This
   * count is a proxy for complexity/cost of collision-handling in the current
   * scene. See also getNumActiveOverlappingPairs.
   */
  int getNumActiveContactPoints(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief The number of active overlapping pairs during the last step. When
   * object bounding boxes overlap and either object is active, additional
   * "narrowphase" collision-detection must be run. This count is a proxy for
   * complexity/cost of collision-handling in the current scene. See also
   * getNumActiveContactPoints.
   */
  int getNumActiveOverlappingPairs(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief Get a summary of collision-processing from the last physics step.
   */
  std::string getStepCollisionSummary(btMultiBodyDynamicsWorld* bWorld);

  /**
   * @brief Describe collision-filtering for all known collision objects (see
   * also mapCollisionObjectTo).
   *
   * Unfortunately, this will cause a crash if you have added and then removed
   * physics objects, because we don't yet have unmapCollisionObject. So don't
   * use this in that case!
   */
  std::string getCollisionFilteringSummary(bool doVerbose = true);

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
