// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ArticulatedObjectManager.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/objectWrappers/ManagedBulletArticulatedObject.h"
#endif

namespace esp {
namespace physics {

ArticulatedObjectManager::ArticulatedObjectManager()
    : esp::physics::PhysicsObjectBaseManager<ManagedArticulatedObject>::
          PhysicsObjectBaseManager("ArticulatedObject") {
  // build this manager's copy constructor map
  this->copyConstructorMap_["ManagedArticulatedObject"] =
      &ArticulatedObjectManager::createObjectCopy<ManagedArticulatedObject>;

  // build the function pointers to proper wrapper construction methods, keyed
  // by the wrapper names
  managedObjTypeConstructorMap_["ManagedArticulatedObject"] =
      &ArticulatedObjectManager::createPhysicsObjectWrapper<
          ManagedArticulatedObject>;

#ifdef ESP_BUILD_WITH_BULLET
  this->copyConstructorMap_["ManagedBulletRigidObject"] =
      &ArticulatedObjectManager::createObjectCopy<
          ManagedBulletArticulatedObject>;
  managedObjTypeConstructorMap_["ManagedBulletRigidObject"] =
      &ArticulatedObjectManager::createPhysicsObjectWrapper<
          ManagedBulletArticulatedObject>;
#endif
}

}  // namespace physics
}  // namespace esp
