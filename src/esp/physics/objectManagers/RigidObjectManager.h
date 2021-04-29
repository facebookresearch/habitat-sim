// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDOBJECTMANAGER_H
#define ESP_PHYSICS_RIGIDOBJECTMANAGER_H

#include "RigidBaseManager.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"
namespace esp {
namespace physics {

class RigidObjectManager : public RigidBaseManager<ManagedRigidObject> {
 public:
  RigidObjectManager(
      const std::shared_ptr<esp::physics::PhysicsManager>& physMgr)
      : RigidBaseManager<ManagedRigidObject>::RigidBaseManager(physMgr,
                                                               "RigidObject") {
    buildCtorFuncPtrMaps();
  }

 protected:
  /**
   * @brief This function will build the appropriate @ref copyConstructorMap_
   * copy constructor function pointer map for this container's managed object,
   * keyed on the managed object's class type.  This MUST be called in the
   * constructor of the -instancing- class.
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["ManagedRigidObject"] =
        &RigidObjectManager::createObjectCopy<ManagedRigidObject>;
  }  // ObjectAttributesManager::buildCtorFuncPtrMaps()

 public:
  ESP_SMART_POINTERS(RigidObjectManager)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDOBJECTMANAGER_H
