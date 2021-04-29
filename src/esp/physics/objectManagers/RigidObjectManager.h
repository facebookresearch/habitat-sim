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
  RigidObjectManager()
      : RigidBaseManager<ManagedRigidObject>::RigidBaseManager("RigidObject") {
    this->buildCtorFuncPtrMaps();
  }

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle handle name to be assigned to the managed object.
   * @param builtFromConfig Unused for wrapper objects.  All wrappers are
   * constructed from scratch.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  ManagedRigidObject::ptr initNewObjectInternal(
      const std::string& objectHandle,
      CORRADE_UNUSED bool builtFromConfig) override;

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

  /**
   * @brief implementation of managed object type-specific registration
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  int registerObjectFinalize(ManagedRigidObject::ptr object,
                             const std::string& objectHandle,
                             CORRADE_UNUSED bool forceRegistration) override;

 public:
  ESP_SMART_POINTERS(RigidObjectManager)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDOBJECTMANAGER_H
