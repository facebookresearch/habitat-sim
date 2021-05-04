// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDOBJECTMANAGER_H
#define ESP_PHYSICS_RIGIDOBJECTMANAGER_H

#include "RigidBaseManager.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"
namespace esp {
namespace physics {

class RigidObjectManager
    : public esp::physics::RigidBaseManager<ManagedRigidObject> {
 public:
  RigidObjectManager()
      : esp::physics::RigidBaseManager<ManagedRigidObject>::RigidBaseManager(
            "RigidObject") {
    this->buildCtorFuncPtrMaps();
  }

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle Unused for wrapper objects.  All wrappers use the name
   * of their underlying objects.
   * @param builtFromConfig Unused for wrapper objects.  All wrappers are
   * constructed from scratch.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  std::shared_ptr<ManagedRigidObject> initNewObjectInternal(
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool builtFromConfig) override {
    return ManagedRigidObject::create();
  }  // RigidObjectManager::initNewObjectInternal(

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
