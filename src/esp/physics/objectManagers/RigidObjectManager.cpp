// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObjectManager.h"
namespace esp {
namespace physics {

ManagedRigidObject::ptr RigidObjectManager::initNewObjectInternal(
    const std::string& objectHandle,
    bool builtFromConfig) {
  ManagedRigidObject::ptr newObjectWrapper =
      this->constructFromDefault(objectHandle);
  if (nullptr == newObjectWrapper) {
    newObjectWrapper = ManagedRigidObject::create(objectHandle);
  }
  return newObjectWrapper;
}  // RigidObjectManager::initNewObjectInternal(

int RigidObjectManager::registerObjectFinalize(ManagedRigidObject::ptr object,
                                               const std::string& objectHandle,
                                               bool forceRegistration) {
  // Add object wrapper to template library
  return this->addObjectToLibrary(object, objectHandle);
}  // RigidObjectManager::registerObjectFinalize

}  // namespace physics
}  // namespace esp
