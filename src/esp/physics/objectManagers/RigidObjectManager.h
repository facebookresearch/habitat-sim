// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDOBJECTMANAGER_H
#define ESP_PHYSICS_RIGIDOBJECTMANAGER_H

#include "RigidBaseManager.h"
#include "esp/physics/bullet/objectWrappers/ManagedBulletRigidObject.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"
namespace esp {
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality shared for
 * managing all @ref esp::physics::ManagedRigidObject wrappers.
 */
class RigidObjectManager
    : public esp::physics::RigidBaseManager<ManagedRigidObject> {
 public:
  RigidObjectManager();

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager.  This method
   * calls the physicsManager method with the same signature that queries for a
   * DrawableGroup from Simulator.
   * @param attributesHandle The handle of the object attributes used as the key
   * to query @ref esp::metadata::managers::ObjectAttributesManager.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return a copy of the instanced object, or nullptr.
   */
  std::shared_ptr<ManagedRigidObject> addObjectByHandle(
      const std::string& attributesHandle,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Templated version of addObjectByHandle. Will cast result to
   * appropriate dynamics library wrapper.
   * @param attributesHandle The handle of the object attributes used as the key
   * to query @ref esp::metadata::managers::ObjectAttributesManager.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return a copy of the instanced object, appropriately cast, or nullptr.
   */
  std::shared_ptr<ManagedRigidObject> addBulletObjectByHandle(
      const std::string& attributesHandle,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    std::shared_ptr<ManagedRigidObject> objPtr =
        addObjectByHandle(attributesHandle, attachmentNode, lightSetup);

    if (std::shared_ptr<ManagedBulletRigidObject> castObjPtr =
            std::dynamic_pointer_cast<ManagedBulletRigidObject>(objPtr)) {
      return castObjPtr;
    }
    return objPtr;
  }

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager by template
   * ID.  This method calls the physicsManager method with the same signature
   * that queries for a DrawableGroup from Simulator.
   * @param attributesID The ID of the object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup An optional custom lightsetup for the object.
   * @return a copy of the instanced object, or nullptr.
   */
  std::shared_ptr<ManagedRigidObject> addObjectByID(
      int attributesID,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Templated version of addObjectByID. Will cast result to
   * appropriate dynamics library wrapper.
   * @param attributesID The ID of the object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return a copy of the instanced object, appropriately cast, or nullptr.
   */
  std::shared_ptr<ManagedRigidObject> addBulletObjectByID(
      const int attributesID,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    std::shared_ptr<ManagedRigidObject> objPtr =
        addObjectByID(attributesID, attachmentNode, lightSetup);
    if (std::shared_ptr<ManagedBulletRigidObject> castObjPtr =
            std::dynamic_pointer_cast<ManagedBulletRigidObject>(objPtr)) {
      return castObjPtr;
    }
    return objPtr;
  }

  /**
   * @brief Overload of standard @ref
   * esp::core::ManagedContainer::removeObjectByID to allow for the retention
   * of scene node or visual node of the underlying RigidObject after it and
   * its wrapper's removal.
   *
   * @param objectID The ID of the managed object to be deleted.
   * @param deleteObjectNode If true, deletes the object's scene node.
   * Otherwise detaches the object from simulation.
   * @param deleteVisualNode If true, deletes the object's visual node.
   * Otherwise detaches the object from simulation. Is not considered if
   * deleteObjectNode==true.
   * @return this always returns a nullptr, since a wrapper of a deleted
   * object is unusable.
   */

  std::shared_ptr<ManagedRigidObject> removePhysObjectByID(
      int objectID,
      bool deleteObjectNode = true,
      bool deleteVisualNode = true);

  /**
   * @brief Overload of standard @ref
   * esp::core::ManagedContainer::removeObjectByHandle to allow for the
   * retention of scene node or visual node of the underlying RigidObject
   * after it and its wrapper's removal.
   *
   * @param objectHandle The handle of the managed object to be deleted.
   * @param deleteObjectNode If true, deletes the object's scene node.
   * Otherwise detaches the object from simulation.
   * @param deleteVisualNode If true, deletes the object's visual node.
   * Otherwise detaches the object from simulation. Is not considered if
   * deleteObjectNode==true.
   * @return this always returns a nullptr, since a wrapper of a deleted
   * object is unusable.
   */
  std::shared_ptr<ManagedRigidObject> removePhysObjectByHandle(
      const std::string& objectHandle,
      bool deleteObjectNode = true,
      bool deleteVisualNode = true);

 protected:
  /**
   * @brief This method will remove rigid objects from physics manager.  The
   * wrapper has already been removed by the time this method is called (this is
   * called from @ref esp::core::ManagedContainerBase::deleteObjectInternal)
   *
   * @param objectID the ID of the managed object to remove
   * @param objectHandle the string key of the managed object to remove.
   */
  void deleteObjectInternalFinalize(
      int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {
    if (auto physMgr = this->getPhysicsManager()) {
      // don't try to double remove or will throw an exception
      if (physMgr->isValidRigidObjectId(objectID)) {
        physMgr->removeObject(objectID);
      }
    }
  }  // deleteObjectInternalFinalize

 public:
  ESP_SMART_POINTERS(RigidObjectManager)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDOBJECTMANAGER_H
