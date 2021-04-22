// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_
#define ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_

#include <Corrade/Utility/Macros.h>

#include "esp/core/AbstractManagedObject.h"
#include "esp/physics/PhysicsObjectBase.h"

namespace esp {
namespace physics {

/**
 * @brief Base class template for wrapper for physics objects of all kinds to
 * enable Managed Container access.
 */
template <class T>
class AbstractManagedPhysicsObject : public esp::core::AbstractManagedObject {
 public:
  static_assert(
      std::is_base_of<esp::physics::PhysicsObjectBase, T>::value,
      "AbstractManagedPhysicsObject :: Managed physics object type must be "
      "derived from esp::physics::PhysicsObjectBase");

  typedef std::weak_ptr<T> WeakObjRef;

  AbstractManagedPhysicsObject(std::shared_ptr<T>& objPtr,
                               const std::string& classKey)
      : weakObjRef_(objPtr) {
    setClassKey(classKey);
  }

  ~AbstractManagedPhysicsObject() override = default;

  /**
   * @brief Get this managed object's class.  Should only be set from
   * constructor. Used as key in constructor function pointer maps in Managed
   * Container.
   */
  std::string getClassKey() const override { return classKey_; }

  std::string getHandle() const override {
    if (auto sp = getObjectReference()) {
      return sp->getObjectName();
    } else {
      return "";
    }
  }
  void setHandle(const std::string& name) override {
    if (auto sp = getObjectReference()) {
      sp->setObjectName(name);
      // TODO need to update object manager
    }
  }

  /**
   *  @brief Managed Physics objects manage their own IDs, so this is
   * unsettable.
   */
  void setID(CORRADE_UNUSED int ID) override {}
  /**
   * @brief return the object's ID or nullptr if doesn't exist.
   */
  int getID() const override {
    if (auto sp = getObjectReference()) {
      return sp->getObjectID();
    } else {
      return ID_UNDEFINED;
    }
  }  // getID()

  /**
   * @brief File directory is unused for managed phyiscs objects
   */
  void setFileDirectory(
      CORRADE_UNUSED const std::string& fileDirectory) override {}
  std::string getFileDirectory() const override { return ""; }

 protected:
  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's
   * @p weakObjRef_ if it exists; if not, it provides a message and executes
   * appropriate cleanup code.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * dne.
   */
  std::shared_ptr<T> inline getObjectReference() const {
    std::shared_ptr<T> sp = weakObjRef_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      LOG(WARNING)
          << "This object no longer exists.  Please delete any variable "
             "references.";
    }
    return sp;
  }  // getObjectReference

  /**
   * @brief Set this managed object's class.  Should only be set from
   * constructor. Used as key in constructor function pointer maps in Managed
   * Container.
   * @param classKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setClassKey(const std::string& classKey) { classKey_ = classKey; }

  /**
   * @brief Weak ref to object. If user has copy of this wrapper but object has
   * been deleted, this will be nullptr.
   */
  WeakObjRef weakObjRef_;

  /**
   * @brief Name of instancing class responsible for this managed object
   */
  std::string classKey_;

 public:
  ESP_SMART_POINTERS(AbstractManagedPhysicsObject<T>)
};  // class ManagedPhysicsObject

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_
