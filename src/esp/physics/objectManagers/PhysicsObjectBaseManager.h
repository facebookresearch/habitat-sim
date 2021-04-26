// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
#define ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H

/** @file
 * @brief Class Template @ref esp::physics::PhysicsObjectBaseManager
 */

#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"

#include "esp/core/ManagedContainer.h"

namespace esp {
namespace core {
enum class ManagedObjectAccess;
class ManagedContainerBase;
}  // namespace core
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality for
 * managineg object wrappers specializing @ref
 * esp::physics::AbstractManagedPhysicsObject template.
 * @tparam T the type of managed physics object wrapper a particular
 * specialization of this class works with.  Must inherit from @ref
 * esp::physics::AbstractManagedPhysicsObject
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */
template <class T>
class PhysicsObjectBaseManager
    : public esp::core::ManagedContainer<T, core::ManagedObjectAccess::Copy> {
 public:
  typedef std::shared_ptr<T> ObjWrapperPtr;
  PhysicsObjectBaseManager(
      std::shared_ptr<esp::physics::PhysicsManager> physMgr,
      const std::string& objType)
      : esp::core::ManagedContainer<T, core::ManagedObjectAccess::Copy>::
            ManagedContainer(objType),
        weakPhysManager_(physMgr) {}
  ~PhysicsObjectBaseManager() override = default;

  /**
   * @brief Creates an instance of a managed object described by passed string.
   *
   * If a managed object exists with this handle, the existing managed object
   * will be overwritten with the newly created one if @ref
   * registerObject is true.
   *
   * @param objectHandle the origin of the desired managed object to be
   * created.
   * @param registerObject whether to add this managed object to the
   * library or not. If the user is going to edit this managed object, this
   * should be false. Defaults to true. If specified as true, then this function
   * returns a copy of the registered managed object.
   * @return a reference to the desired managed object.
   */
  ObjWrapperPtr createObject(const std::string& objectHandle,
                             bool registerObject = true) override;

  /**
   * @brief Parse passed JSON Document specifically for @ref ManagedPtr object.
   * It always returns a @ref ManagedPtr object.
   * @param filename UNUSED The name of the file describing the @ref ManagedPtr,
   * used as managed object handle/name on create.
   * @param jsonConfig UNUSED json document to parse - assumed to be legal JSON
   * doc.
   * @return nullptr - this function is not supported for physics object
   * wrappers.
   */
  ObjWrapperPtr buildObjectFromJSONDoc(
      CORRADE_UNUSED const std::string& filename,
      CORRADE_UNUSED const io::JsonGenericValue& jsonConfig) override {
    return nullptr;
  }  // buildObjectFromJSONDoc

 protected:
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
  virtual int registerObjectFinalize(ObjWrapperPtr object,
                                     const std::string& objectHandle,
                                     bool forceRegistration) override;

  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle handle name to be assigned to the managed object.
   * @param builtFromConfig Managed Object is being constructed from a config
   * file (i.e. @p objectHandle is config file filename).  If false this means
   * Manage Object is being constructed as some kind of new/default.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  virtual ObjWrapperPtr initNewObjectInternal(const std::string& objectHandle,
                                              bool builtFromConfig) override;

  /**
   * @brief return a reference to physicsManager_, or null ptr if it does not
   * exist anymore.  This is necessary since a reference of this manager may
   * linger in python after simulator/physicsManager get clobbered/rebuilt.
   */
  std::shared_ptr<esp::physics::PhysicsManager> getPhysicsManager() const {
    std::shared_ptr<esp::physics::PhysicsManager> sp = weakPhysManager_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      LOG(WARNING) << "This object manager no longer exists.  Please delete "
                      "any variable references.";
    }
    return sp;
  }  // getPhysicsManager

  /** @brief Weak reference to owning physics manager.
   */
  std::weak_ptr<esp::physics::PhysicsManager> weakPhysManager_;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBaseManager<T>)

};  // class PhysicsObjectBaseManager

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
