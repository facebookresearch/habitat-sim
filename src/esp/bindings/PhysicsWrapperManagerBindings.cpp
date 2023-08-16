// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include "esp/physics/bullet/objectWrappers/ManagedBulletArticulatedObject.h"
#include "esp/physics/bullet/objectWrappers/ManagedBulletRigidObject.h"
#include "esp/physics/objectManagers/ArticulatedObjectManager.h"
#include "esp/physics/objectManagers/PhysicsObjectBaseManager.h"
#include "esp/physics/objectManagers/RigidBaseManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace PhysWraps = esp::physics;
using PhysWraps::ArticulatedObjectManager;
using PhysWraps::ManagedBulletArticulatedObject;
using PhysWraps::ManagedBulletRigidObject;
using PhysWraps::ManagedRigidObject;
using PhysWraps::PhysicsObjectBaseManager;
using PhysWraps::RigidBaseManager;
using PhysWraps::RigidObjectManager;

namespace esp {
namespace physics {
/**
 * @brief instance class template base classes for object wrapper managers.
 * @tparam The type used to specialize class template for each object wrapper
 * manager. Will be a type of a wrapper.
 * @param m pybind module reference.
 * @param objType string describing the managed object, for help text.
 * @param classStrPrefix string prefix for python class name specification.
 */

template <typename T, typename U>
void declareBaseWrapperManager(py::module& m,
                               const std::string& objType,
                               const std::string& classStrPrefix) {
  using MgrClass = PhysicsObjectBaseManager<T>;
  using WrapperPtr = std::shared_ptr<U>;
  // Most, but not all, of these methods are from ManagedContainer class
  // template.  However, we use PHysicsObjectBaseManager as the base class
  // because we wish to have appropriate (wrapper-related) access, argument
  // nomenclature and documentation.
  std::string pyclass_name =
      classStrPrefix + std::string("_PhysWrapperManager");
  py::class_<MgrClass, std::shared_ptr<MgrClass>>(m, pyclass_name.c_str())
      .def("get_object_handle_by_id", &MgrClass::getObjectHandleByID,
           ("Returns string handle for the " + objType +
            " corresponding to passed ID.")
               .c_str(),
           "object_id"_a)
      .def(
          "get_object_id_by_handle",
          py::overload_cast<const std::string&>(&MgrClass::getObjectIDByHandle),
          ("Returns integer ID for the " + objType + " with the passed handle.")
              .c_str(),
          "handle"_a)
      .def("get_object_handles",
           static_cast<std::vector<std::string> (MgrClass::*)(
               const std::string&, bool, bool) const>(
               &MgrClass::getObjectHandlesBySubstring),
           ("Returns a potentially sorted list of " + objType +
            " handles that either contain or explicitly do not contain the "
            "passed search_str, based on the value of boolean contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true, "sorted"_a = true)
      .def("get_objects_info", &MgrClass::getObjectInfoStrings,
           ("Returns a list of CSV strings describing each " + objType +
            " whose handles either contain or explicitly do not contain the "
            "passed search_str, based on the value of boolean contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)

      .def("get_objects_CSV_info", &MgrClass::getObjectInfoCSVString,
           ("Returns a comma-separated string describing each " + objType +
            " whose handles either contain or explicitly do not "
            "contain the passed search_str, based on the value of boolean "
            "contains.  Each " +
            objType + "'s info is separated by a newline.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)

      .def("get_num_objects", &MgrClass::getNumObjects,
           ("Returns the number of existing " + objType + "s being managed.")
               .c_str())
      .def("get_random_object_handle", &MgrClass::getRandomObjectHandle,
           ("Returns the handle for a random " + objType +
            " chosen from the existing " + objType + " being managed.")
               .c_str())
      .def("get_undeletable_handles", &MgrClass::getUndeletableObjectHandles,
           ("Returns a list of  " + objType + " handles for " + objType +
            "s that have been marked undeletable by the system. These " +
            objType + "s can still be modified.")
               .c_str())
      .def("get_user_locked_handles", &MgrClass::getUserLockedObjectHandles,
           ("Returns a list of handles for " + objType +
            "s that have been marked locked by the user. These will be "
            "undeletable until unlocked by the user. These " +
            objType + " can still be modified.")
               .c_str())
      .def("get_library_has_handle", &MgrClass::getObjectLibHasHandle,
           ("Returns whether the passed handle describes an existing " +
            objType + " in the library.")
               .c_str(),
           "handle"_a)
      .def("get_library_has_id", &MgrClass::getObjectLibHasID,
           ("Returns whether the passed object ID describes an existing " +
            objType + " in the library.")
               .c_str(),
           "object_id"_a)
      .def("set_object_lock", &MgrClass::setLock,
           ("This sets the lock state for the  " + objType +
            " that has the passed name. Lock == True makes the  " + objType +
            " unable to be deleted. Note : Locked  " + objType +
            "s can still be modified.")
               .c_str(),
           "handle"_a, "lock"_a)
      .def("set_lock_by_substring", &MgrClass::setLockBySubstring,
           ("This sets the lock state for all  " + objType +
            "s whose handles either contain or explicitly do not contain the "
            "passed search_str. Returns a list of handles for  " +
            objType +
            "s locked by this function call. Lock == True makes the  " +
            objType + " unable to be deleted. Note : Locked  " + objType +
            "s can still be modified.")
               .c_str(),
           "lock"_a, "search_str"_a = "", "contains"_a = true)
      .def("set_object_list_lock", &MgrClass::setLockByHandles,
           ("This sets the lock state for all  " + objType +
            "s whose handles are passed in list. Returns a list of handles "
            "for " +
            objType +
            "s locked by this function call. Lock == True makes the  " +
            objType + " unable to be deleted. Note : Locked  " + objType +
            "s can still be modified.")
               .c_str(),
           "handles"_a, "lock"_a)
      .def("remove_all_objects", &MgrClass::removeAllObjects,
           ("This removes a list of all the  " + objType +
            "s referenced in the library that have not been marked undeletable "
            "by the system or read-only by the user.")
               .c_str())
      .def("remove_objects_by_str", &MgrClass::removeObjectsBySubstring,
           ("This removes a list of all the  " + objType +
            "s referenced in the library that have not been marked undeletable "
            "by the system or read-only by the user and whose handles either "
            "contain or explicitly do not contain the passed search_str.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)
      .def("remove_object_by_id", &MgrClass::removeObjectByID,
           ("This removes the " + objType +
            " referenced by the passed ID from the library.")
               .c_str(),
           "object_id"_a)
      .def("remove_object_by_handle", &MgrClass::removeObjectByHandle,
           ("This removes the " + objType +
            " referenced by the passed handle from the library.")
               .c_str(),
           "handle"_a)
      .def(
          "get_object_by_id",
          static_cast<WrapperPtr (MgrClass::*)(int)>(
              &MgrClass::template getObjectOrCopyByID<U>),
          ("This returns a copy of the  " + objType +
           " specified by the passed ID if it exists, and NULL if it does not.")
              .c_str(),
          "object_id"_a)
      .def("get_object_by_handle",
           static_cast<WrapperPtr (MgrClass::*)(const std::string&)>(
               &MgrClass::template getObjectOrCopyByHandle<U>),
           ("This returns a copy of the  " + objType +
            " specified by the passed handle if it exists, and NULL if it does "
            "not.")
               .c_str(),
           "handle"_a)
      .def("get_objects_by_handle_substring",
           static_cast<std::unordered_map<std::string, WrapperPtr> (
               MgrClass::*)(const std::string&, bool)>(
               &MgrClass::template getObjectsByHandleSubstring<U>),
           ("Returns a dictionary of " + objType +
            " objects, keyed by their handles, for all handles that either "
            "contain or explicitly do not contain the passed search_str, based "
            "on the value of boolean contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true);
}  // declareBaseWrapperManager

template <typename T>
void declareRigidBaseWrapperManager(py::module& m,
                                    CORRADE_UNUSED const std::string& objType,
                                    const std::string& classStrPrefix) {
  using MgrClass = RigidBaseManager<T>;
  std::string pyclass_name =
      classStrPrefix + std::string("_RigidBaseWrapperManager");

  py::class_<MgrClass, PhysicsObjectBaseManager<T>, std::shared_ptr<MgrClass>>(
      m, pyclass_name.c_str());

}  //

void initPhysicsWrapperManagerBindings(pybind11::module& m) {
#ifdef ESP_BUILD_WITH_BULLET
  declareBaseWrapperManager<ManagedRigidObject, ManagedBulletRigidObject>(
      m, "BulletRigidObject", "BulletRigidObject");

  declareRigidBaseWrapperManager<ManagedRigidObject>(m, "BulletRigidObject",
                                                     "BulletRigidObject");

#else
  // if dynamics library not being used, just use base rigid object
  declareBaseWrapperManager<ManagedRigidObject, ManagedRigidObject>(
      m, "RigidObject", "RigidObject");

  declareRigidBaseWrapperManager<ManagedRigidObject>(m, "RigidObject",
                                                     "RigidObject");
#endif
  // RigidObject wrapper manager
  py::class_<RigidObjectManager, RigidBaseManager<ManagedRigidObject>,
             std::shared_ptr<RigidObjectManager>>(m, "RigidObjectManager")
      .def(
          "add_object_by_template_id",
#ifdef ESP_BUILD_WITH_BULLET
          &RigidObjectManager::addBulletObjectByID,
#else
          &RigidObjectManager::addObjectByID,
#endif
          "object_lib_id"_a, "attachment_node"_a = nullptr,
          "light_setup_key"_a = DEFAULT_LIGHTING_KEY,
          R"(Instance an object into the scene via a template referenced by library id.
          Optionally attach the object to an existing SceneNode and assign its initial
          LightSetup key. Returns a reference to the created object.)")
      .def(
          "add_object_by_template_handle",
#ifdef ESP_BUILD_WITH_BULLET
          &RigidObjectManager::addBulletObjectByHandle,
#else
          &RigidObjectManager::addObjectByHandle,
#endif
          "object_lib_handle"_a, "attachment_node"_a = nullptr,
          "light_setup_key"_a = DEFAULT_LIGHTING_KEY,
          R"(Instance an object into the scene via a template referenced by its handle.
          Optionally attach the object to an existing SceneNode and assign its initial
          LightSetup key. Returns a reference to the created object.)")
      .def(
          "remove_object_by_id", &RigidObjectManager::removePhysObjectByID,
          "object_id"_a, "delete_object_node"_a = true,
          "delete_visual_node"_a = true,
          R"(This removes the RigidObject referenced by the passed ID from the library, while allowing for "
          "the optional retention of the object's scene node and/or the visual node)")
      .def(
          "remove_object_by_handle",
          &RigidObjectManager::removePhysObjectByHandle, "handle"_a,
          "delete_object_node"_a = true, "delete_visual_node"_a = true,
          R"(This removes the RigidObject referenced by the passed handle from the library, while allowing "
          "for the optional retention of the object's scene node and/or the visual node)");

  // initialize bindings for articulated objects

#ifdef ESP_BUILD_WITH_BULLET
  declareBaseWrapperManager<ManagedArticulatedObject,
                            ManagedBulletArticulatedObject>(
      m, "BulletArticulatedObject", "BulletArticulatedObject");

#else
  // if dynamics library not being used, just use base rigid object
  declareBaseWrapperManager<ManagedArticulatedObject, ManagedArticulatedObject>(
      m, "ArticulatedObject", "ArticulatedObject");

#endif
  py::class_<ArticulatedObjectManager,
             PhysicsObjectBaseManager<ManagedArticulatedObject>,
             std::shared_ptr<ArticulatedObjectManager>>(
      m, "ArticulatedObjectManager")

      .def(
          "add_articulated_object_by_template_handle",
#ifdef ESP_BUILD_WITH_BULLET
          &ArticulatedObjectManager::addBulletArticulatedObjectByHandle,
#else
          &ArticulatedObjectManager::addArticulatedObjectByHandle,
#endif
          "ao_lib_handle"_a, "force_reload"_a = false,
          "light_setup_key"_a = DEFAULT_LIGHTING_KEY,
          R"(Instance an articulated object into the scene via a template referenced by its handle.
          Optionally force the articulated object's model to be reloaded from disk and assign its initial
          LightSetup key. Returns a reference to the created object.)")
      .def(
          "add_articulated_object_by_template_id",
#ifdef ESP_BUILD_WITH_BULLET
          &ArticulatedObjectManager::addBulletArticulatedObjectByID,
#else
          &ArticulatedObjectManager::addArticulatedObjectByID,
#endif
          "ao_lib_id"_a, "force_reload"_a = false,
          "light_setup_key"_a = DEFAULT_LIGHTING_KEY,
          R"(Instance an articulated object into the scene via a template referenced by its ID.
          Optionally force the articulated object's model to be reloaded from disk and assign its initial
          LightSetup key. Returns a reference to the created object.)")

      .def(
          "add_articulated_object_from_urdf",
#ifdef ESP_BUILD_WITH_BULLET
          &ArticulatedObjectManager::addBulletArticulatedObjectFromURDF,
#else
          &ArticulatedObjectManager::addArticulatedObjectFromURDF,
#endif
          "filepath"_a, "fixed_base"_a = false, "global_scale"_a = 1.0,
          "mass_scale"_a = 1.0, "force_reload"_a = false,
          "maintain_link_order"_a = false, "intertia_from_urdf"_a = false,
          "light_setup_key"_a = DEFAULT_LIGHTING_KEY,
          R"(Load and parse a URDF file using the given 'filepath' into a model,
          then use this model to instantiate an Articulated Object in the world.
          Returns a reference to the created object.)");
}  // initPhysicsWrapperManagerBindings

}  // namespace physics
}  // namespace esp
