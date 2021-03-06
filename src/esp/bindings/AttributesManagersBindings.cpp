
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

#include "esp/metadata/attributes/LightLayoutAttributes.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/LightLayoutAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace py = pybind11;
using py::literals::operator""_a;
namespace Attrs = esp::metadata::attributes;
using Attrs::AbstractAttributes;
using Attrs::AbstractObjectAttributes;
using Attrs::AbstractPrimitiveAttributes;
using Attrs::CapsulePrimitiveAttributes;
using Attrs::ConePrimitiveAttributes;
using Attrs::CubePrimitiveAttributes;
using Attrs::CylinderPrimitiveAttributes;
using Attrs::IcospherePrimitiveAttributes;
using Attrs::LightLayoutAttributes;
using Attrs::ObjectAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::StageAttributes;
using Attrs::UVSpherePrimitiveAttributes;

namespace esp {
namespace metadata {
namespace managers {

/**
 * @brief instance class template base classes for attributes managers.
 * @tparam The type used to specialize class template for each attributes
 * manager. Will be an attributes instance
 * @param m pybind module reference.
 * @param classStrPrefix string prefix for python class name specification.
 */

template <class T, core::ManagedObjectAccess Access>
void declareBaseAttributesManager(py::module& m,
                                  const std::string& classStrPrefix) {
  using MgrClass = AttributesManager<T, Access>;
  using AttribsPtr = std::shared_ptr<T>;
  // Most, but not all, of these methods are from ManagedContainer class
  // template.  However, we use AttributesManager as the base class because we
  // wish to have appropriate (attributes-related) argument nomenclature and
  // documentation.
  std::string pyclass_name = classStrPrefix + std::string("AttributesManager");
  py::class_<MgrClass, std::shared_ptr<MgrClass>>(m, pyclass_name.c_str())
      .def(
          "get_template_handle_by_ID", &MgrClass::getObjectHandleByID,
          R"(Returns string handle for the template corresponding to passed ID.)",
          "ID"_a)
      .def(
          "get_template_ID_by_handle",
          py::overload_cast<const std::string&>(&MgrClass::getObjectIDByHandle),
          R"(Returns integer ID for the template with the passed handle.)",
          "handle"_a)
      .def(
          "get_template_handles",
          static_cast<std::vector<std::string> (MgrClass::*)(const std::string&,
                                                             bool) const>(
              &MgrClass::getObjectHandlesBySubstring),
          R"(Returns a list of template handles that either contain or explicitly do not
            contain the passed search_str, based on the value of boolean contains.)",
          "search_str"_a = "", "contains"_a = true)
      .def(
          "load_configs",
          static_cast<std::vector<int> (MgrClass::*)(const std::string&, bool)>(
              &MgrClass::loadAllConfigsFromPath),
          R"(Build templates for all JSON files with appropriate extension
            that exist in the provided file or directory path. If save_as_defaults
            is true, then these templates will be unable to be deleted)",
          "path"_a, "save_as_defaults"_a = false)
      .def("create_template",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&, bool)>(
               &MgrClass::createObject),
           R"(Creates a template based on passed handle, and registers it in
            the library if register_template is True.)",
           "handle"_a, "register_template"_a = true)
      .def("create_new_template",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&, bool)>(
               &MgrClass::createDefaultObject),
           R"(Creates a template built with default values, and registers it in
            the library if register_template is True.)",
           "handle"_a, "register_template"_a = false)
      .def("is_valid_filename", &MgrClass::isValidFileName, R"(
             Returns whether the passed handle exists and the user has access.)",
           "handle"_a)
      .def("get_num_templates", &MgrClass::getNumObjects, R"(
             Returns the number of existing templates being managed.)")
      .def("get_random_template_handle", &MgrClass::getRandomObjectHandle,
           R"(Returns the handle for a random template chosen from the
             existing templates being managed.)")
      .def(
          "get_undeletable_handles", &MgrClass::getUndeletableObjectHandles,
          R"(Returns a list of template handles for templates that have been marked
            undeletable by the system. These templates can still be edited.)")
      .def(
          "get_user_locked_handles", &MgrClass::getUserLockedObjectHandles,
          R"(Returns a list of template handles for templates that have been marked
            locked by the user. These will be undeletable until unlocked by the user.
            These templates can still be edited.)")
      .def("get_library_has_handle", &MgrClass::getObjectLibHasHandle,
           R"(Returns whether the passed handle describes an existing template
             in the library.)",
           "handle"_a)
      .def(
          "set_template_lock", &MgrClass::setLock,
          R"(This sets the lock state for the template that has the passed name.
             Lock == True makes the template unable to be deleted.
             Note : Locked templates can still be edited.)",
          "handle"_a, "lock"_a)
      .def("set_lock_by_substring", &MgrClass::setLockBySubstring,
           R"(This sets the lock state for all templates whose handles either
             contain or explictly do not contain the passed search_str.
             Returns a list of handles for templates locked by this function
             call. Lock == True makes the template unable to be deleted.
             Note : Locked templates can still be edited.)",
           "lock"_a, "search_str"_a = "", "contains"_a = true)
      .def("set_template_list_lock", &MgrClass::setLockByHandles,
           R"(This sets the lock state for all templates whose handles
             are passed in list. Returns a list of handles for templates
             locked by this function call. Lock == True makes the template unable
             to be deleted. Note : Locked templates can still be edited.)",
           "handles"_a, "lock"_a)
      .def("remove_all_templates", &MgrClass::removeAllObjects,
           R"(This removes, and returns, a list of all the templates referenced
             in the library that have not been marked undeletable by the system
             or read-only by the user.)")
      .def("remove_templates_by_str", &MgrClass::removeObjectsBySubstring,
           R"(This removes, and returns, a list of all the templates referenced
             in the library that have not been marked undeletable by the system
             or read-only by the user and whose handles either contain or explictly
             do not contain the passed search_str.)",
           "search_str"_a = "", "contains"_a = true)
      .def("remove_template_by_ID", &MgrClass::removeObjectByID,
           R"(This removes, and returns the template referenced by the passed ID
             from the library.)",
           "ID"_a)
      .def(
          "remove_template_by_handle", &MgrClass::removeObjectByHandle,
          R"(This removes, and returns the template referenced by the passed handle
             from the library.)",
          "handle"_a)
      .def("register_template", &MgrClass::registerObject,
           R"(This registers a copy of the passed template in the library, and
             returns the template's integer ID.)",
           "template"_a, "specified_handle"_a = "",
           "force_registration"_a = false)
      .def("get_template_by_ID",
           static_cast<AttribsPtr (MgrClass::*)(int)>(
               &MgrClass::getObjectOrCopyByID),
           R"(This returns a copy of the template specified by the passed
             ID if it exists, and NULL if it does not.)",
           "ID"_a)
      .def("get_template_by_handle",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&)>(
               &MgrClass::getObjectOrCopyByHandle),
           R"(This returns a copy of the template specified by the passed
             handle if it exists, and NULL if it does not.)",
           "handle"_a);
}  // declareBaseAttributesManager

void initAttributesManagersBindings(py::module& m) {
  // ==== PrimObjTypes enum describing types of primitives supported ====
  py::enum_<metadata::PrimObjTypes>(m, "PrimObjTypes")
      .value("CAPSULE_SOLID", metadata::PrimObjTypes::CAPSULE_SOLID)
      .value("CAPSULE_WF", metadata::PrimObjTypes::CAPSULE_WF)
      .value("CONE_SOLID", metadata::PrimObjTypes::CONE_SOLID)
      .value("CONE_WF", metadata::PrimObjTypes::CONE_WF)
      .value("CUBE_SOLID", metadata::PrimObjTypes::CUBE_SOLID)
      .value("CUBE_WF", metadata::PrimObjTypes::CUBE_WF)
      .value("CYLINDER_SOLID", metadata::PrimObjTypes::CYLINDER_SOLID)
      .value("CYLINDER_WF", metadata::PrimObjTypes::CYLINDER_WF)
      .value("ICOSPHERE_SOLID", metadata::PrimObjTypes::ICOSPHERE_SOLID)
      .value("ICOSPHERE_WF", metadata::PrimObjTypes::ICOSPHERE_WF)
      .value("UVSPHERE_SOLID", metadata::PrimObjTypes::UVSPHERE_SOLID)
      .value("UVSPHERE_WF", metadata::PrimObjTypes::UVSPHERE_WF)
      .value("END_PRIM_OBJ_TYPE", metadata::PrimObjTypes::END_PRIM_OBJ_TYPES);

  // ==== Primitive Asset Attributes Template manager ====
  declareBaseAttributesManager<AbstractPrimitiveAttributes,
                               core::ManagedObjectAccess::Copy>(m, "BaseAsset");
  py::class_<AssetAttributesManager,
             AttributesManager<AbstractPrimitiveAttributes,
                               core::ManagedObjectAccess::Copy>,
             AssetAttributesManager::ptr>(m, "AssetAttributesManager")
      // AssetAttributesMangaer-specific bindings
      // return appropriately cast capsule templates
      .def("get_default_capsule_template",
           &AssetAttributesManager::getDefaultCapsuleTemplate,
           R"(This returns an appropriately cast copy of the default Capsule
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_capsule_template", &AssetAttributesManager::getCapsuleTemplate,
           R"(This returns an appropriately cast copy of the Capsule primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a)
      // return appropriately cast cone templates
      .def("get_default_cone_template",
           &AssetAttributesManager::getDefaultConeTemplate,
           R"(This returns an appropriately cast copy of the default Cone
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_cone_template", &AssetAttributesManager::getConeTemplate,
           R"(This returns an appropriately cast copy of the Cone primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a)
      // return appropriately cast cube templates
      .def("get_default_cube_template",
           &AssetAttributesManager::getDefaultCubeTemplate,
           R"(This returns an appropriately cast copy of the default Cube
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_cube_template", &AssetAttributesManager::getCubeTemplate,
           R"(This returns an appropriately cast copy of the Cube primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a)
      // return appropriately cast cylinder templates
      .def("get_default_cylinder_template",
           &AssetAttributesManager::getDefaultCylinderTemplate,
           R"(This returns an appropriately cast copy of the default Cylinder
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_cylinder_template",
           &AssetAttributesManager::getCylinderTemplate,
           R"(This returns an appropriately cast copy of the Cylinder primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a)
      // return appropriately cast icosphere templates
      .def("get_default_icosphere_template",
           &AssetAttributesManager::getDefaultIcosphereTemplate,
           R"(This returns an appropriately cast copy of the default Icosphere
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_icosphere_template",
           &AssetAttributesManager::getIcosphereTemplate,
           R"(This returns an appropriately cast copy of the Icosphere primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a)
      // return appropriately cast UVSphere templates
      .def("get_default_UVsphere_template",
           &AssetAttributesManager::getDefaultUVSphereTemplate,
           R"(This returns an appropriately cast copy of the default UVSphere
             primitive template in the library, either solid or wireframe
             based on is_wireframe.)",
           "is_wireframe"_a)
      .def("get_UVsphere_template",
           &AssetAttributesManager::getUVSphereTemplate,
           R"(This returns an appropriately cast copy of the UVSphere primitive
             template in the library that is referenced by the passed handle, or
             NULL if none exists.)",
           "handle"_a);

  // ==== Light Layout Attributes Template manager ====
  declareBaseAttributesManager<LightLayoutAttributes,
                               core::ManagedObjectAccess::Copy>(
      m, "BaseLightLayout");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      LightLayoutAttributesManager,
      AttributesManager<LightLayoutAttributes, core::ManagedObjectAccess::Copy>,
      LightLayoutAttributesManager::ptr>(m, "LightLayoutAttributesManager");
  // ==== Object Attributes Template manager ====
  declareBaseAttributesManager<ObjectAttributes,
                               core::ManagedObjectAccess::Copy>(m,
                                                                "BaseObject");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      ObjectAttributesManager,
      AttributesManager<ObjectAttributes, core::ManagedObjectAccess::Copy>,
      ObjectAttributesManager::ptr>(m, "ObjectAttributesManager")

      // ObjectAttributesManager-specific bindings
      .def("load_object_configs",
           &ObjectAttributesManager::loadAllConfigsFromPath,
           R"(DEPRECATED : use "load_configs" instead.
            Build templates for all files with ".object_config.json" extension
            that exist in the provided file or directory path. If save_as_defaults
            is true, then these templates will be unable to be deleted)",
           "path"_a, "save_as_defaults"_a = false)

      // manage file-based templates access
      .def(
          "get_num_file_templates",
          &ObjectAttributesManager::getNumFileTemplateObjects,
          R"(Returns the number of existing file-based templates being managed.)")
      .def(
          "get_file_template_handles",
          static_cast<std::vector<std::string> (ObjectAttributesManager::*)(
              const std::string&, bool) const>(
              &ObjectAttributesManager::getFileTemplateHandlesBySubstring),
          R"(Returns a list of file-based template handles that either contain or
          explicitly do not contain the passed search_str, based on the value of
          contains.)",
          "search_str"_a = "", "contains"_a = true)
      .def(
          "get_random_file_template_handle",
          &ObjectAttributesManager::getRandomFileTemplateHandle,
          R"(Returns the handle for a random file-based template chosen from the
             existing templates being managed.)")

      // manage synthesized/primitive asset-based templates access
      .def("get_num_synth_templates",
           &ObjectAttributesManager::getNumSynthTemplateObjects, R"(
             Returns the number of existing synthesized(primitive asset)-based
             templates being managed.)")
      .def(
          "get_synth_template_handles",
          static_cast<std::vector<std::string> (ObjectAttributesManager::*)(
              const std::string&, bool) const>(
              &ObjectAttributesManager::getSynthTemplateHandlesBySubstring),
          R"(Returns a list of synthesized(primitive asset)-based template handles
            that either contain or explicitly do not contain the passed search_str,
            based on the value of contains.)",
          "search_str"_a = "", "contains"_a = true)
      .def("get_random_synth_template_handle",
           &ObjectAttributesManager::getRandomSynthTemplateHandle,
           R"(Returns the handle for a random synthesized(primitive asset)-based
          template chosen from the existing templates being managed.)");

  // ==== Stage Attributes Template manager ====
  declareBaseAttributesManager<StageAttributes,
                               core::ManagedObjectAccess::Copy>(m, "BaseStage");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      StageAttributesManager,
      AttributesManager<StageAttributes, core::ManagedObjectAccess::Copy>,
      StageAttributesManager::ptr>(m, "StageAttributesManager");

  // ==== Physics World/Manager Template manager ====

  declareBaseAttributesManager<PhysicsManagerAttributes,
                               core::ManagedObjectAccess::Copy>(m,
                                                                "BasePhysics");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<PhysicsAttributesManager,
             AttributesManager<PhysicsManagerAttributes,
                               core::ManagedObjectAccess::Copy>,
             PhysicsAttributesManager::ptr>(m, "PhysicsAttributesManager");

}  // initAttributesManagersBindings
}  // namespace managers
}  // namespace metadata
}  // namespace esp
