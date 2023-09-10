
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

#include "esp/metadata/attributes/AbstractObjectAttributes.h"
#include "esp/metadata/attributes/LightLayoutAttributes.h"
#include "esp/metadata/attributes/ObjectAttributes.h"
#include "esp/metadata/attributes/StageAttributes.h"

#include "esp/metadata/managers/AOAttributesManager.h"
#include "esp/metadata/managers/AssetAttributesManager.h"
#include "esp/metadata/managers/AttributesManagerBase.h"
#include "esp/metadata/managers/LightLayoutAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PbrShaderAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/StageAttributesManager.h"

namespace py = pybind11;
using py::literals::operator""_a;
namespace Attrs = esp::metadata::attributes;
using Attrs::AbstractAttributes;
using Attrs::AbstractObjectAttributes;
using Attrs::AbstractPrimitiveAttributes;
using Attrs::ArticulatedObjectAttributes;
using Attrs::LightLayoutAttributes;
using Attrs::ObjectAttributes;
using Attrs::PbrShaderAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::StageAttributes;
using esp::core::managedContainers::ManagedObjectAccess;

namespace esp {
namespace metadata {
namespace managers {

/**
 * @brief instance class template base classes for attributes managers.
 * @tparam The type used to specialize class template for each attributes
 * manager. Will be an attributes instance
 * @param m pybind module reference.
 * @param attrType type of attributes being managed, for better help docs
 * @param classStrPrefix string prefix for python class name specification.
 */

template <class T, ManagedObjectAccess Access>
void declareBaseAttributesManager(py::module& m,
                                  const std::string& attrType,
                                  const std::string& classStrPrefix) {
  using MgrClass = AttributesManager<T, Access>;
  using AttribsPtr = std::shared_ptr<T>;
  // Most, but not all, of these methods are from ManagedContainer class
  // template.  However, we use AttributesManager as the base class because we
  // wish to have appropriate (attributes-related) argument nomenclature and
  // documentation.
  std::string pyclass_name = classStrPrefix + std::string("AttributesManager");
  py::class_<MgrClass, std::shared_ptr<MgrClass>>(m, pyclass_name.c_str())
      .def("get_template_handle_by_id", &MgrClass::getObjectHandleByID,
           ("Returns string handle for the " + attrType +
            " template corresponding to passed ID.")
               .c_str(),
           "template_id"_a)
      .def(
          "get_template_id_by_handle",
          py::overload_cast<const std::string&>(&MgrClass::getObjectIDByHandle),
          ("Returns integer ID for the " + attrType +
           " template with the passed handle.")
              .c_str(),
          "handle"_a)
      .def("get_template_handles",
           static_cast<std::vector<std::string> (MgrClass::*)(
               const std::string&, bool, bool) const>(
               &MgrClass::getObjectHandlesBySubstring),
           ("Returns a potentially sorted list of " + attrType +
            " template handles that either contain or "
            "explicitly do not contain the passed search_str, based on the "
            "value of boolean contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true, "sorted"_a = true)
      .def("get_templates_info", &MgrClass::getObjectInfoStrings,
           ("Returns a list of CSV strings describing each " + attrType +
            " template whose handles either contain or explicitly do not "
            "contain the passed search_str, based on the value of boolean "
            "contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)
      .def("get_templates_CSV_info", &MgrClass::getObjectInfoCSVString,
           ("Returns a comma-separated string describing each " + attrType +
            " template whose handles either contain or explicitly do not "
            "contain the passed search_str, based on the value of boolean "
            "contains.  Each template's info is separated by a newline.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)
      .def(
          "load_configs",
          static_cast<std::vector<int> (MgrClass::*)(const std::string&, bool)>(
              &MgrClass::loadAllJSONConfigsFromPath),
          ("Build " + attrType +
           " templates for all JSON files with appropriate extension "
           "that exist in the provided file or directory path. If "
           "save_as_defaults is true, then these " +
           attrType + " templates will be unable to be deleted")
              .c_str(),
          "path"_a, "save_as_defaults"_a = false)
      .def("create_template",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&, bool)>(
               &MgrClass::createObject),
           ("Creates a " + attrType +
            " template based on passed handle, and registers it "
            "in the library if register_template is True.")
               .c_str(),
           "handle"_a, "register_template"_a = true)
      .def("create_new_template",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&, bool)>(
               &MgrClass::createDefaultObject),
           ("Creates a " + attrType +
            " template built with default values, and registers "
            "it in the library if register_template is True.")
               .c_str(),
           "handle"_a, "register_template"_a = false)
      .def(
          "is_valid_filename",
          [](CORRADE_UNUSED MgrClass& self, const std::string& filename) {
            return Corrade::Utility::Path::exists(filename);
          },
          R"(Returns whether the passed handle is a valid, existing file.)",
          "handle"_a)
      .def("get_num_templates", &MgrClass::getNumObjects,
           ("Returns the number of existing " + attrType +
            " templates being managed.")
               .c_str())
      .def("get_random_template_handle", &MgrClass::getRandomObjectHandle,
           ("Returns the handle for a random " + attrType +
            " template chosen"
            " from the existing " +
            attrType + " templates being managed.")
               .c_str())
      .def("get_undeletable_handles", &MgrClass::getUndeletableObjectHandles,
           ("Returns a list of " + attrType + " template handles for " +
            attrType +
            " templates that have been marked undeletable by the system. "
            "These " +
            attrType + " templates can still be edited.")
               .c_str())
      .def(
          "get_user_locked_handles", &MgrClass::getUserLockedObjectHandles,
          ("Returns a list of " + attrType + " template handles for " +
           attrType +
           " templates that have been marked locked by the user. These will be "
           "undeletable until unlocked by the user. These " +
           attrType + " templates can still be edited.")
              .c_str())
      .def("get_library_has_handle", &MgrClass::getObjectLibHasHandle,
           ("Returns whether the passed handle describes an existing " +
            attrType + " template in the library.")
               .c_str(),
           "handle"_a)
      .def("get_library_has_id", &MgrClass::getObjectLibHasID,
           ("Returns whether the passed template ID describes an existing " +
            attrType + " template in the library.")
               .c_str(),
           "template_id"_a)
      .def("set_template_lock", &MgrClass::setLock,
           ("This sets the lock state for the " + attrType +
            " template that has the passed name. Lock == True makes the " +
            attrType + " template unable to be deleted.  Note : Locked " +
            attrType + " templates can still be edited.")
               .c_str(),
           "handle"_a, "lock"_a)
      .def("set_lock_by_substring", &MgrClass::setLockBySubstring,
           ("This sets the lock state for all " + attrType +
            " templates whose handles either contain or explicitly do not "
            "contain the passed search_str. Returns a list of handles for " +
            attrType +
            " templates locked by this function call. Lock == True makes the " +
            attrType + " template unable to be deleted. Note : Locked " +
            attrType + " templates can still be edited.")
               .c_str(),
           "lock"_a, "search_str"_a = "", "contains"_a = true)
      .def("set_template_list_lock", &MgrClass::setLockByHandles,
           ("This sets the lock state for all " + attrType +
            " templates whose handles are passed "
            "in list. Returns a list of handles for templates locked by this "
            "function call. Lock == True makes the " +
            attrType + " template unable to be deleted. Note : Locked " +
            attrType + " templates can still be edited.")
               .c_str(),
           "handles"_a, "lock"_a)
      .def("remove_all_templates", &MgrClass::removeAllObjects,
           ("This removes, and returns, a list of all the " + attrType +
            " templates referenced in the library that have not been marked "
            "undeletable by the system or read-only by the user.")
               .c_str())
      .def("remove_templates_by_str", &MgrClass::removeObjectsBySubstring,
           ("This removes, and returns, a list of all the " + attrType +
            " templates referenced in the library that have not been marked "
            "undeletable by the system or read-only by the user and whose "
            "handles either contain or explicitly do not contain the passed "
            "search_str.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true)
      .def("remove_template_by_id", &MgrClass::removeObjectByID,
           ("This removes, and returns the " + attrType +
            " template referenced by the passed ID from the library.")
               .c_str(),
           "template_id"_a)
      .def("remove_template_by_handle", &MgrClass::removeObjectByHandle,
           ("This removes, and returns the " + attrType +
            " template referenced by the passed handle from the library.")
               .c_str(),
           "handle"_a)
      .def("register_template", &MgrClass::registerObject,
           ("This registers a copy of the passed " + attrType +
            " template in the library, and returns the template's integer ID.")
               .c_str(),
           "template"_a, "specified_handle"_a = "",
           "force_registration"_a = false)
      .def("get_template_by_id",
           static_cast<AttribsPtr (MgrClass::*)(int)>(
               &MgrClass::getObjectOrCopyByID),
           ("This returns a copy of the " + attrType +
            " template specified by the passed ID if it exists, and NULL if it "
            "does not.")
               .c_str(),
           "template_id"_a)
      .def("get_template_by_handle",
           static_cast<AttribsPtr (MgrClass::*)(const std::string&)>(
               &MgrClass::getObjectOrCopyByHandle),
           ("This returns a copy of the " + attrType +
            " template specified by the passed handle if it exists, and NULL "
            "if it does not.")
               .c_str(),
           "handle"_a)
      .def("get_templates_by_handle_substring",
           static_cast<std::unordered_map<std::string, AttribsPtr> (
               MgrClass::*)(const std::string&, bool)>(
               &MgrClass::getObjectsByHandleSubstring),
           ("Returns a dictionary of " + attrType +
            " templates, keyed by their handles, for all handles that either "
            "contain or explicitly do not contain the passed search_str, based "
            "on the value of boolean contains.")
               .c_str(),
           "search_str"_a = "", "contains"_a = true);
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
                               ManagedObjectAccess::Copy>(m, "Primitive Asset",
                                                          "BaseAsset");
  py::class_<
      AssetAttributesManager,
      AttributesManager<AbstractPrimitiveAttributes, ManagedObjectAccess::Copy>,
      AssetAttributesManager::ptr>(
      m, "AssetAttributesManager",
      R"(Manages PrimtiveAttributes objects which define parameters for constructing primitive mesh shapes such as cubes, capsules, cylinders, and cones.)")
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
                               ManagedObjectAccess::Copy>(m, "LightLayout",
                                                          "BaseLightLayout");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      LightLayoutAttributesManager,
      AttributesManager<LightLayoutAttributes, ManagedObjectAccess::Copy>,
      LightLayoutAttributesManager::ptr>(m, "LightLayoutAttributesManager");

  // ==== Articulated Object Attributes Template manager ====
  declareBaseAttributesManager<ArticulatedObjectAttributes,
                               ManagedObjectAccess::Copy>(
      m, "ArticulatedObjectAttributes", "BaseArticulatedObject");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      AOAttributesManager,
      AttributesManager<ArticulatedObjectAttributes, ManagedObjectAccess::Copy>,
      AOAttributesManager::ptr>(
      m, "AOAttributesManager",
      R"(Manages ArticulatedObjectAttributes which define Habitat-specific metadata for articulated objects
      (i.e. render asset or semantic ID), in addition to data held in defining URDF file, pre-instantiation.
      Can import .ao_config.json files.)");

  // ==== Object Attributes Template manager ====
  declareBaseAttributesManager<ObjectAttributes, ManagedObjectAccess::Copy>(
      m, "ObjectAttributes", "BaseObject");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<ObjectAttributesManager,
             AttributesManager<ObjectAttributes, ManagedObjectAccess::Copy>,
             ObjectAttributesManager::ptr>(
      m, "ObjectAttributesManager",
      R"(Manages ObjectAttributes which define metadata for rigid objects pre-instantiation.
      Can import .object_config.json files.)")

      // ObjectAttributesManager-specific bindings
      .def("load_object_configs",
           &ObjectAttributesManager::loadAllJSONConfigsFromPath,
           R"(DEPRECATED : use "load_configs" instead.
            Build ObjectAttributes templates for all files with ".object_config.json" extension
            that exist in the provided file or directory path. If save_as_defaults
            is true, then these ObjectAttributes templates will be unable to be deleted)",
           "path"_a, "save_as_defaults"_a = false)

      // manage file-based templates access
      .def(
          "get_num_file_templates",
          &ObjectAttributesManager::getNumFileTemplateObjects,
          R"(Returns the number of existing file-based ObjectAttributes templates being managed.)")
      .def(
          "get_file_template_handles",
          static_cast<std::vector<std::string> (ObjectAttributesManager::*)(
              const std::string&, bool, bool) const>(
              &ObjectAttributesManager::getFileTemplateHandlesBySubstring),
          R"(Returns a potentially sorted list of file-based ObjectAttributes template handles
          that either contain or explicitly do not contain the passed search_str, based on the value of
          contains.)",
          "search_str"_a = "", "contains"_a = true, "sorted"_a = true)
      .def(
          "get_random_file_template_handle",
          &ObjectAttributesManager::getRandomFileTemplateHandle,
          R"(Returns the handle for a random file-based template chosen from the
             existing ObjectAttributes templates being managed.)")

      // manage synthesized/primitive asset-based templates access
      .def("get_num_synth_templates",
           &ObjectAttributesManager::getNumSynthTemplateObjects, R"(
             Returns the number of existing synthesized(primitive asset)-based ObjectAttributes
             templates being managed.)")
      .def(
          "get_synth_template_handles",
          static_cast<std::vector<std::string> (ObjectAttributesManager::*)(
              const std::string&, bool, bool) const>(
              &ObjectAttributesManager::getSynthTemplateHandlesBySubstring),
          R"(Returns a potentially sorted list of synthesized(primitive asset)-based ObjectAttributes
            template handles that either contain or explicitly do not contain the passed search_str,
            based on the value of contains.)",
          "search_str"_a = "", "contains"_a = true, "sorted"_a = true)
      .def("get_random_synth_template_handle",
           &ObjectAttributesManager::getRandomSynthTemplateHandle,
           R"(Returns the handle for a random synthesized(primitive asset)-based
          template chosen from the existing ObjectAttributes templates being managed.)");

  // ==== Stage Attributes Template manager ====
  declareBaseAttributesManager<StageAttributes, ManagedObjectAccess::Copy>(
      m, "StageAttributes", "BaseStage");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<StageAttributesManager,
             AttributesManager<StageAttributes, ManagedObjectAccess::Copy>,
             StageAttributesManager::ptr>(
      m, "StageAttributesManager",
      R"(Manages StageAttributes which define metadata for stages (i.e. static background mesh such
      as architectural elements) pre-instantiation. Can import .stage_config.json files.)");

  // ==== Physics World/Manager Template manager ====

  declareBaseAttributesManager<PhysicsManagerAttributes,
                               ManagedObjectAccess::Copy>(
      m, "PhysicsAttributes", "BasePhysics");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<
      PhysicsAttributesManager,
      AttributesManager<PhysicsManagerAttributes, ManagedObjectAccess::Copy>,
      PhysicsAttributesManager::ptr>(
      m, "PhysicsAttributesManager",
      R"(Manages PhysicsManagerAttributes which define global Simulation parameters
      such as timestep. Can import .physics_config.json files.)");

  // ==== Pbr Shader configuration Template manager ====
  declareBaseAttributesManager<PbrShaderAttributes, ManagedObjectAccess::Copy>(
      m, "PbrShaderAttributes", "BasePbrConfig");
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<PbrShaderAttributesManager,
             AttributesManager<PbrShaderAttributes, ManagedObjectAccess::Copy>,
             PbrShaderAttributesManager::ptr>(
      m, "PbrShaderAttributesManager",
      R"(Manages PbrShaderAttributess which define PBR shader calculation control values, such as
      enabling IBL or specifying direct and indirect lighting balance. Can import .pbr_config.json files.)");

}  // initAttributesManagersBindings
}  // namespace managers
}  // namespace metadata
}  // namespace esp
