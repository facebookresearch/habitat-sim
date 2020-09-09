
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

#include "esp/assets/managers/AttributesManagerBase.h"

#include "esp/assets/managers/AssetAttributesManager.h"
#include "esp/assets/managers/ObjectAttributesManager.h"
#include "esp/assets/managers/PhysicsAttributesManager.h"
#include "esp/assets/managers/StageAttributesManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace assets {
namespace managers {

using attributes::AbstractPrimitiveAttributes;
using attributes::CapsulePrimitiveAttributes;
using attributes::ConePrimitiveAttributes;
using attributes::CubePrimitiveAttributes;
using attributes::CylinderPrimitiveAttributes;
using attributes::IcospherePrimitiveAttributes;
using attributes::ObjectAttributes;
using attributes::PhysicsManagerAttributes;
using attributes::StageAttributes;
using attributes::UVSpherePrimitiveAttributes;

/**
 * @brief instance class template base classes for attributes managers.
 * @tparam The type used to specialize class template for each attributes
 * manager. Will be a smart pointer to an attributes
 * @param m pybind module reference.
 * @param classStrPrefix string prefix for python class name specification.
 */

template <class T>
void declareBaseAttributesManager(py::module& m, std::string classStrPrefix) {
  using AttrClass = AttributesManager<T>;
  std::string pyclass_name = classStrPrefix + std::string("AttributesManager");
  py::class_<AttrClass, std::shared_ptr<AttrClass>>(m, pyclass_name.c_str())
      .def(
          "get_template_handle_by_ID", &AttrClass::getTemplateHandleByID,
          R"(Returns string handle for the template corresponding to passed ID.)",
          "ID"_a)
      .def("get_template_ID_by_handle",
           py::overload_cast<const std::string&>(
               &AttrClass::getTemplateIDByHandle),
           R"(Returns integer ID for the template with the passed handle.)",
           "handle"_a)
      .def(
          "get_template_handles",
          static_cast<std::vector<std::string> (AttrClass::*)(
              const std::string&, bool) const>(
              &AttrClass::getTemplateHandlesBySubstring),
          R"(Returns a list of template handles that either contain or explicitly do not
            contain the passed search_str, based on the value of boolean contains.)",
          "search_str"_a = "", "contains"_a = true)
      .def("create_template",
           static_cast<T (AttrClass::*)(const std::string&, bool)>(
               &AttrClass::createAttributesTemplate),
           R"(Creates a template based on passed handle, and registers it in
            the library if register_template is True.)",
           "handle"_a, "register_template"_a = true)
      .def("create_new_template",
           static_cast<T (AttrClass::*)(const std::string&, bool)>(
               &AttrClass::createDefaultAttributesTemplate),
           R"(Creates a template built with default values, and registers it in
            the library if register_template is True.)",
           "handle"_a, "register_template"_a = false)
      .def("is_valid_filename", &AttrClass::isValidFileName, R"(
             Returns whether the passed handle exists and the user has access.)",
           "handle"_a)
      .def("get_num_templates", &AttrClass::getNumTemplates, R"(
             Returns the number of existing templates being managed.)")
      .def("get_random_template_handle", &AttrClass::getRandomTemplateHandle,
           R"(Returns the handle for a random template chosen from the
             existing templates being managed.)")
      .def(
          "get_undeletable_handles", &AttrClass::getUndeletableTemplateHandles,
          R"(Returns a list of template handles for templates that have been marked
            undeletable by the system. These templates can still be edited.)")
      .def(
          "get_user_locked_handles", &AttrClass::getUserLockedTemplateHandles,
          R"(Returns a list of template handles for templates that have been marked
            locked by the user. These will be undeletable until unlocked by the user.
            These templates can still be edited.)")
      .def("get_library_has_handle", &AttrClass::getTemplateLibHasHandle,
           R"(Returns whether the passed handle describes an existing template
             in the library.)",
           "handle"_a)
      .def(
          "set_template_lock", &AttrClass::setTemplateLock,
          R"(This sets the lock state for the template that has the passed name.
             Lock == True makes the template unable to be deleted.
             Note : Locked templates can still be edited.)",
          "handle"_a, "lock"_a)
      .def("set_lock_by_substring", &AttrClass::setTemplatesLockBySubstring,
           R"(This sets the lock state for all templates whose handles either
             contain or explictly do not contain the passed search_str.
             Returns a list of handles for templates locked by this function
             call. Lock == True makes the template unable to be deleted.
             Note : Locked templates can still be edited.)",
           "lock"_a, "search_str"_a = "", "contains"_a = true)
      .def("set_template_list_lock", &AttrClass::setTemplateLockByHandles,
           R"(This sets the lock state for all templates whose handles
             are passed in list. Returns a list of handles for templates
             locked by this function call. Lock == True makes the template unable
             to be deleted. Note : Locked templates can still be edited.)",
           "handles"_a, "lock"_a)
      .def("remove_all_templates", &AttrClass::removeAllTemplates,
           R"(This removes, and returns, a list of all the templates referenced
             in the library that have not been marked undeletable by the system
             or read-only by the user.)")
      .def("remove_templates_by_str", &AttrClass::removeTemplatesBySubstring,
           R"(This removes, and returns, a list of all the templates referenced
             in the library that have not been marked undeletable by the system
             or read-only by the user and whose handles either contain or explictly
             do not contain the passed search_str.)",
           "search_str"_a = "", "contains"_a = true)
      .def("remove_template_by_ID", &AttrClass::removeTemplateByID,
           R"(This removes, and returns the template referenced by the passed ID
             from the library.)",
           "ID"_a)
      .def(
          "remove_template_by_handle", &AttrClass::removeTemplateByHandle,
          R"(This removes, and returns the template referenced by the passed handle
             from the library.)",
          "handle"_a)
      .def("register_template", &AttrClass::registerAttributesTemplate,
           R"(This registers a copy of the passed template in the library, and
             returns the template's integer ID.)",
           "template"_a, "specified_handle"_a = "")
      .def("get_template_by_ID",
           static_cast<T (AttrClass::*)(int)>(&AttrClass::getTemplateCopyByID),
           R"(This returns a copy of the template specified by the passed
             ID if it exists, and NULL if it does not.)",
           "ID"_a)
      .def("get_template_by_handle",
           static_cast<T (AttrClass::*)(const std::string&)>(
               &AttrClass::getTemplateCopyByHandle),
           R"(This returns a copy of the template specified by the passed
             handle if it exists, and NULL if it does not.)",
           "handle"_a);
}  // declareBaseAttributesManager

void initAttributesManagersBindings(py::module& m) {
  // ==== PrimObjTypes enum describing types of primitives supported ====
  py::enum_<assets::PrimObjTypes>(m, "PrimObjTypes")
      .value("CAPSULE_SOLID", assets::PrimObjTypes::CAPSULE_SOLID)
      .value("CAPSULE_WF", assets::PrimObjTypes::CAPSULE_WF)
      .value("CONE_SOLID", assets::PrimObjTypes::CONE_SOLID)
      .value("CONE_WF", assets::PrimObjTypes::CONE_WF)
      .value("CUBE_SOLID", assets::PrimObjTypes::CUBE_SOLID)
      .value("CUBE_WF", assets::PrimObjTypes::CUBE_WF)
      .value("CYLINDER_SOLID", assets::PrimObjTypes::CYLINDER_SOLID)
      .value("CYLINDER_WF", assets::PrimObjTypes::CYLINDER_WF)
      .value("ICOSPHERE_SOLID", assets::PrimObjTypes::ICOSPHERE_SOLID)
      .value("ICOSPHERE_WF", assets::PrimObjTypes::ICOSPHERE_WF)
      .value("UVSPHERE_SOLID", assets::PrimObjTypes::UVSPHERE_SOLID)
      .value("UVSPHERE_WF", assets::PrimObjTypes::UVSPHERE_WF)
      .value("END_PRIM_OBJ_TYPE", assets::PrimObjTypes::END_PRIM_OBJ_TYPES);

  // ==== Primitive Asset Attributes Template manager ====
  declareBaseAttributesManager<AbstractPrimitiveAttributes::ptr>(m,
                                                                 "BaseAsset");
  py::class_<AssetAttributesManager,
             AttributesManager<AbstractPrimitiveAttributes::ptr>,
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

  // ==== Physical Object Attributes Template manager ====
  declareBaseAttributesManager<ObjectAttributes::ptr>(m, "BaseObject");
  py::class_<ObjectAttributesManager, AttributesManager<ObjectAttributes::ptr>,
             ObjectAttributesManager::ptr>(m, "ObjectAttributesManager")

      // ObjectAttributesManager-specific bindings
      .def(
          "load_object_configs", &ObjectAttributesManager::loadObjectConfigs,
          R"(Build templates for all "*.phys_properties.json" files that exist in
            the provided file or directory path. If save_as_defaults is true, then
            these templates will be unable to be deleted)"
          "path"_a,
          "save_as_defaults"_a = false)

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
          R"(Returns a list of file-based template handles that either contain or explicitly do not
            contain the passed search_str, based on the value of contains.)",
          "search_str"_a = "", "contains"_a = true)
      .def(
          "get_random_file_template_handle",
          &ObjectAttributesManager::getRandomFileTemplateHandle,
          R"(Returns the handle for a random file-based template chosen from the
             existing templates being managed.)")

      // manage synthesized/primitive asset-based templates access
      .def("get_num_synth_templates",
           &ObjectAttributesManager::getNumSynthTemplateObjects, R"(
             Returns the number of existing synthesized(primitive asset)-based templates being managed.)")
      .def("get_synth_template_handles",
           static_cast<std::vector<std::string> (ObjectAttributesManager::*)(
               const std::string&, bool) const>(
               &ObjectAttributesManager::getSynthTemplateHandlesBySubstring),
           R"(
            Returns a list of template synthesized(primitive asset)-based handles that either contain or explicitly do not
            contain the passed search_str, based on the value of contains.)",
           "search_str"_a = "", "contains"_a = true)
      .def(
          "get_random_synth_template_handle",
          &ObjectAttributesManager::getRandomSynthTemplateHandle,
          R"(Returns the handle for a random synthesized(primitive asset)-based template chosen from the
             existing templates being managed.)");

  // ==== Stage Attributes Template manager ====
  declareBaseAttributesManager<StageAttributes::ptr>(m, "BaseStage");
  py::class_<StageAttributesManager, AttributesManager<StageAttributes::ptr>,
             StageAttributesManager::ptr>(m, "StageAttributesManager");

  // ==== Physics World/Manager Template manager ====

  declareBaseAttributesManager<PhysicsManagerAttributes::ptr>(m, "BasePhysics");
  py::class_<PhysicsAttributesManager,
             AttributesManager<PhysicsManagerAttributes::ptr>,
             PhysicsAttributesManager::ptr>(m, "PhysicsAttributesManager");

}  // namespace managers

}  // namespace managers

}  // namespace assets
}  // namespace esp
