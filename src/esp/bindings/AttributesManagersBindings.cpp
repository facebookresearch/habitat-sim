
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/Python.h>

#include "esp/assets/Attributes.h"

#include "esp/assets/managers/AttributesManagerBase.h"

#include "esp/assets/managers/AssetAttributesManager.h"
#include "esp/assets/managers/ObjectAttributesManager.h"
#include "esp/assets/managers/PhysicsAttributesManager.h"
#include "esp/assets/managers/SceneAttributesManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace assets {
namespace managers {

/**
 * @brief instance class template base classes for attributes managers.
 * @tparam The type used to specialize class template for each attributes
 * manager.  Will be a smart pointer to an attributes
 * @param m pybind module reference.
 * @param classStrPrefix string prefix for python class name specification.
 */

template <class T>
void declareBaseAttributesManager(py::module& m, std::string classStrPrefix) {
  using AttrClass = AttributesManager<T>;
  std::string pyclass_name = classStrPrefix + std::string("AttributesManager");
  py::class_<AttrClass, std::shared_ptr<AttrClass>>(m, pyclass_name.c_str())
      .def("get_template_handle_by_ID", &AttrClass::getTemplateHandleByID,
           "id"_a)
      .def("get_template_handles",
           (std::vector<std::string>(AttrClass::*)(const std::string&, bool))(
               &AttrClass::getTemplateHandlesBySubstring),
           "search_str"_a = "", "contains"_a = true)
      .def("get_template_ID_by_handle",
           py::overload_cast<const std::string&>(
               &AttrClass::getTemplateIDByHandle),
           "handle"_a)
      .def("create_template",
           (T(AttrClass::*)(const std::string&, bool))(
               &AttrClass::createAttributesTemplate),
           "handle"_a, "register_template"_a = true)
      .def("get_num_templates", &AttrClass::getNumTemplates)
      .def("get_random_template_handle", &AttrClass::getRandomTemplateHandle)
      .def("get_library_has_handle", &AttrClass::getTemplateLibHasHandle)
      .def("remove_template_by_ID", &AttrClass::removeTemplateByID, "id"_a)
      .def("remove_template_by_handle", &AttrClass::removeTemplateByHandle,
           "handle"_a)
      .def("register_template", &AttrClass::registerAttributesTemplate,
           "template"_a, "specified_handle"_a = "")
      .def("get_template_by_ID",
           (T(AttrClass::*)(int))(&AttrClass::getTemplateCopyByID), "id"_a)
      .def("get_template_by_handle",
           (T(AttrClass::*)(const std::string&))(
               &AttrClass::getTemplateCopyByHandle),
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
  py::class_<AssetAttributesManager, AssetAttributesManager::ptr>(
      m, "AssetAttributesManager", "BaseAssetAttributesManager")

      // return appropriately cast capsule templates
      .def("get_default_capsule_template",
           &AssetAttributesManager::getDefaultCapsuleTemplate, "is_wireframe"_a)
      .def("get_capsule_template", &AssetAttributesManager::getCapsuleTemplate,
           "handle"_a)
      // return appropriately cast cone templates
      .def("get_default_cone_template",
           &AssetAttributesManager::getDefaultConeTemplate, "is_wireframe"_a)
      .def("get_cone_template", &AssetAttributesManager::getConeTemplate,
           "handle"_a)
      // return appropriately cast cube templates
      .def("get_default_cube_template",
           &AssetAttributesManager::getDefaultCubeTemplate, "is_wireframe"_a)
      .def("get_cube_template", &AssetAttributesManager::getCubeTemplate,
           "handle"_a)
      // return appropriately cast cylinder templates
      .def("get_default_cylinder_template",
           &AssetAttributesManager::getDefaultCylinderTemplate,
           "is_wireframe"_a)
      .def("get_cylinder_template",
           &AssetAttributesManager::getCylinderTemplate, "handle"_a)
      // return appropriately cast icosphere templates
      .def("get_default_icosphere_template",
           &AssetAttributesManager::getDefaultIcosphereTemplate,
           "is_wireframe"_a)
      .def("get_icosphere_template",
           &AssetAttributesManager::getIcosphereTemplate, "handle"_a)
      // return appropriately cast UVSphere templates
      .def("get_default_UVsphere_template",
           &AssetAttributesManager::getDefaultUVSphereTemplate,
           "is_wireframe"_a)
      .def("get_UVsphere_template",
           &AssetAttributesManager::getUVSphereTemplate, "handle"_a);

  // ==== Physical Object Attributes Template manager ====
  declareBaseAttributesManager<PhysicsObjectAttributes::ptr>(m, "BaseObject");
  py::class_<ObjectAttributesManager, ObjectAttributesManager::ptr>(
      m, "ObjectAttributesManager", "BaseObjectAttributesManager")
      // manage file-based templates access
      .def("get_num_file_templates",
           &ObjectAttributesManager::getNumFileTemplateObjects)
      .def("get_file_template_handles",
           (std::vector<std::string>(ObjectAttributesManager::*)(
               const std::string&, bool))(
               &ObjectAttributesManager::getFileTemplateHandlesBySubstring),
           "search_str"_a = "", "contains"_a = true)
      .def("get_random_file_template_handle",
           &ObjectAttributesManager::getRandomFileTemplateHandle)

      // manage synthesized/primitive asset-based templates access
      .def("get_num_synth_templates",
           &ObjectAttributesManager::getNumSynthTemplateObjects)
      .def("get_synth_template_handles",
           (std::vector<std::string>(ObjectAttributesManager::*)(
               const std::string&, bool))(
               &ObjectAttributesManager::getSynthTemplateHandlesBySubstring),
           "search_str"_a = "", "contains"_a = true)
      .def("get_random_synth_template_handle",
           &ObjectAttributesManager::getRandomSynthTemplateHandle);

  // ==== Scene Attributes Template manager ====
  declareBaseAttributesManager<PhysicsSceneAttributes::ptr>(m, "BaseScene");
  py::class_<SceneAttributesManager, SceneAttributesManager::ptr>(
      m, "SceneAttributesManager", "BaseSceneAttributesManager");

  // ==== Physics World/Manager Template manager ====

  declareBaseAttributesManager<PhysicsManagerAttributes::ptr>(m, "BasePhysics");
  py::class_<PhysicsAttributesManager, PhysicsAttributesManager::ptr>(
      m, "PhysicsAttributesManager", "BasePhysicsAttributesManager");

}  // namespace managers

}  // namespace managers

}  // namespace assets
}  // namespace esp
