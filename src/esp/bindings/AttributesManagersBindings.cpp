
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

  // // ==== Primitive Asset Attributes Template manager ====
  // py::class_<AssetAttributesManager,
  //            AttributesManager<AbstractPrimitiveAttributes::ptr>,
  //            AssetAttributesManager::ptr>(m, "AssetAttributesManager")
  //     .def("get_template_handle_by_ID",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getTemplateHandleByID,
  //          "object_id"_a)
  //     .def("get_template_ID_by_handle",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getTemplateIDByHandle,
  //          "object_id"_a)
  //     .def(
  //         "get_num_templates",
  //         &AttributesManager<AbstractPrimitiveAttributes::ptr>::getNumTemplates)
  //     .def("get_random_template_handle",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getRandomTemplateHandle)
  //     .def("get_template_handles",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getTemplateHandlesBySubstring,
  //          "search_str"_a = "", "contains"_a = true)
  //     .def("get_library_has_handle",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getTemplateLibHasHandle)

  //     .def("create_template",
  //          py::overload_cast<const std::string&, bool>(
  //              &AssetAttributesManager::createAttributesTemplate),
  //          "primitive_class_name"_a, "register_template"_a = true,
  //          py::return_value_policy::reference)
  //     .def("create_template",
  //          py::overload_cast<assets::PrimObjTypes, bool>(
  //              &AssetAttributesManager::createAttributesTemplate),
  //          "primitive_object_type_enum"_a, "register_template"_a = true,
  //          py::return_value_policy::reference)

  //     .def("register_template",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::registerAttributesTemplate,
  //          "template"_a, "template_handle"_a)
  //     // only ever return a copy of primitive asset template to user, so
  //     // that name used as map key and internal name do not get out of synch
  //     .def("get_template_by_handle",
  //          &AttributesManager<
  //              AbstractPrimitiveAttributes::ptr>::getTemplateCopyByHandle,
  //          "template_name"_a);

  // // ==== Physical Object Attributes Template manager ====
  // py::class_<ObjectAttributesManager,
  //            AttributesManager<PhysicsObjectAttributes::ptr>,
  //            ObjectAttributesManager::ptr>(m, "ObjectAttributesManager")
  //     .def("get_template_handle_by_ID",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getTemplateHandleByID,
  //          "object_id"_a)
  //     .def("get_template_ID_by_handle",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getTemplateIDByHandle,
  //          "object_id"_a)
  //     .def("get_num_templates",
  //          &AttributesManager<PhysicsObjectAttributes::ptr>::getNumTemplates)
  //     .def("get_random_template_handle",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getRandomTemplateHandle)
  //     .def("get_template_handles",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getTemplateHandlesBySubstring,
  //          "search_str"_a = "", "contains"_a = true)
  //     .def("get_library_has_handle",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getTemplateLibHasHandle)
  //     .def("get_template_by_id",
  //          &AttributesManager<PhysicsObjectAttributes::ptr>::getTemplateByID,
  //          "template_id"_a, py::return_value_policy::reference)

  //     .def("create_template",
  //          &ObjectAttributesManager::createAttributesTemplate,
  //          "primitive_object_type_enum"_a, "register_template"_a = true,
  //          py::return_value_policy::reference)
  //     .def("register_template",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::registerAttributesTemplate,
  //          "template"_a, "template_handle"_a)

  //     .def("get_num_file_templates",
  //          &ObjectAttributesManager::getNumFileTemplateObjects)
  //     .def("get_random_file_template_handle",
  //          &ObjectAttributesManager::getRandomFileTemplateHandle)
  //     .def("get_file_template_handles",
  //          &ObjectAttributesManager::getFileTemplateHandlesBySubstring,
  //          "search_str"_a = "", "contains"_a = true)
  //     .def("get_num_synth_templates",
  //          &ObjectAttributesManager::getNumSynthTemplateObjects)
  //     .def("get_random_synth_template_handle",
  //          &ObjectAttributesManager::getRandomSynthTemplateHandle)
  //     .def("get_synth_template_handles",
  //          &ObjectAttributesManager::getSynthTemplateHandlesBySubstring,
  //          "search_str"_a = "", "contains"_a = true)

  //     .def("get_template_by_handle",
  //          &AttributesManager<
  //              PhysicsObjectAttributes::ptr>::getTemplateCopyByHandle,
  //          "template_name"_a)

  //     ;

  /**
   * TODO: Add bindings for PhysicsAttributesManager and SceneAttributesManager?
   */

}  // initAttributesBindings

}  // namespace managers

}  // namespace assets
}  // namespace esp
