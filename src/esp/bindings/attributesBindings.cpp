// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/Python.h>

#include "esp/assets/Attributes.h"
#include "esp/assets/ResourceManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace assets {

void initAttributesBindings(py::module& m) {
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

  // ==== AbstractAttributes ====
  py::class_<AbstractAttributes, esp::core::Configuration,
             AbstractAttributes::ptr>(m, "AbstractAttributes")
      .def(py::init(&AbstractAttributes::create<>))
      .def(py::init(&AbstractAttributes::create<const std::string&>))
      .def("set_origin_handle", &AbstractAttributes::setOriginHandle,
           "origin_handle"_a)
      .def("get_origin_handle", &AbstractAttributes::getOriginHandle)
      .def("get_object_template_ID", &AbstractAttributes::getObjectTemplateID);

  // ==== AbstractPhysicsAttributes ====
  py::class_<AbstractPhysicsAttributes, AbstractAttributes,
             AbstractPhysicsAttributes::ptr>(m, "AbstractPhysicsAttributes")
      .def(py::init(&AbstractPhysicsAttributes::create<>))
      .def(py::init(&AbstractPhysicsAttributes::create<const std::string&>))
      .def("set_scale", &AbstractPhysicsAttributes::setScale, "scale"_a)
      .def("get_scale", &AbstractPhysicsAttributes::getScale)
      .def("set_friction_coefficient",
           &AbstractPhysicsAttributes::setFrictionCoefficient,
           "friction_coefficient"_a)
      .def("get_friction_coefficient",
           &AbstractPhysicsAttributes::getFrictionCoefficient)
      .def("set_restitution_coefficient",
           &AbstractPhysicsAttributes::setRestitutionCoefficient,
           "restitution_coefficient"_a)
      .def("get_restitution_coefficient",
           &AbstractPhysicsAttributes::getRestitutionCoefficient);

  // ==== PhysicsObjectAttributes ====
  py::class_<PhysicsObjectAttributes, AbstractPhysicsAttributes,
             PhysicsObjectAttributes::ptr>(m, "PhysicsObjectAttributes")
      .def(py::init(&PhysicsObjectAttributes::create<>))
      .def(py::init(&PhysicsObjectAttributes::create<const std::string&>))
      .def("set_com", &PhysicsObjectAttributes::setCOM, "com"_a)
      .def("get_com", &PhysicsObjectAttributes::getCOM)
      .def("set_margin", &PhysicsObjectAttributes::setMargin, "margin"_a)
      .def("get_margin", &PhysicsObjectAttributes::getMargin)
      .def("set_mass", &PhysicsObjectAttributes::setMass, "mass"_a)
      .def("get_mass", &PhysicsObjectAttributes::getMass)
      .def("set_inertia", &PhysicsObjectAttributes::setInertia, "inertia"_a)
      .def("get_inertia", &PhysicsObjectAttributes::getInertia)

      .def("set_linear_damping", &PhysicsObjectAttributes::setLinearDamping,
           "linear_damping"_a)
      .def("get_linear_damping", &PhysicsObjectAttributes::getLinearDamping)
      .def("set_angular_damping", &PhysicsObjectAttributes::setAngularDamping,
           "angular_damping"_a)
      .def("get_angular_damping", &PhysicsObjectAttributes::getAngularDamping)
      .def("set_render_asset_handle",
           &PhysicsObjectAttributes::setRenderAssetHandle,
           "render_asset_handle"_a)
      .def("get_render_asset_handle",
           &PhysicsObjectAttributes::getRenderAssetHandle)
      .def("set_collision_asset_handle",
           &PhysicsObjectAttributes::setCollisionAssetHandle,
           "collision_asset_handle"_a)
      .def("get_collision_asset_handle",
           &PhysicsObjectAttributes::getCollisionAssetHandle)
      .def("set_bounding_box_collisions",
           &PhysicsObjectAttributes::setBoundingBoxCollisions,
           "use_bounding_box_for_collision"_a)
      .def("get_bounding_box_collisions",
           &PhysicsObjectAttributes::getBoundingBoxCollisions)
      .def("set_join_collision_meshes",
           &PhysicsObjectAttributes::setJoinCollisionMeshes,
           "join_collision_meshes"_a)
      .def("get_join_collision_meshes",
           &PhysicsObjectAttributes::getJoinCollisionMeshes)
      .def("set_requires_lighting",
           &PhysicsObjectAttributes::setRequiresLighting, "requires_lighting"_a)
      .def("get_requires_lighting",
           &PhysicsObjectAttributes::getRequiresLighting);

  // ==== AbstractPrimitiveAttributes ====
  py::class_<AbstractPrimitiveAttributes, AbstractAttributes,
             AbstractPrimitiveAttributes::ptr>(m, "AbstractPrimitiveAttributes")
      .def(py::init(
          &AbstractPrimitiveAttributes::create<bool, int, const std::string&>))
      .def("set_origin_handle", &AbstractPrimitiveAttributes::setOriginHandle,
           "do_not_use"_a)
      .def("get_is_wireframe", &AbstractPrimitiveAttributes::getIsWireframe)
      .def("set_use_texture_coords",
           &AbstractPrimitiveAttributes::setUseTextureCoords,
           "use_texture_coords"_a)
      .def("get_use_texture_coords",
           &AbstractPrimitiveAttributes::getUseTextureCoords)
      .def("set_use_tangents", &AbstractPrimitiveAttributes::setUseTangents,
           "use_tangents"_a)
      .def("get_use_tangents", &AbstractPrimitiveAttributes::getUseTangents)

      .def("set_num_rings", &AbstractPrimitiveAttributes::setNumRings,
           "num_rings"_a)
      .def("get_num_rings", &AbstractPrimitiveAttributes::getNumRings)
      .def("set_num_segments", &AbstractPrimitiveAttributes::setNumSegments,
           "num_segments"_a)
      .def("get_num_segments", &AbstractPrimitiveAttributes::getNumSegments)
      .def("set_half_length", &AbstractPrimitiveAttributes::setHalfLength,
           "half_length"_a)
      .def("get_half_length", &AbstractPrimitiveAttributes::getHalfLength)
      .def("get_prim_obj_class_name",
           &AbstractPrimitiveAttributes::getPrimObjClassName)
      .def("get_prim_obj_type", &AbstractPrimitiveAttributes::getPrimObjType);

  // ==== CapsulePrimitiveAttributes ====
  py::class_<CapsulePrimitiveAttributes, AbstractPrimitiveAttributes,
             CapsulePrimitiveAttributes::ptr>(m, "CapsulePrimitiveAttributes")
      .def(py::init(
          &CapsulePrimitiveAttributes::create<bool, int, const std::string&>))
      .def("set_hemisphere_rings",
           &CapsulePrimitiveAttributes::setHemisphereRings,
           "hemisphere_rings"_a)
      .def("get_hemisphere_rings",
           &CapsulePrimitiveAttributes::getHemisphereRings)
      .def("set_cylinder_rings", &CapsulePrimitiveAttributes::setCylinderRings,
           "cylinder_rings"_a)
      .def("get_cylinder_rings", &CapsulePrimitiveAttributes::getCylinderRings);

  // ==== ConePrimitiveAttributes ====
  py::class_<ConePrimitiveAttributes, AbstractPrimitiveAttributes,
             ConePrimitiveAttributes::ptr>(m, "ConePrimitiveAttributes")
      .def(py::init(
          &ConePrimitiveAttributes::create<bool, int, const std::string&>))
      .def("set_cap_end", &ConePrimitiveAttributes::setCapEnd, "cap_end"_a)
      .def("get_cap_end", &ConePrimitiveAttributes::getCapEnd);

  // ==== CubePrimitiveAttributes ====
  py::class_<CubePrimitiveAttributes, AbstractPrimitiveAttributes,
             CubePrimitiveAttributes::ptr>(m, "CubePrimitiveAttributes")
      .def(py::init(
          &CubePrimitiveAttributes::create<bool, int, const std::string&>));

  // ==== CylinderPrimitiveAttributes ====
  py::class_<CylinderPrimitiveAttributes, AbstractPrimitiveAttributes,
             CylinderPrimitiveAttributes::ptr>(m, "CylinderPrimitiveAttributes")
      .def(py::init(
          &CylinderPrimitiveAttributes::create<bool, int, const std::string&>))
      .def("set_cap_ends", &CylinderPrimitiveAttributes::setCapEnds,
           "cap_ends"_a)
      .def("get_cap_ends", &CylinderPrimitiveAttributes::getCapEnds);

  // ==== IcospherePrimitiveAttributes ====
  py::class_<IcospherePrimitiveAttributes, AbstractPrimitiveAttributes,
             IcospherePrimitiveAttributes::ptr>(m,
                                                "IcospherePrimitiveAttributes")
      .def(py::init(
          &IcospherePrimitiveAttributes::create<bool, int, const std::string&>))
      .def("set_subdivisions", &IcospherePrimitiveAttributes::setSubdivisions,
           "subdivisions"_a)
      .def("get_subdivisions", &IcospherePrimitiveAttributes::getSubdivisions);

  // ==== UVSpherePrimitiveAttributes ====
  py::class_<UVSpherePrimitiveAttributes, AbstractPrimitiveAttributes,
             UVSpherePrimitiveAttributes::ptr>(m, "UVSpherePrimitiveAttributes")
      .def(py::init(
          &UVSpherePrimitiveAttributes::create<bool, int, const std::string&>));

}  // initAttributesBindings

}  // namespace assets
}  // namespace esp
