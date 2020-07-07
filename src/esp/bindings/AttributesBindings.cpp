// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/Python.h>

#include "esp/assets/Attributes.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace assets {

void initAttributesBindings(py::module& m) {
  // ==== AbstractAttributes ====
  py::class_<AbstractAttributes, esp::core::Configuration,
             AbstractAttributes::ptr>(m, "AbstractAttributes")
      .def(py::init(
          &AbstractAttributes::create<const std::string&, const std::string&>))
      .def_property("handle", &AbstractAttributes::getHandle,
                    &AbstractAttributes::setHandle)
      .def_property_readonly("ID", &AbstractAttributes::getID)
      .def_property_readonly("template_class",
                             &AbstractAttributes::getClassKey);

  // ==== AbstractPhysicsAttributes ====
  py::class_<AbstractPhysicsAttributes, AbstractAttributes,
             AbstractPhysicsAttributes::ptr>(m, "AbstractPhysicsAttributes")
      .def(py::init(&AbstractPhysicsAttributes::create<const std::string&,
                                                       const std::string&>))
      .def_property("scale", &AbstractPhysicsAttributes::getScale,
                    &AbstractPhysicsAttributes::setScale)
      .def_property("friction_coefficient",
                    &AbstractPhysicsAttributes::getFrictionCoefficient,
                    &AbstractPhysicsAttributes::setFrictionCoefficient)
      .def_property("restitution_coefficient",
                    &AbstractPhysicsAttributes::getRestitutionCoefficient,
                    &AbstractPhysicsAttributes::setRestitutionCoefficient);

  // ==== PhysicsObjectAttributes ====
  py::class_<PhysicsObjectAttributes, AbstractPhysicsAttributes,
             PhysicsObjectAttributes::ptr>(m, "PhysicsObjectAttributes")
      .def(py::init(&PhysicsObjectAttributes::create<>))
      .def(py::init(&PhysicsObjectAttributes::create<const std::string&>))
      .def_property("com", &PhysicsObjectAttributes::getCOM,
                    &PhysicsObjectAttributes::setCOM)
      .def_property("compute_COM_from_shape",
                    &PhysicsObjectAttributes::getComputeCOMFromShape,
                    &PhysicsObjectAttributes::setComputeCOMFromShape)
      .def_property("margin", &PhysicsObjectAttributes::getMargin,
                    &PhysicsObjectAttributes::setMargin)
      .def_property("mass", &PhysicsObjectAttributes::getMass,
                    &PhysicsObjectAttributes::setMass)
      .def_property("inertia", &PhysicsObjectAttributes::getInertia,
                    &PhysicsObjectAttributes::setInertia)
      .def_property("linear_damping",
                    &PhysicsObjectAttributes::getLinearDamping,
                    &PhysicsObjectAttributes::setLinearDamping)
      .def_property("angular_damping",
                    &PhysicsObjectAttributes::getAngularDamping,
                    &PhysicsObjectAttributes::setAngularDamping)
      .def_property("render_asset_handle",
                    &PhysicsObjectAttributes::getRenderAssetHandle,
                    &PhysicsObjectAttributes::setRenderAssetHandle)
      .def_property("collision_asset_handle",
                    &PhysicsObjectAttributes::getCollisionAssetHandle,
                    &PhysicsObjectAttributes::setCollisionAssetHandle)
      .def_property("bounding_box_collisions",
                    &PhysicsObjectAttributes::getBoundingBoxCollisions,
                    &PhysicsObjectAttributes::setBoundingBoxCollisions)
      .def_property("join_collision_meshes",
                    &PhysicsObjectAttributes::getJoinCollisionMeshes,
                    &PhysicsObjectAttributes::setJoinCollisionMeshes)
      .def_property("requires_lighting",
                    &PhysicsObjectAttributes::getRequiresLighting,
                    &PhysicsObjectAttributes::setRequiresLighting)
      .def_property("semantic_id", &PhysicsObjectAttributes::getSemanticId,
                    &PhysicsObjectAttributes::setSemanticId);

  // ==== PhysicsSceneAttributes ====
  py::class_<PhysicsSceneAttributes, AbstractPhysicsAttributes,
             PhysicsSceneAttributes::ptr>(m, "PhysicsSceneAttributes")
      .def(py::init(&PhysicsSceneAttributes::create<>))
      .def(py::init(&PhysicsSceneAttributes::create<const std::string&>))
      .def_property("gravity", &PhysicsSceneAttributes::getGravity,
                    &PhysicsSceneAttributes::setGravity);

  // ==== PhysicsManagerAttributes ====
  py::class_<PhysicsManagerAttributes, AbstractAttributes,
             PhysicsManagerAttributes::ptr>(m, "PhysicsManagerAttributes")
      .def(py::init(&PhysicsManagerAttributes::create<>))
      .def(py::init(&PhysicsManagerAttributes::create<const std::string&>))
      .def_property_readonly("simulator",
                             &PhysicsManagerAttributes::getSimulator)
      .def_property("timestep", &PhysicsManagerAttributes::getTimestep,
                    &PhysicsManagerAttributes::setTimestep)
      .def_property("max_substeps", &PhysicsManagerAttributes::getMaxSubsteps,
                    &PhysicsManagerAttributes::setMaxSubsteps)
      .def_property("gravity", &PhysicsManagerAttributes::getGravity,
                    &PhysicsManagerAttributes::setGravity)
      .def_property("friction_coefficient",
                    &PhysicsManagerAttributes::getFrictionCoefficient,
                    &PhysicsManagerAttributes::setFrictionCoefficient)
      .def_property("restitution_coefficient",
                    &PhysicsManagerAttributes::getRestitutionCoefficient,
                    &PhysicsManagerAttributes::setRestitutionCoefficient);

  // ==== AbstractPrimitiveAttributes ====
  py::class_<AbstractPrimitiveAttributes, AbstractAttributes,
             AbstractPrimitiveAttributes::ptr>(m, "AbstractPrimitiveAttributes")
      .def_property_readonly("is_wireframe",
                             &AbstractPrimitiveAttributes::getIsWireframe)
      .def_property("use_texture_coords",
                    &AbstractPrimitiveAttributes::getUseTextureCoords,
                    &AbstractPrimitiveAttributes::setUseTextureCoords)
      .def_property("use_tangents",
                    &AbstractPrimitiveAttributes::getUseTangents,
                    &AbstractPrimitiveAttributes::setUseTangents)
      .def_property("num_rings", &AbstractPrimitiveAttributes::getNumRings,
                    &AbstractPrimitiveAttributes::setNumRings)
      .def_property("num_segments",
                    &AbstractPrimitiveAttributes::getNumSegments,
                    &AbstractPrimitiveAttributes::setNumSegments)
      .def_property("get_half_length",
                    &AbstractPrimitiveAttributes::getHalfLength,
                    &AbstractPrimitiveAttributes::setHalfLength)
      .def_property_readonly("prim_obj_class_name",
                             &AbstractPrimitiveAttributes::getPrimObjClassName)
      .def_property_readonly("prim_obj_type",
                             &AbstractPrimitiveAttributes::getPrimObjType)
      .def("build_handle", &AbstractPrimitiveAttributes::buildHandle)
      .def_property_readonly("is_valid_template",
                             &AbstractPrimitiveAttributes::isValidTemplate);

  // ==== CapsulePrimitiveAttributes ====
  py::class_<CapsulePrimitiveAttributes, AbstractPrimitiveAttributes,
             CapsulePrimitiveAttributes::ptr>(m, "CapsulePrimitiveAttributes")
      .def(py::init(
          &CapsulePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property("hemisphere_rings",
                    &CapsulePrimitiveAttributes::getHemisphereRings,
                    &CapsulePrimitiveAttributes::setHemisphereRings)
      .def_property("cylinder_rings",
                    &CapsulePrimitiveAttributes::getCylinderRings,
                    &CapsulePrimitiveAttributes::setCylinderRings);

  // ==== ConePrimitiveAttributes ====
  py::class_<ConePrimitiveAttributes, AbstractPrimitiveAttributes,
             ConePrimitiveAttributes::ptr>(m, "ConePrimitiveAttributes")
      .def(py::init(
          &ConePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property("use_cap_end", &ConePrimitiveAttributes::getCapEnd,
                    &ConePrimitiveAttributes::setCapEnd);

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
      .def_property("use_cap_ends", &CylinderPrimitiveAttributes::getCapEnds,
                    &CylinderPrimitiveAttributes::setCapEnds);

  // ==== IcospherePrimitiveAttributes ====
  py::class_<IcospherePrimitiveAttributes, AbstractPrimitiveAttributes,
             IcospherePrimitiveAttributes::ptr>(m,
                                                "IcospherePrimitiveAttributes")
      .def(py::init(
          &IcospherePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property("subdivisions",
                    &IcospherePrimitiveAttributes::getSubdivisions,
                    &IcospherePrimitiveAttributes::setSubdivisions);

  // ==== UVSpherePrimitiveAttributes ====
  py::class_<UVSpherePrimitiveAttributes, AbstractPrimitiveAttributes,
             UVSpherePrimitiveAttributes::ptr>(m, "UVSpherePrimitiveAttributes")
      .def(py::init(
          &UVSpherePrimitiveAttributes::create<bool, int, const std::string&>));

}  // initAttributesBindings

}  // namespace assets
}  // namespace esp
