// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

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
                    &AbstractAttributes::setHandle,
                    R"(Name of attributes template. )")
      .def_property_readonly(
          "file_directory", &AbstractAttributes::getFileDirectory,
          R"(Directory where file-based templates were loaded from.)")
      .def_property_readonly(
          "ID", &AbstractAttributes::getID,
          R"(System-generated ID for template.  Will be unique among templates
          of same type.)")
      .def_property_readonly("template_class", &AbstractAttributes::getClassKey,
                             R"(Class name of Attributes template.)");

  // ==== AbstractPhysicsAttributes ====
  py::class_<AbstractPhysicsAttributes, AbstractAttributes,
             AbstractPhysicsAttributes::ptr>(m, "AbstractPhysicsAttributes")
      .def(py::init(&AbstractPhysicsAttributes::create<const std::string&,
                                                       const std::string&>))
      .def_property(
          "scale", &AbstractPhysicsAttributes::getScale,
          &AbstractPhysicsAttributes::setScale,
          R"(Scale multiplier for constructions built from this template in x,y,z)")
      .def_property(
          "margin", &AbstractPhysicsAttributes::getMargin,
          &AbstractPhysicsAttributes::setMargin,
          R"(Collision margin for constructions built from this template.)")
      .def_property(
          "orient_up", &AbstractPhysicsAttributes::getOrientUp,
          &AbstractPhysicsAttributes::setOrientUp,
          R"(Up direction for constructions built from this template.)")
      .def_property(
          "orient_front", &AbstractPhysicsAttributes::getOrientFront,
          &AbstractPhysicsAttributes::setOrientFront,
          R"(Forward direction for constructions built from this template.)")
      .def_property("units_to_meters",
                    &AbstractPhysicsAttributes::getUnitsToMeters,
                    &AbstractPhysicsAttributes::setUnitsToMeters,
                    R"(Conversion ratio for given units to meters.)")
      .def_property(
          "friction_coefficient",
          &AbstractPhysicsAttributes::getFrictionCoefficient,
          &AbstractPhysicsAttributes::setFrictionCoefficient,
          R"(Friction coefficient for constructions built from this template.)")
      .def_property(
          "restitution_coefficient",
          &AbstractPhysicsAttributes::getRestitutionCoefficient,
          &AbstractPhysicsAttributes::setRestitutionCoefficient,
          R"(Coefficient of restitution for constructions built from this template.)")
      .def_property(
          "render_asset_type", &AbstractPhysicsAttributes::getRenderAssetType,
          &AbstractPhysicsAttributes::setRenderAssetType,
          R"(Type of the mesh asset used to render constructions built 
          from this template.)")
      .def_property(
          "collision_asset_type",
          &AbstractPhysicsAttributes::getCollisionAssetType,
          &AbstractPhysicsAttributes::setCollisionAssetType,
          R"(Type of the mesh asset used for collision calculations for 
          constructions built from this template.)")
      .def_property(
          "render_asset_handle",
          &AbstractPhysicsAttributes::getRenderAssetHandle,
          &AbstractPhysicsAttributes::setRenderAssetHandle,
          R"(Handle of the asset used to render constructions built from 
          this template.)")
      .def_property(
          "collision_asset_handle",
          &AbstractPhysicsAttributes::getCollisionAssetHandle,
          &AbstractPhysicsAttributes::setCollisionAssetHandle,
          R"(Handle of the asset used to calculate collsions for constructions 
          built from this template.)")
      .def_property(
          "requires_lighting", &AbstractPhysicsAttributes::getRequiresLighting,
          &AbstractPhysicsAttributes::setRequiresLighting,
          R"(Whether constructions built from this template should use phong 
          shading or not.)")
      .def_property_readonly(
          "render_asset_is_primitive",
          &AbstractPhysicsAttributes::getRenderAssetIsPrimitive,
          R"(Whether constructions built from this template should 
          be rendered using an internally sourced primitive.)")
      .def_property_readonly(
          "collision_asset_is_primitive",
          &AbstractPhysicsAttributes::getCollisionAssetIsPrimitive,
          R"(Whether collisions invloving constructions built from 
          this template should be solved using an internally sourced 
          primitive.)")
      .def_property_readonly(
          "use_mesh_for_collision",
          &AbstractPhysicsAttributes::getUseMeshCollision,
          R"(Whether collisions involving constructions built from 
           this template should be solved using the collision mesh 
           or a primitive.)")
      .def_property_readonly(
          "is_dirty", &AbstractPhysicsAttributes::getIsDirty,
          R"(Whether values in this attributes have been changed requiring 
          re-registartion before they can be used an object can be created. )");

  // ==== PhysicsObjectAttributes ====
  py::class_<PhysicsObjectAttributes, AbstractPhysicsAttributes,
             PhysicsObjectAttributes::ptr>(m, "PhysicsObjectAttributes")
      .def(py::init(&PhysicsObjectAttributes::create<>))
      .def(py::init(&PhysicsObjectAttributes::create<const std::string&>))
      .def_property(
          "com", &PhysicsObjectAttributes::getCOM,
          &PhysicsObjectAttributes::setCOM,
          R"(The Center of Mass for objects built from this template.)")
      .def_property(
          "compute_COM_from_shape",
          &PhysicsObjectAttributes::getComputeCOMFromShape,
          &PhysicsObjectAttributes::setComputeCOMFromShape,
          R"(Whether the COM should be calculated when an object is created 
          based on its bounding box)")
      .def_property("mass", &PhysicsObjectAttributes::getMass,
                    &PhysicsObjectAttributes::setMass,
                    R"(The mass of objects constructed from this template.)")
      .def_property(
          "inertia", &PhysicsObjectAttributes::getInertia,
          &PhysicsObjectAttributes::setInertia,
          R"(The diagonal of the Intertia matrix for objects constructed 
          from this template.)")
      .def_property(
          "linear_damping", &PhysicsObjectAttributes::getLinearDamping,
          &PhysicsObjectAttributes::setLinearDamping,
          R"(The damping of the linear velocity for objects constructed 
          from this template.)")
      .def_property(
          "angular_damping", &PhysicsObjectAttributes::getAngularDamping,
          &PhysicsObjectAttributes::setAngularDamping,
          R"(The damping of angular velocity for objects constructed from 
          this template.)")
      .def_property(
          "bounding_box_collisions",
          &PhysicsObjectAttributes::getBoundingBoxCollisions,
          &PhysicsObjectAttributes::setBoundingBoxCollisions,
          R"(Whether objects constructed from this template should use 
          bounding box for collisions or designated mesh.)")
      .def_property(
          "join_collision_meshes",
          &PhysicsObjectAttributes::getJoinCollisionMeshes,
          &PhysicsObjectAttributes::setJoinCollisionMeshes,
          R"(Whether collision meshes for objects constructed from this 
          template should be joined into a convex hull or kept separate.)")
      .def_property(
          "is_visibile", &PhysicsObjectAttributes::getIsVisible,
          &PhysicsObjectAttributes::setIsVisible,
          R"(Whether objects constructed from this template are visible.)")
      .def_property(
          "is_collidable", &PhysicsObjectAttributes::getIsCollidable,
          &PhysicsObjectAttributes::setIsCollidable,
          R"(Whether objects constructed from this template are collidable.)")
      .def_property(
          "semantic_id", &PhysicsObjectAttributes::getSemanticId,
          &PhysicsObjectAttributes::setSemanticId,
          R"(The semantic ID for objects constructed from this template.)");

  // ==== PhysicsStageAttributes ====
  py::class_<PhysicsStageAttributes, AbstractPhysicsAttributes,
             PhysicsStageAttributes::ptr>(m, "PhysicsStageAttributes")
      .def(py::init(&PhysicsStageAttributes::create<>))
      .def(py::init(&PhysicsStageAttributes::create<const std::string&>))
      .def_property(
          "gravity", &PhysicsStageAttributes::getGravity,
          &PhysicsStageAttributes::setGravity,
          R"(The 3-vector representation of gravity to use for physically-based 
          simulations on stages built from this template.)")
      .def_property(
          "origin", &PhysicsStageAttributes::getOrigin,
          &PhysicsStageAttributes::setOrigin,
          R"(The desired location of the origin of stages built from this 
          template.)")
      .def_property(
          "semantic_asset_handle",
          &PhysicsStageAttributes::getSemanticAssetHandle,
          &PhysicsStageAttributes::setSemanticAssetHandle,
          R"(Handle of the asset used for semantic segmentation of stages 
          built from this template.)")
      .def_property(
          "semantic_asset_type", &PhysicsStageAttributes::getSemanticAssetType,
          &PhysicsStageAttributes::setSemanticAssetType,
          R"(Type of asset used for collision calculations for constructions 
          built from this template.)")
      .def_property("navmesh_asset_handle",
                    &PhysicsStageAttributes::getNavmeshAssetHandle,
                    &PhysicsStageAttributes::setNavmeshAssetHandle)
      .def_property("house_filename", &PhysicsStageAttributes::getHouseFilename,
                    &PhysicsStageAttributes::setHouseFilename)
      .def_property("light_setup", &PhysicsStageAttributes::getLightSetup,
                    &PhysicsStageAttributes::setLightSetup)
      .def_property("frustrum_culling",
                    &PhysicsStageAttributes::getFrustrumCulling,
                    &PhysicsStageAttributes::setFrustrumCulling);

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
      .def_property("half_length", &AbstractPrimitiveAttributes::getHalfLength,
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
