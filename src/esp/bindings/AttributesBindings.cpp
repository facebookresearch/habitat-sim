// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/PythonBindings.h>

#include "esp/core/managedContainers/AbstractManagedObject.h"
#include "esp/metadata/attributes/AbstractObjectAttributes.h"
#include "esp/metadata/attributes/ArticulatedObjectAttributes.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/metadata/attributes/LightLayoutAttributes.h"
#include "esp/metadata/attributes/ObjectAttributes.h"
#include "esp/metadata/attributes/PbrShaderAttributes.h"
#include "esp/metadata/attributes/PhysicsManagerAttributes.h"
#include "esp/metadata/attributes/PrimitiveAssetAttributes.h"
#include "esp/metadata/attributes/SceneInstanceAttributes.h"
#include "esp/metadata/attributes/StageAttributes.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace Attrs = esp::metadata::attributes;
using Attrs::AbstractAttributes;
using Attrs::AbstractObjectAttributes;
using Attrs::AbstractPrimitiveAttributes;
using Attrs::ArticulatedObjectAttributes;
using Attrs::CapsulePrimitiveAttributes;
using Attrs::ConePrimitiveAttributes;
using Attrs::CubePrimitiveAttributes;
using Attrs::CylinderPrimitiveAttributes;
using Attrs::IcospherePrimitiveAttributes;
using Attrs::LightInstanceAttributes;
using Attrs::LightLayoutAttributes;
using Attrs::ObjectAttributes;
using Attrs::PbrShaderAttributes;
using Attrs::PhysicsManagerAttributes;
using Attrs::StageAttributes;
using Attrs::UVSpherePrimitiveAttributes;
using esp::core::managedContainers::AbstractFileBasedManagedObject;
using esp::core::managedContainers::AbstractManagedObject;

namespace esp {
namespace metadata {

void initAttributesBindings(py::module& m) {
  // ==== AbstractManagedObject ====
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<AbstractManagedObject, AbstractManagedObject::ptr>(
      m, "AbstractManagedObject");
  // ==== AbstractFileBasedManagedObject ====
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<AbstractFileBasedManagedObject, AbstractManagedObject,
             AbstractFileBasedManagedObject::ptr>(
      m, "AbstractFileBasedManagedObject");

  // ==== AbstractAttributes ====
  py::class_<AbstractAttributes, AbstractFileBasedManagedObject,
             esp::core::config::Configuration, AbstractAttributes::ptr>(
      m, "AbstractAttributes")
      .def(py::init(
          &AbstractAttributes::create<const std::string&, const std::string&>))
      .def_property("handle", &AbstractAttributes::getHandle,
                    &AbstractAttributes::setHandle,
                    R"(Name of attributes template. )")
      .def_property_readonly(
          "file_directory", &AbstractAttributes::getFileDirectory,
          R"(Directory where file-based templates were loaded from.)")
      .def_property_readonly(
          "template_id", &AbstractAttributes::getID,
          R"(System-generated ID for template.  Will be unique among templates
          of same type.)")
      .def(
          "get_user_config", &AbstractAttributes::editUserConfiguration,
          py::return_value_policy::reference_internal,
          R"(Returns a reference to the User Config object for this attributes, so that it can be
          viewed or modified. Any changes to the user_config will require the owning
          attributes to be re-registered.)")
      .def_property_readonly(
          "num_user_configs",
          &AbstractAttributes::getNumUserDefinedConfigurations,
          R"(The number of currently specified user-defined configuration values.)")
      .def_property_readonly("template_class", &AbstractAttributes::getClassKey,
                             R"(Class name of Attributes template.)")
      .def_property_readonly(
          "csv_info", &AbstractAttributes::getObjectInfo,
          R"(Comma-separated informational string describing this Attributes template)")

      // Attributes should only use named properties or subconfigurations to set
      // string values, to guarantee essential value type integrity.)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const std::string& val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set string values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const char* val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set string values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const int val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set integer values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const double val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set floating-point values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const bool val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set boolean values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const Magnum::Quaternion& val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set Nagnum::Quaternion values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](CORRADE_UNUSED AbstractAttributes& self,
             CORRADE_UNUSED const std::string& key,
             CORRADE_UNUSED const Magnum::Vector3& val) {
            ESP_CHECK(false,
                      "Attributes should only use named properties or "
                      "subconfigurations to set Magnum::Vector3 values, to "
                      "guarantee essential value type integrity.");
          },
          R"(This method is inherited from Configuration, but should not be used with Attributes due
          to the possibility of changing the type of a required variable. Use the provided Attributes
          instead, to change values for this object.)",
          "key"_a, "value"_a);
  // ======== Enums ================

  // ==== ArticulatedObjectBaseType enum ====
  // Describes the type of base joint used to connect the Articulated Object to
  // the world
  py::enum_<metadata::attributes::ArticulatedObjectBaseType>(
      m, "ArticulatedObjectBaseType")
      .value("UNSPECIFIED",
             metadata::attributes::ArticulatedObjectBaseType::Unspecified)
      .value("FREE", metadata::attributes::ArticulatedObjectBaseType::Free)
      .value("FIXED", metadata::attributes::ArticulatedObjectBaseType::Fixed);

  // ==== ArticulatedObjectInertiaSource enum ====
  // Describes the source of the interia values to use for the Articulated
  // Object.
  py::enum_<metadata::attributes::ArticulatedObjectInertiaSource>(
      m, "ArticulatedObjectInertiaSource")
      .value("UNSPECIFIED",
             metadata::attributes::ArticulatedObjectInertiaSource::Unspecified)
      .value("COMPUTED",
             metadata::attributes::ArticulatedObjectInertiaSource::Computed)
      .value("URDF",
             metadata::attributes::ArticulatedObjectInertiaSource::URDF);

  //
  // ==== ArticulatedObjectLinkOrder enum ====
  // Describes how the links in the Articulated Object should be ordered.
  py::enum_<metadata::attributes::ArticulatedObjectLinkOrder>(
      m, "ArticulatedObjectLinkOrder")
      .value("UNSPECIFIED",
             metadata::attributes::ArticulatedObjectLinkOrder::Unspecified)
      .value("URDF_ORDER",
             metadata::attributes::ArticulatedObjectLinkOrder::URDFOrder)
      .value("TREE_TRAVERSAL",
             metadata::attributes::ArticulatedObjectLinkOrder::TreeTraversal);

  // ==== ArticulatedObjectAttributes ====
  py::class_<ArticulatedObjectAttributes, AbstractAttributes,
             ArticulatedObjectAttributes::ptr>(
      m, "ArticulatedObjectAttributes",
      R"(A metadata template for articulated object configurations. Can be imported from
      .ao_config.json files.)")
      .def(py::init(&ArticulatedObjectAttributes::create<>))
      .def(py::init(&ArticulatedObjectAttributes::create<const std::string&>))
      .def_property(
          "urdf_filepath", &ArticulatedObjectAttributes::getURDFPath,
          &ArticulatedObjectAttributes::setURDFPath,
          R"(Relative filepath of the URDF file used to create the Articulated Object
          described by this template.)")
      .def_property(
          "render_asset_handle",
          &ArticulatedObjectAttributes::getRenderAssetHandle,
          &ArticulatedObjectAttributes::setRenderAssetHandle,
          R"(Handle of the asset used to render constructions built from
          this articulated object template.)")
      .def_property(
          "semantic_id", &ArticulatedObjectAttributes::getSemanticId,
          &ArticulatedObjectAttributes::setSemanticId,
          R"(The semantic ID for articulated objects constructed from this template.)")
      .def_property(
          "shader_type", &ArticulatedObjectAttributes::getShaderType,
          &ArticulatedObjectAttributes::setShaderType,
          R"(The shader type [0=material, 1=flat, 2=phong, 3=pbr] to use for this construction.
          Currently Articulated Objects only support Flat/Phong shading.)")
      .def_property(
          "render_mode", &ArticulatedObjectAttributes::getRenderMode,
          &ArticulatedObjectAttributes::setRenderMode,
          R"(Whether we should render using the articulated object's its skin,
          its xml defined rigid visual elements, both or nothing.)")
      .def_property(
          "base_type", &ArticulatedObjectAttributes::getBaseType,
          &ArticulatedObjectAttributes::setBaseType,
          R"(The type of base/root joint to use to add this Articulated Object to the world.
          Possible values are "FREE" and "FIXED".)")
      .def_property(
          "inertia_source", &ArticulatedObjectAttributes::getInertiaSource,
          &ArticulatedObjectAttributes::setInertiaSource,
          R"(Tthe source of the inertia tensors to use for this Articulated Object.
          Possible values are "COMPUTED" and "URDF".)")
      .def_property(
          "link_order", &ArticulatedObjectAttributes::getLinkOrder,
          &ArticulatedObjectAttributes::setLinkOrder,
          R"(The link order to use for the linkages of this Articulated Object.
          Possible values are "URDF_ORDER" and "TREE_TRAVERSAL".)");

  // ==== AbstractObjectAttributes ====
  py::class_<AbstractObjectAttributes, AbstractAttributes,
             AbstractObjectAttributes::ptr>(m, "AbstractObjectAttributes")
      .def(py::init(&AbstractObjectAttributes::create<const std::string&,
                                                      const std::string&>))
      .def_property(
          "scale", &AbstractObjectAttributes::getScale,
          &AbstractObjectAttributes::setScale,
          R"(Scale multiplier for constructions built from this template in x,y,z)")
      .def_property(
          "collision_asset_size",
          &AbstractObjectAttributes::getCollisionAssetSize,
          &AbstractObjectAttributes::setCollisionAssetSize,
          R"(Size of collsion assets for constructions built from this template in
          x,y,z.  Default is [1.0,1.0,1.0].  This is used to resize a collision asset
          to match a render asset if necessary, such as when using a primitive.)")
      .def_property(
          "margin", &AbstractObjectAttributes::getMargin,
          &AbstractObjectAttributes::setMargin,
          R"(Collision margin for constructions built from this template.)")
      .def_property(
          "orient_up", &AbstractObjectAttributes::getOrientUp,
          &AbstractObjectAttributes::setOrientUp,
          R"(Up direction for constructions built from this template.)")
      .def_property(
          "orient_front", &AbstractObjectAttributes::getOrientFront,
          &AbstractObjectAttributes::setOrientFront,
          R"(Forward direction for constructions built from this template.)")
      .def_property("units_to_meters",
                    &AbstractObjectAttributes::getUnitsToMeters,
                    &AbstractObjectAttributes::setUnitsToMeters,
                    R"(Conversion ratio for given units to meters.)")
      .def_property(
          "friction_coefficient",
          &AbstractObjectAttributes::getFrictionCoefficient,
          &AbstractObjectAttributes::setFrictionCoefficient,
          R"(Friction coefficient for constructions built from this template.)")
      .def_property(
          "rolling_friction_coefficient",
          &AbstractObjectAttributes::getRollingFrictionCoefficient,
          &AbstractObjectAttributes::setRollingFrictionCoefficient,
          R"(Rolling friction coefficient for constructions built from this template. Damps angular velocity about axis orthogonal to the contact normal to prevent rounded shapes from rolling forever.)")
      .def_property(
          "spinning_friction_coefficient",
          &AbstractObjectAttributes::getSpinningFrictionCoefficient,
          &AbstractObjectAttributes::setSpinningFrictionCoefficient,
          R"(Spinning friction coefficient for constructions built from this template. Damps angular velocity about the contact normal.)")
      .def_property(
          "restitution_coefficient",
          &AbstractObjectAttributes::getRestitutionCoefficient,
          &AbstractObjectAttributes::setRestitutionCoefficient,
          R"(Coefficient of restitution for constructions built from this template.)")
      .def_property("render_asset_type",
                    &AbstractObjectAttributes::getRenderAssetType,
                    &AbstractObjectAttributes::setRenderAssetType,
                    R"(Type of the mesh asset used to render constructions built
          from this template.)")
      .def_property(
          "collision_asset_type",
          &AbstractObjectAttributes::getCollisionAssetType,
          &AbstractObjectAttributes::setCollisionAssetType,
          R"(Type of the mesh asset used for collision calculations for
          constructions built from this template.)")
      .def_property(
          "render_asset_handle",
          &AbstractObjectAttributes::getRenderAssetHandle,
          &AbstractObjectAttributes::setRenderAssetHandle,
          R"(Handle of the asset used to render constructions built from
          this template.)")
      .def_property(
          "collision_asset_handle",
          &AbstractObjectAttributes::getCollisionAssetHandle,
          &AbstractObjectAttributes::setCollisionAssetHandle,
          R"(Handle of the asset used to calculate collsions for constructions
          built from this template.)")
      .def_property(
          "shader_type", &AbstractObjectAttributes::getShaderType,
          &AbstractObjectAttributes::setShaderType,
          R"(The shader type [0=material, 1=flat, 2=phong, 3=pbr] to use for this construction)")
      .def_property(
          "force_flat_shading", &AbstractObjectAttributes::getForceFlatShading,
          &AbstractObjectAttributes::setForceFlatShading,
          R"(If true, this object will be rendered flat, ignoring shader type settings.)")
      .def_property_readonly(
          "render_asset_is_primitive",
          &AbstractObjectAttributes::getRenderAssetIsPrimitive,
          R"(Whether constructions built from this template should
          be rendered using an internally sourced primitive.)")
      .def_property_readonly(
          "collision_asset_is_primitive",
          &AbstractObjectAttributes::getCollisionAssetIsPrimitive,
          R"(Whether collisions involving constructions built from
          this template should be solved using an internally sourced
          primitive.)")
      .def_property_readonly(
          "use_mesh_for_collision",
          &AbstractObjectAttributes::getUseMeshCollision,
          R"(Whether collisions involving constructions built from
           this template should be solved using the collision mesh
           or a primitive.)")
      .def_property(
          "is_collidable", &ObjectAttributes::getIsCollidable,
          &ObjectAttributes::setIsCollidable,
          R"(Whether constructions built from this template are collidable upon initialization.)")
      .def_property_readonly(
          "is_dirty", &AbstractObjectAttributes::getIsDirty,
          R"(Whether values in this attributes have been changed requiring
          re-registration before they can be used an object can be created. )");

  // ==== ObjectAttributes ====
  py::class_<ObjectAttributes, AbstractObjectAttributes, ObjectAttributes::ptr>(
      m, "ObjectAttributes",
      R"(A metadata template for rigid objects pre-instantiation. Defines asset paths, physical
      properties, scale, semantic ids, shader type overrides, and user defined metadata.
      ManagedRigidObjects are instantiated from these blueprints. Can be imported from
      .object_config.json files.)")
      .def(py::init(&ObjectAttributes::create<>))
      .def(py::init(&ObjectAttributes::create<const std::string&>))
      .def_property(
          "com", &ObjectAttributes::getCOM, &ObjectAttributes::setCOM,
          R"(The Center of Mass for objects built from this template.)")
      .def_property(
          "compute_COM_from_shape", &ObjectAttributes::getComputeCOMFromShape,
          &ObjectAttributes::setComputeCOMFromShape,
          R"(Whether the COM should be calculated when an object is created
          based on its bounding box)")
      .def_property("mass", &ObjectAttributes::getMass,
                    &ObjectAttributes::setMass,
                    R"(The mass of objects constructed from this template.)")
      .def_property(
          "inertia", &ObjectAttributes::getInertia,
          &ObjectAttributes::setInertia,
          R"(The diagonal of the Intertia matrix for objects constructed
          from this template.)")
      .def_property(
          "linear_damping", &ObjectAttributes::getLinearDamping,
          &ObjectAttributes::setLinearDamping,
          R"(The damping of the linear velocity for objects constructed
          from this template.)")
      .def_property(
          "angular_damping", &ObjectAttributes::getAngularDamping,
          &ObjectAttributes::setAngularDamping,
          R"(The damping of angular velocity for objects constructed from
          this template.)")
      .def_property("bounding_box_collisions",
                    &ObjectAttributes::getBoundingBoxCollisions,
                    &ObjectAttributes::setBoundingBoxCollisions,
                    R"(Whether objects constructed from this template should use
          bounding box for collisions or designated mesh.)")
      .def_property(
          "join_collision_meshes", &ObjectAttributes::getJoinCollisionMeshes,
          &ObjectAttributes::setJoinCollisionMeshes,
          R"(Whether collision meshes for objects constructed from this
          template should be joined into a convex hull or kept separate.)")
      .def_property(
          "is_visibile", &ObjectAttributes::getIsVisible,
          &ObjectAttributes::setIsVisible,
          R"(Whether objects constructed from this template are visible.)")
      .def_property(
          "semantic_id", &ObjectAttributes::getSemanticId,
          &ObjectAttributes::setSemanticId,
          R"(The semantic ID for objects constructed from this template.)");

  // ==== StageAttributes ====
  py::class_<StageAttributes, AbstractObjectAttributes, StageAttributes::ptr>(
      m, "StageAttributes",
      R"(A metadata template for stages pre-instantiation. Defines asset paths, collision properties, gravity direction, shader type overrides, semantic asset information, and user defined metadata. Consumed to instantiate the static background of a scene (e.g. the building architecture). Can be imported from .stage_config.json files.)")
      .def(py::init(&StageAttributes::create<>))
      .def(py::init(&StageAttributes::create<const std::string&>))
      .def_property(
          "gravity", &StageAttributes::getGravity, &StageAttributes::setGravity,
          R"(The 3-vector representation of gravity to use for physically-based
          simulations on stages built from this template.)")
      .def_property(
          "origin", &StageAttributes::getOrigin, &StageAttributes::setOrigin,
          R"(The desired location of the origin of stages built from this
          template.)")
      .def_property(
          "semantic_orient_up", &StageAttributes::getSemanticOrientUp,
          &StageAttributes::setSemanticOrientUp,
          R"(Up direction for semantic stage meshes built from this template.)")
      .def_property(
          "semantic_orient_front", &StageAttributes::getSemanticOrientFront,
          &StageAttributes::setSemanticOrientFront,
          R"(Forward direction for semantic stage meshes built from this template.)")
      .def_property(
          "semantic_asset_handle", &StageAttributes::getSemanticAssetHandle,
          &StageAttributes::setSemanticAssetHandle,
          R"(Handle of the asset used for semantic segmentation of stages
          built from this template.)")
      .def_property(
          "semantic_asset_type", &StageAttributes::getSemanticAssetType,
          &StageAttributes::setSemanticAssetType,
          R"(Type of asset used for collision calculations for constructions
          built from this template.)")
      .def_property(
          "navmesh_asset_handle", &StageAttributes::getNavmeshAssetHandle,
          &StageAttributes::setNavmeshAssetHandle,
          R"(Handle of the navmesh asset used for constructions built from
          this template.)")
      .def_property(
          "house_filename", &StageAttributes::getSemanticDescriptorFilename,
          &StageAttributes::setSemanticDescriptorFilename,
          R"(Handle for file containing semantic type maps and hierarchy for
          constructions built from this template.)")
      .def_property(
          "frustum_culling", &StageAttributes::getFrustumCulling,
          &StageAttributes::setFrustumCulling,
          R"(Whether frustum culling should be enabled for constructions built by this template.)");

  // ==== LightInstanceAttributes ====
  py::class_<LightInstanceAttributes, AbstractAttributes,
             LightInstanceAttributes::ptr>(
      m, "LightInstanceAttributes",
      R"(A metadata template for light configurations. Supports point and directional lights. Can be imported from .lighting_config.json files.)")
      .def(py::init(&LightInstanceAttributes::create<>))
      .def(py::init(&LightInstanceAttributes::create<const std::string&>))
      .def_property(
          "position", &LightInstanceAttributes::getPosition,
          &LightInstanceAttributes::setPosition,
          R"(The 3-vector representation of the desired position of the light in the scene.)")
      .def_property(
          "direction", &LightInstanceAttributes::getDirection,
          &LightInstanceAttributes::setDirection,
          R"(The 3-vector representation of the desired direction of the light in the scene.)")
      .def_property(
          "color", &LightInstanceAttributes::getColor,
          &LightInstanceAttributes::setColor,
          R"(The 3-vector representation of the desired color of the light.)")
      .def_property("intensity", &LightInstanceAttributes::getIntensity,
                    &LightInstanceAttributes::setIntensity,
                    R"(The intensity to use for the light.)")
      .def_property("type", &LightInstanceAttributes::getType,
                    &LightInstanceAttributes::setType,
                    R"(The type of the light.)")
      .def_property(
          "spot_inner_cone_angle", &LightInstanceAttributes::getInnerConeAngle,
          &LightInstanceAttributes::setInnerConeAngle,
          R"(The inner cone angle to use for the dispersion of spot lights.
                    Ignored for other types of lights.)")
      .def_property(
          "spot_outer_cone_angle", &LightInstanceAttributes::getOuterConeAngle,
          &LightInstanceAttributes::setOuterConeAngle,
          R"(The outter cone angle to use for the dispersion of spot lights.
                    Ignored for other types of lights.)");

  // TODO : LightLayoutAttributes

  // ==== PbrShaderAttributes ====
  py::class_<PbrShaderAttributes, AbstractAttributes, PbrShaderAttributes::ptr>(
      m, "PbrShaderAttributes",
      R"(A metadata template for PBR shader creation and control values and multipliers,
      such as enabling Image Based Lighting and controlling the mix of direct and indirect
      lighting contributions. Can be imported from .pbr_config.json files.)")
      .def(py::init(&PbrShaderAttributes::create<>))
      .def(py::init(&PbrShaderAttributes::create<const std::string&>))
      .def_property(
          "enable_direct_lights", &PbrShaderAttributes::getEnableDirectLighting,
          &PbrShaderAttributes::setEnableDirectLighting,
          R"(Whether the specified direct lights are used to illuminate the scene.)")
      .def_property(
          "enable_ibl", &PbrShaderAttributes::getEnableIBL,
          &PbrShaderAttributes::setEnableIBL,
          R"(Whether Image-based Lighting is used to illuminate the scene.)")
      .def_property(
          "direct_light_intensity",
          &PbrShaderAttributes::getDirectLightIntensity,
          &PbrShaderAttributes::setDirectLightIntensity,
          R"(Sets the global direct lighting multiplier to control overall direct light
                brightness. This is used to balance PBR and Phong lighting of the same scene.
                Default value is 3.14)")
      .def_property(
          "skip_calc_missing_tbn", &PbrShaderAttributes::getSkipCalcMissingTBN,
          &PbrShaderAttributes::setSkipCalcMissingTBN,
          R"(Whether the fragment shader should skip the tangent frame calculation if precomputed
                tangents are not provided. This calculation provides a tangent frame to be used for
                normal textures and anisotropy calculations. If precomputed tangents are missing and
                this calculation is not enabled, any normal textures will be ignored, which will adversely
                affect visual fidelity.)")
      .def_property("use_mikkelsen_tbn_calc",
                    &PbrShaderAttributes::getUseMikkelsenTBN,
                    &PbrShaderAttributes::setUseMikkelsenTBN,
                    R"(Whether the more expensive calculation by Mikkelsen from
                https://jcgt.org/published/0009/03/04/paper.pdf should be used for the TBN calc. If
                false, a less expensive method based on
                https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag that gives
                empirically validated equivalent results will be used instead.)")
      .def_property(
          "map_mat_txtr_to_linear", &PbrShaderAttributes::getMapMatTxtrToLinear,
          &PbrShaderAttributes::setMapMatTxtrToLinear,
          R"(Whether we should use shader-based srgb->linear approximation remapping of applicable
                color textures in PBR rendering.)")
      .def_property(
          "map_ibl_txtr_to_linear", &PbrShaderAttributes::getMapIBLTxtrToLinear,
          &PbrShaderAttributes::setMapIBLTxtrToLinear,
          R"(Whether we should use shader-based srgb->linear approximation remapping of environment map
                textures used by IBL in PBR rendering.)")
      .def_property(
          "map_output_to_srgb", &PbrShaderAttributes::getMapOutputToSRGB,
          &PbrShaderAttributes::setMapOutputToSRGB,
          R"(Whether we should use shader-based linear->srgb approximation remapping of shader
                output in PBR rendering.)")
      .def_property(
          "use_direct_tonemap", &PbrShaderAttributes::getUseDirectLightTonemap,
          &PbrShaderAttributes::setUseDirectLightTonemap,
          R"(Whether tonemapping is enabled for direct lighting results, remapping the colors
                to a slightly different colorspace.)")
      .def_property_readonly(
          "ibl_brdfLUT_filename",
          &PbrShaderAttributes::getIBLBrdfLUTAssetHandle,
          R"(The filename or resource handle for the BRDF Lookup Table used for by the consumers of
                this config for image-based lighting.)")
      .def_property_readonly(
          "ibl_environment_map_filename",
          &PbrShaderAttributes::getIBLEnvMapAssetHandle,
          R"(The filename or resource handle for the Environment Map used by the
                consumers of this config for image-based lighting.)")
      .def_property(
          "use_ibl_tonemap", &PbrShaderAttributes::getUseIBLTonemap,
          &PbrShaderAttributes::setUseIBLTonemap,
          R"(Whether tonemapping is enabled for image-based lighting results, remapping the colors
                to a slightly different colorspace.)")
      .def_property(
          "use_burley_diffuse", &PbrShaderAttributes::getUseBurleyDiffuse,
          &PbrShaderAttributes::setUseBurleyDiffuse,
          R"(If true, the PBR shader uses a diffuse calculation based on Burley, modified to be
                more energy conserving.
          https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
                otherwise, the shader will use a standard Lambertian model, which is easier
                to calculate but doesn't look as nice, and sometimes can appear washed out.)")
      .def_property(
          "skip_clearcoat_calc",
          &PbrShaderAttributes::getSkipCalcClearcoatLayer,
          &PbrShaderAttributes::setSkipCalcClearcoatLayer,
          R"(Whether the clearcoat layer calculations should be skipped. If true, disables calcs
                regardless of material setting.)")
      .def_property(
          "skip_specular_layer_calc",
          &PbrShaderAttributes::getSkipCalcSpecularLayer,
          &PbrShaderAttributes::setSkipCalcSpecularLayer,
          R"(Whether the specular layer calculations should be skipped. If true, disables calcs
                regardless of material setting.)")
      .def_property(
          "skip_anisotropy_layer_calc",
          &PbrShaderAttributes::getSkipCalcAnisotropyLayer,
          &PbrShaderAttributes::setSkipCalcAnisotropyLayer,
          R"(Whether the anisotropy layer calculations should be skipped. If true, disables calcs
                regardless of material setting.)")
      .def_property(
          "ibl_to_direct_diffuse_balance",
          &PbrShaderAttributes::getIBLToDirectDiffuseBalance,
          &PbrShaderAttributes::setIBLToDirectDiffuseBalance,
          R"(The balance between the direct lighting and image-based lighting diffuse
                results, with value values of [0,1]. Any value <= 0 means only direct lighting diffuse
                results are rendered, >=1 means only image-based lighting results are rendered. Only
                used when both direct and image-basedlighting is present)")
      .def_property(
          "ibl_to_direct_specular_balance",
          &PbrShaderAttributes::getIBLToDirectSpecularBalance,
          &PbrShaderAttributes::setIBLToDirectSpecularBalance,
          R"(The balance between the direct lighting and image-based lighting specular
                results, with value values of [0,1]. Any value <= 0 means only direct lighting specular
                results are rendered, >=1 means only image-based lighting results are rendered. Only
                used when both direct and image-basedlighting is present)")
      .def_property(
          "direct_diffuse_scale", &PbrShaderAttributes::getDirectDiffuseScale,
          &PbrShaderAttributes::setDirectDiffuseScale,
          R"(Directly manipulate the value of the direct lighting diffuse scale.
                Note, no range checking is performed on this value, so irrational results are possible
                if this value is set negative or greater than 1. Only used when both direct and
                image-basedlighting is present)")
      .def_property(
          "direct_specular_scale", &PbrShaderAttributes::getDirectSpecularScale,
          &PbrShaderAttributes::setDirectSpecularScale,
          R"(Directly manipulate the value of the direct lighting specular scale.
                Note, no range checking is performed on this value, so irrational results are possible
                if this value is set negative or greater than 1. Only used when both direct and
                image-basedlighting is present)")
      .def_property(
          "ibl_diffuse_scale", &PbrShaderAttributes::getIBLDiffuseScale,
          &PbrShaderAttributes::setIBLDiffuseScale,
          R"(Directly manipulate the value of the image-based lighting diffuse scale.
                Note, no range checking is performed on this value, so irrational results are possible
                if this value is set negative or greater than 1. Only used when both direct and
                image-basedlighting is present)")
      .def_property(
          "ibl_specular_scale", &PbrShaderAttributes::getIBLSpecularScale,
          &PbrShaderAttributes::setIBLSpecularScale,
          R"(Directly manipulate the value of the image-based lighting specular scale.
                Note, no range checking is performed on this value, so irrational results are possible
                if this value is set negative or greater than 1. Only used when both direct and
                image-basedlighting is present)")
      .def_property(
          "tonemap_exposure", &PbrShaderAttributes::getTonemapExposure,
          &PbrShaderAttributes::setTonemapExposure,
          R"(The exposure value for tonemapping in the pbr shader. This value scales the color before the
                tonemapping is applied. Default value is 4.5)")
      .def_property(
          "gamma", &PbrShaderAttributes::getGamma,
          &PbrShaderAttributes::setGamma,
          R"(The gamma value for the pbr shader. This value is used for the approximation
                mapping from sRGB to linear and back. Default value is 2.2)");

  // ==== PhysicsManagerAttributes ====
  py::class_<PhysicsManagerAttributes, AbstractAttributes,
             PhysicsManagerAttributes::ptr>(
      m, "PhysicsManagerAttributes",
      R"(A metadata template for Simulation parameters (e.g. timestep, simulation backend,
      default gravity direction) and defaults. Consumed to instace a Simulator object.
      Can be imported from .physics_config.json files.)")
      .def(py::init(&PhysicsManagerAttributes::create<>))
      .def(py::init(&PhysicsManagerAttributes::create<const std::string&>))
      .def_property_readonly(
          "simulator", &PhysicsManagerAttributes::getSimulator,
          R"(The simulator being used for dynamic simulation.  If none then only kinematic
        support is provided.)")
      .def_property("timestep", &PhysicsManagerAttributes::getTimestep,
                    &PhysicsManagerAttributes::setTimestep,
                    R"(The timestep to use for forward simulation.)")
      .def_property("max_substeps", &PhysicsManagerAttributes::getMaxSubsteps,
                    &PhysicsManagerAttributes::setMaxSubsteps,
                    R"(Maximum simulation steps between each rendering step.
                    (Not currently implemented).)")
      .def_property(
          "gravity", &PhysicsManagerAttributes::getGravity,
          &PhysicsManagerAttributes::setGravity,
          R"(The default 3-vector representation of gravity to use for physically-based
          simulations.  Can be overridden.)")
      .def_property(
          "friction_coefficient",
          &PhysicsManagerAttributes::getFrictionCoefficient,
          &PhysicsManagerAttributes::setFrictionCoefficient,
          R"(Default friction coefficient for contact modeling.  Can be overridden by
          stage and object values.)")
      .def_property(
          "restitution_coefficient",
          &PhysicsManagerAttributes::getRestitutionCoefficient,
          &PhysicsManagerAttributes::setRestitutionCoefficient,
          R"(Default restitution coefficient for contact modeling.  Can be overridden by
          stage and object values.)");

  // ==== AbstractPrimitiveAttributes ====
  py::class_<AbstractPrimitiveAttributes, AbstractAttributes,
             AbstractPrimitiveAttributes::ptr>(m, "AbstractPrimitiveAttributes")
      .def_property_readonly(
          "is_wireframe", &AbstractPrimitiveAttributes::getIsWireframe,
          R"(Whether primitives built from this template are wireframe or solid.)")
      .def_property(
          "use_texture_coords",
          &AbstractPrimitiveAttributes::getUseTextureCoords,
          &AbstractPrimitiveAttributes::setUseTextureCoords,
          R"(Whether texture coordinates should be generated for objects
          constructed using this template.)")
      .def_property(
          "use_tangents", &AbstractPrimitiveAttributes::getUseTangents,
          &AbstractPrimitiveAttributes::setUseTangents,
          R"(Whether 4-component (homogeneous) tangents should be generated for
          objects constructed using this template.)")
      .def_property(
          "num_rings", &AbstractPrimitiveAttributes::getNumRings,
          &AbstractPrimitiveAttributes::setNumRings,
          R"(Number of line (for wireframe) or face (for solid) rings for
          primitives built from this template.
          Must be greater than 1 for template to be valid.
          For all uvSpheres, must be greater than 2, and for wireframe uvSpheres
          must also be multiple of 2.
          Used by solid cones, cylinders, uvSpheres, and wireframe cylinders
          and uvSpheres)")
      .def_property(
          "num_segments", &AbstractPrimitiveAttributes::getNumSegments,
          &AbstractPrimitiveAttributes::setNumSegments,
          R"(Number of line (for wireframe) or face (for solid) segments
          for primitives built from this template.
          For solid primitives, must be 3 or greater for template to
          be valid. For wireframe primitives, must be 4 or greater,
          and a multiple of 4 for template to be valid.
          Used by solid and wireframe capsules, cones, cylinders,
          uvSpheres.)")
      .def_property(
          "half_length", &AbstractPrimitiveAttributes::getHalfLength,
          &AbstractPrimitiveAttributes::setHalfLength,
          R"(Half the length of the cylinder (for capsules and cylinders) or the
          cone (for cones) primitives built from this template. Primitives is
          built with default radius 1.0.  In order to get a desired radius r,
          length l, and preserve correct normals of the primitive being built,
          set half_length to .5 * (l/r) and then scale by r.
          Used by solid and wireframe capsules, cones and cylinders.)")
      .def_property_readonly(
          "prim_obj_class_name",
          &AbstractPrimitiveAttributes::getPrimObjClassName,
          R"(Name of Magnum primitive class this template uses to construct
          primitives)")
      .def_property_readonly("prim_obj_type",
                             &AbstractPrimitiveAttributes::getPrimObjType)
      .def("build_handle", &AbstractPrimitiveAttributes::buildHandle)
      .def_property_readonly(
          "is_valid_template", &AbstractPrimitiveAttributes::isValidTemplate,
          R"(Certain attributes properties, such as num_segments, are subject
          to restrictions in allowable values. If an illegal value is entered,
          the template is considered invalid and no primitive will be built
          from it.  This property will say whether this template is valid
          for creating its designated primitive.)");

  // ==== CapsulePrimitiveAttributes ====
  py::class_<CapsulePrimitiveAttributes, AbstractPrimitiveAttributes,
             CapsulePrimitiveAttributes::ptr>(
      m, "CapsulePrimitiveAttributes",
      R"(Parameters for constructing a primitive capsule mesh shape.)")
      .def(py::init(
          &CapsulePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property(
          "hemisphere_rings", &CapsulePrimitiveAttributes::getHemisphereRings,
          &CapsulePrimitiveAttributes::setHemisphereRings,
          R"(Number of rings for each hemisphere for capsules built with this
          template.  Must be larger than 1 for template to be valid.)")
      .def_property(
          "cylinder_rings", &CapsulePrimitiveAttributes::getCylinderRings,
          &CapsulePrimitiveAttributes::setCylinderRings,
          R"(Number of rings for cylinder body for capsules built with this
          template.  Must be larger than 1 for template to be valid.)");

  // ==== ConePrimitiveAttributes ====
  py::class_<ConePrimitiveAttributes, AbstractPrimitiveAttributes,
             ConePrimitiveAttributes::ptr>(
      m, "ConePrimitiveAttributes",
      R"(Parameters for constructing a primitive cone mesh shape.)")
      .def(py::init(
          &ConePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property(
          "use_cap_end", &ConePrimitiveAttributes::getCapEnd,
          &ConePrimitiveAttributes::setCapEnd,
          R"(Whether to close cone bottom.  Only used for solid cones.)");

  // ==== CubePrimitiveAttributes ====
  py::class_<CubePrimitiveAttributes, AbstractPrimitiveAttributes,
             CubePrimitiveAttributes::ptr>(
      m, "CubePrimitiveAttributes",
      R"(Parameters for constructing a primitive cube mesh shape.)")
      .def(py::init(
          &CubePrimitiveAttributes::create<bool, int, const std::string&>));

  // ==== CylinderPrimitiveAttributes ====
  py::class_<CylinderPrimitiveAttributes, AbstractPrimitiveAttributes,
             CylinderPrimitiveAttributes::ptr>(
      m, "CylinderPrimitiveAttributes",
      R"(Parameters for constructing a primitive capsule mesh shape.)")
      .def(py::init(
          &CylinderPrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property(
          "use_cap_ends", &CylinderPrimitiveAttributes::getCapEnds,
          &CylinderPrimitiveAttributes::setCapEnds,
          R"(Whether to close cylinder ends.  Only used for solid cylinders.)");

  // ==== IcospherePrimitiveAttributes ====
  py::class_<IcospherePrimitiveAttributes, AbstractPrimitiveAttributes,
             IcospherePrimitiveAttributes::ptr>(
      m, "IcospherePrimitiveAttributes",
      R"(Parameters for constructing a primitive icosphere mesh shape.)")
      .def(py::init(
          &IcospherePrimitiveAttributes::create<bool, int, const std::string&>))
      .def_property(
          "subdivisions", &IcospherePrimitiveAttributes::getSubdivisions,
          &IcospherePrimitiveAttributes::setSubdivisions,
          R"(Number of subdivisions to divide mesh for icospheres made from this
          template.  Only used with solid icospheres.)");

  // ==== UVSpherePrimitiveAttributes ====
  py::class_<UVSpherePrimitiveAttributes, AbstractPrimitiveAttributes,
             UVSpherePrimitiveAttributes::ptr>(
      m, "UVSpherePrimitiveAttributes",
      R"(Parameters for constructing a primitive uvsphere mesh shape.)")
      .def(py::init(
          &UVSpherePrimitiveAttributes::create<bool, int, const std::string&>));

}  // initAttributesBindings

}  // namespace metadata
}  // namespace esp
