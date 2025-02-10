// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractObjectAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

AbstractObjectAttributes::AbstractObjectAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  init("friction_coefficient", 0.5);
  init("rolling_friction_coefficient", 0.0);
  init("spinning_friction_coefficient", 0.0);
  init("restitution_coefficient", 0.1);
  init("scale", Mn::Vector3{1.0, 1.0, 1.0});
  init("collision_asset_size", Mn::Vector3{1.0, 1.0, 1.0});
  init("margin", 0.04);
  init("up", Mn::Vector3{0.0, 1.0, 0.0});
  init("front", Mn::Vector3{0.0, 0.0, -1.0});
  // Set this to true so that only used if actually changed.
  // Hidden field
  setUseFrameForAllOrientation(true);
  // default to use material-derived shader unless otherwise specified in config
  // or instance config
  initTranslated("shader_type",
                 getShaderTypeName(ObjectInstanceShaderType::Material));
  // TODO remove this once ShaderType support is complete
  init("force_flat_shading", false);

  // Default rendering and collisions will be mesh for physics objects and
  // scenes. Primitive-based objects do not currently support mesh collisions,
  // however, due to issues with how non-triangle meshes (i.e. wireframes) are
  // handled in @ref GenericMeshData::setMeshData

  // Hidden field
  setRenderAssetIsPrimitive(false);
  // Hidden field
  setCollisionAssetIsPrimitive(false);
  init("use_mesh_collision", true);
  init("is_collidable", true);
  init("is_visible", true);
  init("units_to_meters", 1.0);
  init("render_asset", "");
  init("collision_asset", "");
  // initialize these so they exist in Configuration
  setHidden("__renderAssetFullPath", "");
  setHidden("__collisionAssetFullPath", "");
  // This specifies that we want to investigate the state of the render and
  // collision handles before we allow this attributes to be registered.
  // Hidden field
  setFilePathsAreDirty();
  // set up an existing subgroup for marker_sets attributes
  addOrEditSubgroup<MarkerSets>("marker_sets");
}  // AbstractObjectAttributes ctor

std::string AbstractObjectAttributes::getObjectInfoHeaderInternal() const {
  return "Render Asset Handle,Collision Asset Handle,Scale,Margin,Up XYZ,"
         "Front XYZ,Units to M,Friction Coefficient,Rolling "
         "Coefficient,Spinning Coefficient,Restitution "
         "Coefficient,Current Shader Type," +
         getAbstractObjectInfoHeaderInternal();
}

std::string AbstractObjectAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{},{},{},{}", getRenderAssetHandle(),
      getCollisionAssetHandle(), getAsString("scale"), getAsString("margin"),
      getAsString("up"), getAsString("front"), getAsString("units_to_meters"),
      getAsString("friction_coefficient"),
      getAsString("rolling_friction_coefficient"),
      getAsString("spinning_friction_coefficient"),
      getAsString("restitution_coefficient"),
      getShaderTypeName(getShaderType()), getAbstractObjectInfoInternal());
}  // AbstractObjectAttributes::getObjectInfoInternal

void AbstractObjectAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractObjectAttributes values to json
  writeValueToJson("scale", jsonObj, allocator);
  writeValueToJson("margin", jsonObj, allocator);
  writeValueToJson("is_collidable", jsonObj, allocator);
  writeValueToJson("up", jsonObj, allocator);
  writeValueToJson("front", jsonObj, allocator);
  writeValueToJson("units_to_meters", jsonObj, allocator);
  writeValueToJson("is_visible", jsonObj, allocator);
  writeValueToJson("friction_coefficient", jsonObj, allocator);
  writeValueToJson("rolling_friction_coefficient", jsonObj, allocator);
  writeValueToJson("spinning_friction_coefficient", jsonObj, allocator);
  writeValueToJson("restitution_coefficient", jsonObj, allocator);
  writeValueToJson("render_asset", jsonObj, allocator);
  writeValueToJson("collision_asset", jsonObj, allocator);
  writeValueToJson("collision_asset_size", jsonObj, allocator);
  writeValueToJson("shader_type", jsonObj, allocator);
  writeValueToJson("force_flat_shading", jsonObj, allocator);

  // Configuration::writeValuesToJson(jsonObj, allocator);

  // call child-class-specific
  writeValuesToJsonInternal(jsonObj, allocator);
}  // AbstractObjectAttributes::writeValuesToJson

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
