// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

AbstractObjectAttributes::AbstractObjectAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setScale({1.0, 1.0, 1.0});
  setCollisionAssetSize({1.0, 1.0, 1.0});
  setMargin(0.04);
  setOrientUp({0, 1, 0});
  setOrientFront({0, 0, -1});
  // default rendering and collisions will be mesh for physics objects and
  // scenes. Primitive-based objects do not currently support mesh collisions,
  // however, due to issues with how non-triangle meshes (i.e. wireframes) are
  // handled in @ref GenericMeshData::setMeshData
  setRenderAssetIsPrimitive(false);
  setCollisionAssetIsPrimitive(false);
  setUseMeshCollision(true);
  setIsCollidable(true);
  setIsVisible(true);
  setUnitsToMeters(1.0);
  setRenderAssetHandle("");
  setCollisionAssetHandle("");
}  // AbstractObjectAttributes ctor

std::string AbstractObjectAttributes::getObjectInfoHeaderInternal() const {
  return "Render Asset Handle,Collision Asset Handle,Scale,Margin,Up XYZ,"
         "Front XYZ,Units to M,Friction Coefficient,Restitution "
         "Coefficient,Current Shader Type," +
         getAbstractObjectInfoHeaderInternal();
}

std::string AbstractObjectAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{},{},{},{}", getRenderAssetHandle(),
      getCollisionAssetHandle(), getAsString("scale"), getAsString("margin"),
      getAsString("orient_up"), getAsString("orient_front"),
      getAsString("units_to_meters"), getAsString("friction_coefficient"),
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
  writeValueToJson("orient_up", "up", jsonObj, allocator);
  writeValueToJson("orient_front", "front", jsonObj, allocator);
  writeValueToJson("units_to_meters", jsonObj, allocator);
  writeValueToJson("is_visible", jsonObj, allocator);
  writeValueToJson("friction_coefficient", jsonObj, allocator);
  writeValueToJson("restitution_coefficient", jsonObj, allocator);
  writeValueToJson("render_asset", jsonObj, allocator);
  writeValueToJson("collision_asset", jsonObj, allocator);
  writeValueToJson("collision_asset_size", jsonObj, allocator);
  writeValueToJson("shader_type", jsonObj, allocator);
  writeValueToJson("force_flat_shading", jsonObj, allocator);

  // call instance-specific
  writeValuesToJsonInternal(jsonObj, allocator);
}  // AbstractObjectAttributes::writeValuesToJson

ObjectAttributes::ObjectAttributes(const std::string& handle)
    : AbstractObjectAttributes("ObjectAttributes", handle) {
  // fill necessary attribute defaults
  setMass(1.0);
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setLinearDamping(0.2);
  setAngularDamping(0.2);

  setComputeCOMFromShape(true);

  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  // default to use material-derived shader unless otherwise specified in config
  // or instance config
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Material));
  // TODO remove this once ShaderType support is complete
  setForceFlatShading(false);
  setIsVisible(true);
  setSemanticId(0);
}  // ObjectAttributes ctor

std::string ObjectAttributes::getAbstractObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{}", getAsString("mass"), getAsString("COM"),
      getAsString("inertia"), getAsString("angular_damping"),
      getAsString("linear_damping"), getAsString("semantic_id"));
}

void ObjectAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write ObjectAttributes values to json
  writeValueToJson("mass", jsonObj, allocator);
  writeValueToJson("linear_damping", jsonObj, allocator);
  writeValueToJson("angular_damping", jsonObj, allocator);
  writeValueToJson("use_bounding_box_for_collision", jsonObj, allocator);
  writeValueToJson("COM", jsonObj, allocator);
  writeValueToJson("inertia", jsonObj, allocator);
  writeValueToJson("semantic_id", jsonObj, allocator);
  writeValueToJson("join_collision_meshes", jsonObj, allocator);

}  // ObjectAttributes::writeValuesToJsonInternal

StageAttributes::StageAttributes(const std::string& handle)
    : AbstractObjectAttributes("StageAttributes", handle) {
  setGravity({0, -9.8, 0});
  setOrigin({0, 0, 0});
  // default to use material-derived shader unless otherwise specified in config
  // or instance config
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Material));
  // TODO remove this once ShaderType support is complete
  setForceFlatShading(true);
  // 0 corresponds to esp::assets::AssetType::UNKNOWN->treated as general mesh
  setCollisionAssetType(static_cast<int>(esp::assets::AssetType::UNKNOWN));
  // 4 corresponds to esp::assets::AssetType::INSTANCE_MESH
  setSemanticAssetType(static_cast<int>(esp::assets::AssetType::INSTANCE_MESH));
  // set empty defaults for handles
  set("nav_asset", "");
  set("semantic_asset", "");
  set("semantic_descriptor_filename", "");
}  // StageAttributes ctor

void StageAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("origin", jsonObj, allocator);
  writeValueToJson("gravity", jsonObj, allocator);
  writeValueToJson("semantic_asset", jsonObj, allocator);
  writeValueToJson("nav_asset", jsonObj, allocator);
  writeValueToJson("semantic_descriptor_filename", jsonObj, allocator);

}  // StageAttributes::writeValuesToJsonInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
