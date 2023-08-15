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
  setFrictionCoefficient(0.5);
  setRollingFrictionCoefficient(0.0);
  setSpinningFrictionCoefficient(0.0);
  setRestitutionCoefficient(0.1);
  setScale({1.0, 1.0, 1.0});
  setCollisionAssetSize({1.0, 1.0, 1.0});
  setMargin(0.04);
  setOrientUp({0, 1, 0});
  setOrientFront({0, 0, -1});
  setUseFrameForAllOrientation(true);
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
  writeValueToJson("orient_up", "up", jsonObj, allocator);
  writeValueToJson("orient_front", "front", jsonObj, allocator);
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

  // call instance-specific
  writeValuesToJsonInternal(jsonObj, allocator);
}  // AbstractObjectAttributes::writeValuesToJson

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
