// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

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
  setJoinCollisionMeshes(false);
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

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
