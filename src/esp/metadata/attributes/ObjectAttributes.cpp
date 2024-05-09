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
  init("mass", 1.0);
  init("COM", Mn::Vector3{0.0, 0.0, 0.0});
  init("inertia", Mn::Vector3{0.0, 0.0, 0.0});
  init("linear_damping", 0.2);
  init("angular_damping", 0.2);

  setComputeCOMFromShape(true);

  init("use_bounding_box_for_collision", false);
  init("join_collision_meshes", false);

  init("semantic_id", 0);
}  // ObjectAttributes ctor

std::string ObjectAttributes::getAbstractObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getAsString("mass"), getAsString("COM"),
      getAsString("inertia"), getAsString("angular_damping"),
      getAsString("linear_damping"),
      getAsString("use_bounding_box_for_collision"),
      getAsString("join_collision_meshes"), getAsString("semantic_id"));
}

void ObjectAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write ObjectAttributes values to json
  writeValueToJson("mass", jsonObj, allocator);
  writeValueToJson("COM", jsonObj, allocator);
  writeValueToJson("inertia", jsonObj, allocator);
  writeValueToJson("linear_damping", jsonObj, allocator);
  writeValueToJson("angular_damping", jsonObj, allocator);
  writeValueToJson("use_bounding_box_for_collision", jsonObj, allocator);
  writeValueToJson("join_collision_meshes", jsonObj, allocator);
  writeValueToJson("semantic_id", jsonObj, allocator);

}  // ObjectAttributes::writeValuesToJsonInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
