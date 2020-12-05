// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "JsonBuiltinTypes.h"
#include "JsonMagnumTypes.h"

#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/core/esp.h"
#include "esp/gfx/replay/Keyframe.h"

namespace esp {
namespace io {

inline RJsonValue ToRJsonValue(const esp::vec3f& vec,
                               RJsonAllocator& allocator) {
  RJsonValue floatsArray(rapidjson::kArrayType);
  for (int i = 0; i < vec.size(); i++) {
    floatsArray.PushBack(vec.data()[i], allocator);
  }
  return floatsArray;
}

inline void FromRJsonValue(const RJsonValue& floatsArray, esp::vec3f& vec) {
  // TODO: helper for objects that are simply arrays, with error-handling
  ASSERT(floatsArray.Size() == vec.size());
  for (int i = 0; i < vec.size(); i++) {
    vec.data()[i] = floatsArray[i].GetFloat();
  }
}

inline RJsonValue ToRJsonValue(const esp::geo::CoordinateFrame& frame,
                               RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "up", frame.up(), allocator);
  AddMember(obj, "front", frame.front(), allocator);
  AddMember(obj, "origin", frame.origin(), allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj,
                           esp::geo::CoordinateFrame& frame) {
  esp::vec3f up;
  esp::vec3f front;
  esp::vec3f origin;
  ReadMember(obj, "up", up);
  ReadMember(obj, "front", front);
  ReadMember(obj, "origin", origin);
  frame = esp::geo::CoordinateFrame(up, front, origin);
}

inline RJsonValue ToRJsonValue(const esp::assets::AssetInfo& x,
                               RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMemberAsUint32(obj, "type", x.type, allocator);
  AddMember(obj, "filepath", x.filepath, allocator);
  AddMember(obj, "frame", x.frame, allocator);
  AddMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters, allocator);
  AddMember(obj, "requiresLighting", x.requiresLighting, allocator);
  AddMember(obj, "splitInstanceMesh", x.splitInstanceMesh, allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj, esp::assets::AssetInfo& x) {
  ReadMemberAsUint32(obj, "type", x.type);
  ReadMember(obj, "filepath", x.filepath);
  ReadMember(obj, "frame", x.frame);
  ReadMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters);
  ReadMember(obj, "requiresLighting", x.requiresLighting);
  ReadMember(obj, "splitInstanceMesh", x.splitInstanceMesh);
}

inline RJsonValue ToRJsonValue(
    const esp::assets::RenderAssetInstanceCreationInfo& x,
    RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "filepath", x.filepath, allocator);
  AddMember(obj, "scale", x.scale, allocator);
  AddMember(obj, "isStatic", x.isStatic(), allocator);
  AddMember(obj, "isRGBD", x.isRGBD(), allocator);
  AddMember(obj, "isSemantic", x.isSemantic(), allocator);
  AddMember(obj, "lightSetupKey", x.lightSetupKey, allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj,
                           esp::assets::RenderAssetInstanceCreationInfo& x) {
  ReadMember(obj, "filepath", x.filepath);
  ReadMember(obj, "scale", x.scale);
  bool isStatic;
  ReadMember(obj, "isStatic", isStatic);
  bool isRGBD;
  ReadMember(obj, "isRGBD", isRGBD);
  bool isSemantic;
  ReadMember(obj, "isSemantic", isSemantic);
  if (isStatic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsStatic;
  }
  if (isRGBD) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  }
  if (isSemantic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  }
  ReadMember(obj, "lightSetupKey", x.lightSetupKey);
}

inline RJsonValue ToRJsonValue(const esp::gfx::replay::Transform& x,
                               RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "translation", x.translation, allocator);
  AddMember(obj, "rotation", x.rotation, allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj,
                           esp::gfx::replay::Transform& x) {
  ReadMember(obj, "translation", x.translation);
  ReadMember(obj, "rotation", x.rotation);
}

inline RJsonValue ToRJsonValue(
    const esp::gfx::replay::RenderAssetInstanceState& x,
    RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "absTransform", x.absTransform, allocator);
  AddMember(obj, "semanticId", x.semanticId, allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj,
                           esp::gfx::replay::RenderAssetInstanceState& x) {
  ReadMember(obj, "absTransform", x.absTransform);
  ReadMember(obj, "semanticId", x.semanticId);
}

}  // namespace io
}  // namespace esp
