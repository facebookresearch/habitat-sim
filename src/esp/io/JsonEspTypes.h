// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONESPTYPES_H_
#define ESP_IO_JSONESPTYPES_H_

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

inline JsonGenericValue toJsonValue(const esp::vec3f& vec,
                                    JsonAllocator& allocator) {
  JsonGenericValue floatsArray(rapidjson::kArrayType);
  for (int i = 0; i < vec.size(); i++) {
    floatsArray.PushBack(vec.data()[i], allocator);
  }
  return floatsArray;
}

inline bool fromJsonValue(const JsonGenericValue& floatsArray,
                          esp::vec3f& vec) {
  // TODO: helper for objects that are simply arrays, with error-handling
  ASSERT(floatsArray.Size() == vec.size());
  for (int i = 0; i < vec.size(); i++) {
    vec.data()[i] = floatsArray[i].GetFloat();
  }
  return true;
}

inline JsonGenericValue toJsonValue(const esp::geo::CoordinateFrame& frame,
                                    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "up", frame.up(), allocator);
  addMember(obj, "front", frame.front(), allocator);
  addMember(obj, "origin", frame.origin(), allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::geo::CoordinateFrame& frame) {
  esp::vec3f up;
  esp::vec3f front;
  esp::vec3f origin;
  readMember(obj, "up", up);
  readMember(obj, "front", front);
  readMember(obj, "origin", origin);
  frame = esp::geo::CoordinateFrame(up, front, origin);
  return true;
}

inline JsonGenericValue toJsonValue(const esp::assets::AssetInfo& x,
                                    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMemberAsUint32(obj, "type", x.type, allocator);
  addMember(obj, "filepath", x.filepath, allocator);
  addMember(obj, "frame", x.frame, allocator);
  addMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters, allocator);
  addMember(obj, "requiresLighting", x.requiresLighting, allocator);
  addMember(obj, "splitInstanceMesh", x.splitInstanceMesh, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::assets::AssetInfo& x) {
  readMemberAsUint32(obj, "type", x.type);
  readMember(obj, "filepath", x.filepath);
  readMember(obj, "frame", x.frame);
  readMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters);
  readMember(obj, "requiresLighting", x.requiresLighting);
  readMember(obj, "splitInstanceMesh", x.splitInstanceMesh);
  return true;
}

inline JsonGenericValue toJsonValue(
    const esp::assets::RenderAssetInstanceCreationInfo& x,
    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "filepath", x.filepath, allocator);
  addMember(obj, "scale", x.scale, allocator);
  addMember(obj, "isStatic", x.isStatic(), allocator);
  addMember(obj, "isRGBD", x.isRGBD(), allocator);
  addMember(obj, "isSemantic", x.isSemantic(), allocator);
  addMember(obj, "lightSetupKey", x.lightSetupKey, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::assets::RenderAssetInstanceCreationInfo& x) {
  readMember(obj, "filepath", x.filepath);
  readMember(obj, "scale", x.scale);
  bool isStatic;
  readMember(obj, "isStatic", isStatic);
  bool isRGBD;
  readMember(obj, "isRGBD", isRGBD);
  bool isSemantic;
  readMember(obj, "isSemantic", isSemantic);
  if (isStatic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsStatic;
  }
  if (isRGBD) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  }
  if (isSemantic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  }
  readMember(obj, "lightSetupKey", x.lightSetupKey);
  return true;
}

inline JsonGenericValue toJsonValue(const esp::gfx::replay::Transform& x,
                                    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "translation", x.translation, allocator);
  addMember(obj, "rotation", x.rotation, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::gfx::replay::Transform& x) {
  readMember(obj, "translation", x.translation);
  readMember(obj, "rotation", x.rotation);
  return true;
}

inline JsonGenericValue toJsonValue(
    const esp::gfx::replay::RenderAssetInstanceState& x,
    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "absTransform", x.absTransform, allocator);
  addMember(obj, "semanticId", x.semanticId, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::gfx::replay::RenderAssetInstanceState& x) {
  readMember(obj, "absTransform", x.absTransform);
  readMember(obj, "semanticId", x.semanticId);
  return true;
}

JsonGenericValue toJsonValue(const esp::gfx::replay::Keyframe& x,
                             JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& keyframeObj,
                   esp::gfx::replay::Keyframe& keyframe);

}  // namespace io
}  // namespace esp

#endif
