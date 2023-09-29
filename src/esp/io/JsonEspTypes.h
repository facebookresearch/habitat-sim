// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
#include "esp/core/Esp.h"
#include "esp/gfx/replay/Keyframe.h"
#include "esp/nav/PathFinder.h"

namespace esp {
namespace io {

inline JsonGenericValue toJsonValue(const esp::vec3f& vec,
                                    JsonAllocator& allocator) {
  return toJsonArrayHelper(vec.data(), 3, allocator);
}

inline bool fromJsonValue(const JsonGenericValue& obj, esp::vec3f& val) {
  if (obj.IsArray() && obj.Size() == 3) {
    for (rapidjson::SizeType i = 0; i < 3; ++i) {
      if (obj[i].IsNumber()) {
        val[i] = obj[i].GetDouble();
      } else {
        ESP_ERROR() << "Invalid numeric value specified in JSON vec3f, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

inline JsonGenericValue toJsonValue(
    const esp::assets::PhongMaterialColor& material,
    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "ambient", material.ambientColor, allocator);
  addMember(obj, "diffuse", material.diffuseColor, allocator);
  addMember(obj, "specular", material.specularColor, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::assets::PhongMaterialColor& material) {
  bool success = true;
  success &= readMember(obj, "ambient", material.ambientColor);
  success &= readMember(obj, "diffuse", material.diffuseColor);
  success &= readMember(obj, "specular", material.specularColor);
  return success;
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
  bool success = true;
  esp::vec3f up;
  esp::vec3f front;
  esp::vec3f origin;
  success &= readMember(obj, "up", up);
  success &= readMember(obj, "front", front);
  success &= readMember(obj, "origin", origin);
  frame = esp::geo::CoordinateFrame(up, front, origin);
  return success;
}

JsonGenericValue toJsonValue(const esp::assets::AssetInfo& x,
                             JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, esp::assets::AssetInfo& x);

JsonGenericValue toJsonValue(
    const metadata::attributes::ObjectInstanceShaderType& x,
    JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj,
                   metadata::attributes::ObjectInstanceShaderType& x);

inline JsonGenericValue toJsonValue(
    const esp::assets::RenderAssetInstanceCreationInfo& x,
    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "filepath", x.filepath, allocator);
  addMember(obj, "scale", x.scale, allocator);
  addMember(obj, "isStatic", x.isStatic(), allocator);
  addMember(obj, "isRGBD", x.isRGBD(), allocator);
  addMember(obj, "isSemantic", x.isSemantic(), allocator);
  addMember(obj, "isTextureSemantic", x.isTextureBasedSemantic(), allocator);
  addMember(obj, "lightSetupKey", x.lightSetupKey, allocator);
  addMember(obj, "rigId", x.rigId, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj,
                          esp::assets::RenderAssetInstanceCreationInfo& x) {
  readMember(obj, "filepath", x.filepath);
  readMember(obj, "scale", x.scale);
  bool isStatic = false;
  readMember(obj, "isStatic", isStatic);
  bool isRGBD = false;
  readMember(obj, "isRGBD", isRGBD);
  bool isSemantic = false;
  readMember(obj, "isSemantic", isSemantic);
  bool isTextureSemantic = false;
  readMember(obj, "isTextureSemantic", isTextureSemantic);
  if (isStatic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsStatic;
  }
  if (isRGBD) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  }
  if (isSemantic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  }
  if (isTextureSemantic) {
    x.flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::
        IsTextureBasedSemantic;
  }

  readMember(obj, "lightSetupKey", x.lightSetupKey);
  readMember(obj, "rigId", x.rigId);
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
  bool success = true;
  success &= readMember(obj, "translation", x.translation);
  success &= readMember(obj, "rotation", x.rotation);
  return success;
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

inline JsonGenericValue toJsonValue(const esp::gfx::LightInfo& x,
                                    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "vector", x.vector, allocator);
  addMember(obj, "color", x.color, allocator);
  addMember(obj, "model", x.model, allocator);
  return obj;
}

inline bool fromJsonValue(const JsonGenericValue& obj, esp::gfx::LightInfo& x) {
  readMember(obj, "vector", x.vector);
  readMember(obj, "color", x.color);
  readMember(obj, "model", x.model);
  return true;
}

JsonGenericValue toJsonValue(const esp::gfx::LightPositionModel& x,
                             JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj,
                   esp::gfx::LightPositionModel& x);

JsonGenericValue toJsonValue(const esp::gfx::replay::Keyframe& x,
                             JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& keyframeObj,
                   esp::gfx::replay::Keyframe& keyframe);

// NavMeshSettings JSON serialization
JsonGenericValue toJsonValue(const esp::nav::NavMeshSettings& x,
                             JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, esp::nav::NavMeshSettings& x);

}  // namespace io
}  // namespace esp

#endif
