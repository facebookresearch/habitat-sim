// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ArticulatedObjectAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

ArticulatedObjectAttributes::ArticulatedObjectAttributes(
    const std::string& handle)
    : AbstractAttributes("ArticulatedObjectAttributes", handle) {
  setURDFPath(handle);
  setRenderAssetHandle("");
  setSemanticId(0);
  // Set default to be to use Phong shader
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Phong));
  // Set render mode to be default
  setRenderMode(getAORenderModeName(ArticulatedObjectRenderMode::Default));
}  // ArticulatedObjectAttributes ctor

void ArticulatedObjectAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("urdf_filepath", jsonObj, allocator);
  writeValueToJson("render_asset", jsonObj, allocator);
  writeValueToJson("semantic_id", jsonObj, allocator);
  writeValueToJson("shader_type", jsonObj, allocator);
  writeValueToJson("render_mode", jsonObj, allocator);
}  // ArticulatedObjectAttributes::writeValuesToJson

std::string ArticulatedObjectAttributes::getObjectInfoHeaderInternal() const {
  return "URDF Filepath,Render Asset,Semantic ID,Current Shader Type,Render "
         "Mode,";
}

std::string ArticulatedObjectAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{}", getURDFPath(), getRenderAssetHandle(),
      getAsString("semantic_id"), getShaderTypeName(getShaderType()),
      getAORenderModeName(getRenderMode()));
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
