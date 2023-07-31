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
  setDebugRenderPrimitives(false);
}  // ArticulatedObjectAttributes ctor

void ArticulatedObjectAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("urdf_filepath", jsonObj, allocator);
  writeValueToJson("render_asset", jsonObj, allocator);
  writeValueToJson("semantic_id", jsonObj, allocator);
  writeValueToJson("debug_render_primitives", jsonObj, allocator);
}  // ArticulatedObjectAttributes::writeValuesToJson

std::string ArticulatedObjectAttributes::getObjectInfoHeaderInternal() const {
  return "URDF Filepath,Render Asset,Semantic ID,Debug RenderPrimitives,";
}

std::string ArticulatedObjectAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{}", getURDFPath(), getRenderAssetHandle(),
      getAsString("semantic_id"), getAsString("debug_render_primitives"));
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
