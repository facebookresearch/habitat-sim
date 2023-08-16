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
  // Set the default base type to be free joint
  setBaseType(getAOBaseTypeName(ArticulatedObjectBaseType::Free));
  // Set the default source for the interia calculation to be computed
  setInertiaSource(
      getAOInertiaSourceName(ArticulatedObjectInertiaSource::Computed));
  // Set the default link order to use as the tree traversal order
  setLinkOrder(getAOLinkOrderName(ArticulatedObjectLinkOrder::TreeTraversal));

  // Set render mode to be default - skin if present, otherwise link
  // meshes/primitives
  setRenderMode(getAORenderModeName(ArticulatedObjectRenderMode::Default));
  // Set default to be to use Phong shader
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Phong));

  setUniformScale(1.0f);
  setMassScale(1.0);

}  // ArticulatedObjectAttributes ctor

void ArticulatedObjectAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("urdf_filepath", jsonObj, allocator);
  writeValueToJson("render_asset", jsonObj, allocator);
  writeValueToJson("semantic_id", jsonObj, allocator);
  if (getUniformScale() != 1.0f) {
    writeValueToJson("uniform_scale", jsonObj, allocator);
  }
  if (getMassScale() != 1.0) {
    writeValueToJson("mass_scale", jsonObj, allocator);
  }
  writeValueToJson("base_type", jsonObj, allocator);
  writeValueToJson("inertia_source", jsonObj, allocator);
  writeValueToJson("link_order", jsonObj, allocator);
  writeValueToJson("render_mode", jsonObj, allocator);
  writeValueToJson("shader_type", jsonObj, allocator);
}  // ArticulatedObjectAttributes::writeValuesToJson

std::string ArticulatedObjectAttributes::getObjectInfoHeaderInternal() const {
  return "URDF Filepath,Render Asset,Semantic ID,Uniform Scale,Mass Scale,Base "
         "Type,Inertia Source,Link Order,Render Mode,Current Shader Type,";
}  // ArticulatedObjectAttributes::getObjectInfoHeaderInternal

std::string ArticulatedObjectAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getURDFPath(), getRenderAssetHandle(),
      getAsString("semantic_id"), getAsString("uniform_scale"),
      getAsString("mass_scale"), getAOBaseTypeName(getBaseType()),
      getAOInertiaSourceName(getInertiaSource()),
      getAOLinkOrderName(getLinkOrder()), getAORenderModeName(getRenderMode()),
      getShaderTypeName(getShaderType()));
}  // ArticulatedObjectAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
