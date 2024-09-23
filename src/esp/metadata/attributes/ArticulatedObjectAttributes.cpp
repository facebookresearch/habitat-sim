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
  // We want to set this as a non-default value
  setURDFPath(handle);

  init("render_asset", "");
  init("semantic_id", 0);
  init("is_visible", true);
  // Initialize the default base type to be free joint
  initTranslated("base_type",
                 getAOBaseTypeName(ArticulatedObjectBaseType::Free));
  // Initialize the default source for the inertia calculation to be computed
  initTranslated(
      "inertia_source",
      getAOInertiaSourceName(ArticulatedObjectInertiaSource::Computed));
  // Initialize the default link order to use as the tree traversal order
  initTranslated("link_order",
                 getAOLinkOrderName(ArticulatedObjectLinkOrder::TreeTraversal));

  // Initialize render mode to be default - skin if present, otherwise link
  // meshes/primitives
  initTranslated("render_mode",
                 getAORenderModeName(ArticulatedObjectRenderMode::Default));
  // Initialize the default behavior to "use the best shader for the material"
  initTranslated("shader_type",
                 getShaderTypeName(ObjectInstanceShaderType::Material));

  init("uniform_scale", 1.0f);
  init("mass_scale", 1.0);

  // Initialize these so they exist in the configuration
  setHidden("__urdfFullPath", "");
  setHidden("__renderAssetFullPath", "");
  // This specifies that we want to investigate the state of the urdf and skin
  // render asset handles before we allow this attributes to be registered.
  // Hidden field
  setFilePathsAreDirty();
  // set up an existing subgroup for marker_sets attributes
  addOrEditSubgroup<MarkerSets>("marker_sets");
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
      "{},{},{},{},{},{},{},{},{},{}", getURDFPath(), getRenderAssetHandle(),
      getAsString("semantic_id"), getAsString("uniform_scale"),
      getAsString("mass_scale"), getAOBaseTypeName(getBaseType()),
      getAOInertiaSourceName(getInertiaSource()),
      getAOLinkOrderName(getLinkOrder()), getAORenderModeName(getRenderMode()),
      getShaderTypeName(getShaderType()));
}  // ArticulatedObjectAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
