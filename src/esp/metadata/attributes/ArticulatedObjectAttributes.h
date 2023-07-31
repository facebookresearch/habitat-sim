// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief attributes class describing essential and default quantities used to
 * instantiate an Articulated object
 */
class ArticulatedObjectAttributes : public AbstractAttributes {
 public:
  explicit ArticulatedObjectAttributes(const std::string& handle = "");
  /**
   * @brief Sets the string name for the Articulated Object URDF relative path
   */
  void setURDFPath(const std::string& urdfFilepath) {
    set("urdf_filepath", urdfFilepath);
  }
  /**
   * @brief Gets the string name for the Articulated Object URDF relative path
   */
  std::string getURDFPath() const { return get<std::string>("urdf_filepath"); }
  /**
   * @brief Sets the string name for the render asset relative path
   */
  void setRenderAssetHandle(const std::string& renderAsset) {
    set("render_asset", renderAsset);
  }
  /**
   * @brief Gets the string name for the render asset relative path
   */
  std::string getRenderAssetHandle() const {
    return get<std::string>("render_asset");
  }

  /**
   * @brief Sets the semantic ID for instanced Articulated Objects.
   */
  void setSemanticId(int semanticId) { set("semantic_id", semanticId); }

  /**
   * @brief Gets the semantic ID for instanced Articulated Objects.
   */
  uint32_t getSemanticId() const { return get<int>("semantic_id"); }

  /**
   * @brief Sets whether we should render using the articulated object
   * primitives, even if a render asset is present.
   */
  void setDebugRenderPrimitives(bool dbgRenderPrims) {
    set("debug_render_primitives", dbgRenderPrims);
  }
  /**
   * @brief Gets whether we should render using the articulated object
   * primitives, even if a render asset is present.
   */
  bool getDebugRenderPrimitives() const {
    return get<bool>("debug_render_primitives");
  }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * ArticulatedObjectAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(ArticulatedObjectAttributes)
};  // class ArticulatedObjectAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_
