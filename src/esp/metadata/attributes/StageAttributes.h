// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_STAGEATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_STAGEATTRIBUTES_H_

#include "AbstractObjectAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

///////////////////////////////////////
// stage attributes

/**
 * @brief Specific Attributes instance describing a rigid stage, constructed
 * with a default set of stage-specific required attributes
 */
class StageAttributes : public AbstractObjectAttributes {
 public:
  explicit StageAttributes(const std::string& handle = "");

  void setOrigin(const Magnum::Vector3& origin) { set("origin", origin); }
  Magnum::Vector3 getOrigin() const { return get<Magnum::Vector3>("origin"); }

  void setGravity(const Magnum::Vector3& gravity) { set("gravity", gravity); }
  Magnum::Vector3 getGravity() const { return get<Magnum::Vector3>("gravity"); }

  /**
   * @brief Set default up orientation for semantic mesh. This is to support
   * stage aligning semantic meshes that have different orientations than the
   * stage render mesh.
   */
  void setSemanticOrientUp(const Magnum::Vector3& semanticOrientUp) {
    set("semantic_orient_up", semanticOrientUp);
    // specify that semantic meshes should not use base class render asset
    // orientation
    setUseFrameForAllOrientation(false);
  }
  /**
   * @brief get default up orientation for semantic mesh. This is to support
   * stage aligning semantic meshes that have different orientations than the
   * stage render mesh. Returns render asset up if no value for semantic asset
   * was specifically set.
   */
  Magnum::Vector3 getSemanticOrientUp() const {
    return (getUseFrameForAllOrientation()
                ? getOrientUp()
                : get<Magnum::Vector3>("semantic_orient_up"));
  }

  /**
   * @brief Set default forward orientation for semantic mesh. This is to
   * support stage aligning semantic meshes that have different orientations
   * than the stage render mesh.
   */
  void setSemanticOrientFront(const Magnum::Vector3& semanticOrientFront) {
    set("semantic_orient_front", semanticOrientFront);
    // specify that semantic meshes should not use base class render asset
    // orientation
    setUseFrameForAllOrientation(false);
  }
  /**
   * @brief get default forward orientation for semantic mesh. This is to
   * support stage aligning semantic meshes that have different orientations
   * than the stage render mesh. Returns render asset front if no value for
   * semantic asset was specifically set.
   */
  Magnum::Vector3 getSemanticOrientFront() const {
    return (getUseFrameForAllOrientation()
                ? getOrientFront()
                : get<Magnum::Vector3>("semantic_orient_front"));
  }

  /**
   * @brief Text file that describes the hierharchy of semantic information
   * embedded in the Semantic Asset mesh.  May be overridden by value
   * specified in Scene Instance Attributes.
   */
  void setSemanticDescriptorFilename(
      const std::string& semantic_descriptor_filename) {
    set("semantic_descriptor_filename", semantic_descriptor_filename);
    setIsDirty();
  }
  /**
   * @brief Text file that describes the hierharchy of semantic information
   * embedded in the Semantic Asset mesh.  May be overridden by value
   * specified in Scene Instance Attributes.
   */
  std::string getSemanticDescriptorFilename() const {
    return get<std::string>("semantic_descriptor_filename");
  }
  void setSemanticAssetHandle(const std::string& semanticAssetHandle) {
    set("semantic_asset", semanticAssetHandle);
    setIsDirty();
  }
  std::string getSemanticAssetHandle() const {
    return get<std::string>("semantic_asset");
  }
  void setSemanticAssetType(int semanticAssetType) {
    set("semantic_asset_type", semanticAssetType);
  }
  int getSemanticAssetType() const { return get<int>("semantic_asset_type"); }
  ESP_METADATA_ATTRIBUTES_STAGEATTRIBUTES_H_

  /**
   * @brief Set whether or not the semantic asset for this stage supports
   * texture semantics.
   */
  void setHasSemanticTextures(bool hasSemanticTextures) {
    set("has_semantic_textures", hasSemanticTextures);
  }

  bool getHasSemanticTextures() const {
    return get<bool>("has_semantic_textures");
  }

  /**
   * @brief Only set internally if we should use semantic textures for semantic
   * asset loading/rendering. Should only be true if "has_semantic_textures" is
   * true and user has specified to use semantic textures via
   * SimulatorConfiguration.
   */
  bool useSemanticTextures() const {
    return get<bool>("use_textures_for_semantic_rendering");
  }

  /**
   * @brief Only should be called when simulator::reconfigure is called, based
   * on setting
   */
  void setUseSemanticTextures(bool useSemanticTextures) {
    set("use_textures_for_semantic_rendering",
        (useSemanticTextures && getHasSemanticTextures()));
  }

  // Currently not supported
  void setLoadSemanticMesh(bool loadSemanticMesh) {
    set("loadSemanticMesh", loadSemanticMesh);
  }

  // Currently not supported
  bool getLoadSemanticMesh() { return get<bool>("loadSemanticMesh"); }

  void setNavmeshAssetHandle(const std::string& nav_asset) {
    set("nav_asset", nav_asset);
    setIsDirty();
  }
  std::string getNavmeshAssetHandle() const {
    return get<std::string>("nav_asset");
  }

  /**
   * @brief Set lighting setup for stage.  Default value comes from
   * @ref esp::sim::SimulatorConfiguration, is overridden by any value set in
   * json, if exists.
   */
  void setLightSetupKey(const std::string& light_setup_key) {
    set("light_setup_key", light_setup_key);
    setForceFlatShading(light_setup_key == NO_LIGHT_KEY);
  }
  std::string getLightSetupKey() const {
    return get<std::string>("light_setup_key");
  }

  /**
   * @brief Set frustum culling for stage.  Default value comes from
   * @ref esp::sim::SimulatorConfiguration, is overridden by any value set in
   * json, if exists.
   * Currently only set from SimulatorConfiguration
   */
  void setFrustumCulling(bool frustumCulling) {
    set("frustum_culling", frustumCulling);
  }
  bool getFrustumCulling() const { return get<bool>("frustum_culling"); }

 protected:
  /**
   * @brief Write stage-specific values to json object
   *
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  /**
   * @brief get AbstractObject specific info header
   */
  std::string getAbstractObjectInfoHeaderInternal() const override;

  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(StageAttributes)

};  // class StageAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_STAGEATTRIBUTES_H_
