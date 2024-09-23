// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_

#include "AbstractAttributes.h"
#include "MarkerSets.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief base attributes object holding attributes shared by all
 * @ref esp::metadata::attributes::ObjectAttributes and @ref
 * esp::metadata::attributes::StageAttributes objects; Should be treated as
 * abstract - should never be instanced directly
 */
class AbstractObjectAttributes : public AbstractAttributes {
 public:
  AbstractObjectAttributes(const std::string& classKey,
                           const std::string& handle);

  ~AbstractObjectAttributes() override = default;

  /**
   * @brief Scale of the ojbect
   */
  void setScale(const Magnum::Vector3& scale) { set("scale", scale); }
  Magnum::Vector3 getScale() const { return get<Magnum::Vector3>("scale"); }

  /**
   * @brief collision shape inflation margin
   */
  void setMargin(double margin) { set("margin", margin); }
  double getMargin() const { return get<double>("margin"); }

  // if object should be checked for collisions - if other objects can collide
  // with this object
  void setIsCollidable(bool isCollidable) {
    set("is_collidable", isCollidable);
  }
  bool getIsCollidable() const { return get<bool>("is_collidable"); }

  /**
   * @brief Set default up orientation for object/stage mesh
   */
  void setOrientUp(const Magnum::Vector3& orientUp) { set("up", orientUp); }
  /**
   * @brief get default up orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientUp() const { return get<Magnum::Vector3>("up"); }
  /**
   * @brief Set default forward orientation for object/stage mesh
   */
  void setOrientFront(const Magnum::Vector3& orientFront) {
    set("front", orientFront);
  }
  /**
   * @brief get default forward orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientFront() const {
    return get<Magnum::Vector3>("front");
  }

  /**
   * @brief Sets how many units map to a meter.
   */
  void setUnitsToMeters(double unitsToMeters) {
    set("units_to_meters", unitsToMeters);
  }
  /**
   * @brief Gets how many units map to a meter.
   */
  double getUnitsToMeters() const { return get<double>("units_to_meters"); }

  /**
   * @brief Set whether visible or not. If not visible can add dynamic
   * non-rendered object into a scene. If is not visible then should not add
   * object to drawables.
   */
  void setIsVisible(bool isVisible) { set("is_visible", isVisible); }
  /**
   * @brief Get whether visible or not. If not visible can add dynamic
   * non-rendered object into a scene. If is not visible then should not add
   * object to drawables.
   */
  bool getIsVisible() const { return get<bool>("is_visible"); }

  void setFrictionCoefficient(double frictionCoefficient) {
    set("friction_coefficient", frictionCoefficient);
  }
  double getFrictionCoefficient() const {
    return get<double>("friction_coefficient");
  }
  void setRollingFrictionCoefficient(double rollingFrictionCoefficient) {
    set("rolling_friction_coefficient", rollingFrictionCoefficient);
  }
  double getRollingFrictionCoefficient() const {
    return get<double>("rolling_friction_coefficient");
  }
  void setSpinningFrictionCoefficient(double spinningFrictionCoefficient) {
    set("spinning_friction_coefficient", spinningFrictionCoefficient);
  }
  double getSpinningFrictionCoefficient() const {
    return get<double>("spinning_friction_coefficient");
  }

  void setRestitutionCoefficient(double restitutionCoefficient) {
    set("restitution_coefficient", restitutionCoefficient);
  }
  double getRestitutionCoefficient() const {
    return get<double>("restitution_coefficient");
  }
  /**
   * @brief Sets the relative path/filename for the render asset to be used to
   * render the construct this attributes describes. This is relative to the
   * on-disk location of the file responsible for this configuration.
   */
  void setRenderAssetHandle(const std::string& renderAssetHandle) {
    set("render_asset", renderAssetHandle);
    setFilePathsAreDirty();
  }

  /**
   * @brief Gets the relative path/filename for the render asset to be used to
   * render the construct this attributes describes. This is relative to the
   * on-disk location of the file responsible for this configuration.
   */
  std::string getRenderAssetHandle() const {
    return get<std::string>("render_asset");
  }

  /**
   * @brief Sets the fully-qualified filepath for the render asset to be used to
   * render the construct this attributes describes. This is only used
   * internally and should not be saved to disk.
   */
  void setRenderAssetFullPath(const std::string& renderAssetHandle) {
    setHidden("__renderAssetFullPath", renderAssetHandle);
    setFilePathsAreDirty();
  }

  /**
   * @brief Gets the fully-qualified filepath for the render asset to be used to
   * render the construct this attributes describes. This is only used
   * internally and should not be saved to disk.
   */
  std::string getRenderAssetFullPath() const {
    return get<std::string>("__renderAssetFullPath");
  }

  /**
   * @brief Sets the render asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void setRenderAssetType(const std::string& renderAssetType) {
    const std::string rAssetTypeLC =
        Cr::Utility::String::lowercase(renderAssetType);

    auto mapIter = AssetTypeNamesMap.find(rAssetTypeLC);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal render_asset_type value"
                  << renderAssetType << ":" << rAssetTypeLC
                  << "attempted to be set in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("render_asset_type", renderAssetType);
  }

  /**
   * @brief Initialize the render asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void initRenderAssetType(const std::string& renderAssetType) {
    const std::string rAssetTypeLC =
        Cr::Utility::String::lowercase(renderAssetType);

    auto mapIter = AssetTypeNamesMap.find(rAssetTypeLC);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal render_asset_type value"
                  << renderAssetType << ":" << rAssetTypeLC
                  << "attempted to be initialized in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    initTranslated("render_asset_type", renderAssetType);
  }
  /**
   * @brief Sets the render asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void setRenderAssetTypeEnum(AssetType assetTypeEnum) {
    const std::string renderAssetType = getAssetTypeName(assetTypeEnum);

    auto mapIter = AssetTypeNamesMap.find(renderAssetType);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal render_asset_type enum value given"
                  << static_cast<int>(assetTypeEnum) << ":" << renderAssetType
                  << "attempted to be set in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("render_asset_type", renderAssetType);
  }

  /**
   * @brief Initialize the render asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void initRenderAssetTypeEnum(AssetType assetTypeEnum) {
    const std::string renderAssetType = getAssetTypeName(assetTypeEnum);

    auto mapIter = AssetTypeNamesMap.find(renderAssetType);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal render_asset_type enum value given"
                  << static_cast<int>(assetTypeEnum) << ":" << renderAssetType
                  << "attempted to be initialized in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    initTranslated("render_asset_type", renderAssetType);
  }

  /**
   * @brief Gets the render asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  AssetType getRenderAssetType() {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("render_asset_type"));
    auto mapIter = AssetTypeNamesMap.find(val);
    if (mapIter != AssetTypeNamesMap.end()) {
      return mapIter->second;
    }
    // Asset type is unknown or unspecified.
    return AssetType::Unknown;
  }

  /**
   * @brief Sets whether this object uses file-based mesh render object or
   * primitive render shapes
   * @param renderAssetIsPrimitive whether this object's render asset is a
   * primitive or not
   */
  void setRenderAssetIsPrimitive(bool renderAssetIsPrimitive) {
    setHidden("__renderAssetIsPrimitive", renderAssetIsPrimitive);
  }
  /**
   * @brief Get whether this object uses file-based mesh render object or
   * primitive render shapes
   * @return whether this object's render asset is a
   * primitive or not
   */
  bool getRenderAssetIsPrimitive() const {
    return get<bool>("__renderAssetIsPrimitive");
  }

  /**
   * @brief Sets the relative path/filename for the collision asset to be used
   * for mesh collision detection for the construct this attributes describes.
   * This is relative to the on-disk location of the file responsible for this
   * configuration.
   */
  void setCollisionAssetHandle(const std::string& collisionAssetHandle) {
    set("collision_asset", collisionAssetHandle);
    setFilePathsAreDirty();
  }

  /**
   * @brief Gets the relative path/filename for the collision asset to be used
   * for mesh collision detection for the construct this attributes describes.
   * This is relative to the on-disk location of the file responsible for this
   * configuration.
   */
  std::string getCollisionAssetHandle() const {
    return get<std::string>("collision_asset");
  }
  /**
   * @brief Sets the fully-qualified filepath for the collision asset to be used
   * for mesh collision detection for the construct this attributes describes.
   * This is only used internally and should not be saved to disk.
   */
  void setCollisionAssetFullPath(const std::string& collisionAssetHandle) {
    setHidden("__collisionAssetFullPath", collisionAssetHandle);
    setFilePathsAreDirty();
  }

  /**
   * @brief Gets the fully-qualified filepath for the collision asset to be used
   * for mesh collision detection for the construct this attributes describes.
   * This is only used internally and should not be saved to disk.
   */
  std::string getCollisionAssetFullPath() const {
    return get<std::string>("__collisionAssetFullPath");
  }
  /**
   * @brief Sets the collision asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void setCollisionAssetType(const std::string& collisionAssetType) {
    const std::string cAssetTypeLC =
        Cr::Utility::String::lowercase(collisionAssetType);

    auto mapIter = AssetTypeNamesMap.find(cAssetTypeLC);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal collision_asset_type value"
                  << collisionAssetType << ":" << cAssetTypeLC
                  << "attempted to be set in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("collision_asset_type", collisionAssetType);
  }

  /**
   * @brief Initialize the collision asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void initCollisionAssetType(const std::string& collisionAssetType) {
    const std::string cAssetTypeLC =
        Cr::Utility::String::lowercase(collisionAssetType);

    auto mapIter = AssetTypeNamesMap.find(cAssetTypeLC);
    ESP_CHECK(mapIter != AssetTypeNamesMap.end(),
              "Illegal collision_asset_type value"
                  << collisionAssetType << ":" << cAssetTypeLC
                  << "attempted to be set in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    initTranslated("collision_asset_type", collisionAssetType);
  }

  /**
   * @brief Sets the collision asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void setCollisionAssetTypeEnum(AssetType assetTypeEnum) {
    const std::string collisionAssetType = getAssetTypeName(assetTypeEnum);

    auto mapIter = AssetTypeNamesMap.find(collisionAssetType);
    ESP_CHECK(
        mapIter != AssetTypeNamesMap.end(),
        "Illegal collision_asset_type enum value given"
            << static_cast<int>(assetTypeEnum) << ":" << collisionAssetType
            << "attempted to be set in AbstractObjectAttributes:" << getHandle()
            << ". Aborting.");
    setTranslated("collision_asset_type", collisionAssetType);
  }

  /**
   * @brief Initialize the collision asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  void initCollisionAssetTypeEnum(AssetType assetTypeEnum) {
    const std::string collisionAssetType = getAssetTypeName(assetTypeEnum);

    auto mapIter = AssetTypeNamesMap.find(collisionAssetType);
    ESP_CHECK(
        mapIter != AssetTypeNamesMap.end(),
        "Illegal collision_asset_type enum value given"
            << static_cast<int>(assetTypeEnum) << ":" << collisionAssetType
            << "attempted to be set in AbstractObjectAttributes:" << getHandle()
            << ". Aborting.");
    initTranslated("collision_asset_type", collisionAssetType);
  }

  /**
   * @brief Gets the collision asset type, as specified by @ref AssetType.
   * This specification was generally intended for specifying certain criteria
   * such as orientation for the loaded asset based on file name, which was in
   * turn queried for type. This is an artifact of a very early version of
   * Habitat-Sim and effort should be spent to remove this entirely, as more
   * accurate and thorough mechanisms have been implemented in its place.
   */
  AssetType getCollisionAssetType() const {
    const std::string val = Cr::Utility::String::lowercase(
        get<std::string>("collision_asset_type"));
    auto mapIter = AssetTypeNamesMap.find(val);
    if (mapIter != AssetTypeNamesMap.end()) {
      return mapIter->second;
    }
    // Asset type is unknown or unspecified.
    return AssetType::Unknown;
  }

  /**
   * @brief Sets the size/scale of the collision asset compared to the render
   * asset so that the final constructs align.
   */
  void setCollisionAssetSize(const Magnum::Vector3& collisionAssetSize) {
    set("collision_asset_size", collisionAssetSize);
  }

  /**
   * @brief Gets the size/scale of the collision asset compared to the render
   * asset so that the final constructs align.
   */
  Magnum::Vector3 getCollisionAssetSize() const {
    return get<Magnum::Vector3>("collision_asset_size");
  }

  /**
   * @brief Sets whether this object uses file-based mesh collision object or
   * primitive(implicit) collision shapes
   * @param collisionAssetIsPrimitive whether this object's collision asset is a
   * primitive (implicitly calculated) or a mesh
   */
  void setCollisionAssetIsPrimitive(bool collisionAssetIsPrimitive) {
    setHidden("__collisionAssetIsPrimitive", collisionAssetIsPrimitive);
  }
  /**
   * @brief Gets whether this object uses file-based mesh collision object or
   * primitive(implicit) collision shapes
   * @return whether this object's collision asset is a
   * primitive (implicitly calculated) or a mesh
   */
  bool getCollisionAssetIsPrimitive() const {
    return get<bool>("__collisionAssetIsPrimitive");
  }

  /**
   * @brief Sets whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   */
  void setUseMeshCollision(bool useMeshCollision) {
    set("use_mesh_collision", useMeshCollision);
  }

  /**
   * @brief Gets whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   * @return Whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   */
  bool getUseMeshCollision() const { return get<bool>("use_mesh_collision"); }

  /**
   * @brief Set the default shader to use for an object or stage.  This may be
   * overridden by a scene instance specification.
   */
  void setShaderType(const std::string& shader_type) {
    // force to lowercase before setting
    const std::string shaderTypeLC =
        Cr::Utility::String::lowercase(shader_type);
    auto mapIter = ShaderTypeNamesMap.find(shaderTypeLC);
    ESP_CHECK(mapIter != ShaderTypeNamesMap.end(),
              "Illegal shader_type value"
                  << shader_type
                  << "attempted to be set in AbstractObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("shader_type", shader_type);
  }

  /**
   * @brief Get the default shader to use for an object or stage.  This may be
   * overridden by a scene instance specification.
   */
  ObjectInstanceShaderType getShaderType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("shader_type"));
    auto mapIter = ShaderTypeNamesMap.find(val);
    if (mapIter != ShaderTypeNamesMap.end()) {
      return mapIter->second;
    }
    // Unspecified is default value - should never be returned since setter
    // verifies value
    return ObjectInstanceShaderType::Unspecified;
  }

  /**
   * @brief If true then use flat shading regardless of what shader-type is
   * specified by materials or other configs.
   */
  void setForceFlatShading(bool force_flat_shading) {
    set("force_flat_shading", force_flat_shading);
  }
  /**
   * @brief if true use flat shading instead of phong or pbr shader
   */
  bool getForceFlatShading() const { return get<bool>("force_flat_shading"); }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * AbstractObjectAttributes and deriving (ObjectAttributes and
   * StageAttributes) classes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

  /**
   * @brief Whether to use the specified orientation frame for all orientation
   * tasks for this asset.  This will always be true, except if an overriding
   * semantic mesh-specific frame is specified for stages.
   */
  bool getUseFrameForAllOrientation() const {
    return get<bool>("__useFrameForAllOrientation");
  }

  /**
   * @brief Gets a smart pointer reference to a copy of the marker_sets
   * configuration data from config file.
   */
  std::shared_ptr<MarkerSets> getMarkerSetsConfiguration() const {
    return getSubconfigCopy<MarkerSets>("marker_sets");
  }

  /**
   * @brief Gets a smart pointer reference to the actual marker_sets
   * configuration data from config file. This method is for editing the
   * configuration.
   */
  std::shared_ptr<MarkerSets> editMarkerSetsConfiguration() {
    return editSubconfig<MarkerSets>("marker_sets");
  }

  /**
   * @brief Rekey all the markers in the marker_sets subconfiguration such that
   * each point is keyed by a sequential numeric string that preserves the
   * natural ordering of the key strings.
   */
  int rekeyAllMarkerSets() {
    return editSubconfig<MarkerSets>("marker_sets")->rekeyAllMarkers();
  }

 protected:
  /**
   * @brief Whether to use the specified orientation frame for all orientation
   * tasks for this asset.  This will always be true, except if an overriding
   * semantic mesh-specific frame is specified for stages.
   */
  void setUseFrameForAllOrientation(bool useFrameForAllOrientation) {
    setHidden("__useFrameForAllOrientation", useFrameForAllOrientation);
  }

  /**
   * @brief Write child-class-specific values to json object
   *
   */
  virtual void writeValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {}
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;
  /**
   * @brief get AbstractObject specific info header
   */
  virtual std::string getAbstractObjectInfoHeaderInternal() const {
    return "";
  };

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief get AbstractObject specific info for csv string
   */
  virtual std::string getAbstractObjectInfoInternal() const { return ""; };

 public:
  ESP_SMART_POINTERS(AbstractObjectAttributes)

};  // class AbstractObjectAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_
