// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_

#include "AttributesBase.h"

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
  void setOrientUp(const Magnum::Vector3& orientUp) {
    set("orient_up", orientUp);
  }
  /**
   * @brief get default up orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientUp() const {
    return get<Magnum::Vector3>("orient_up");
  }
  /**
   * @brief Set default forward orientation for object/stage mesh
   */
  void setOrientFront(const Magnum::Vector3& orientFront) {
    set("orient_front", orientFront);
  }
  /**
   * @brief get default forward orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientFront() const {
    return get<Magnum::Vector3>("orient_front");
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
   * @brief If not visible can add dynamic non-rendered object into a scene
   * object.  If is not visible then should not add object to drawables.
   */
  void setIsVisible(bool isVisible) { set("is_visible", isVisible); }
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
  void setRenderAssetType(int renderAssetType) {
    set("render_asset_type", renderAssetType);
  }
  int getRenderAssetType() { return get<int>("render_asset_type"); }

  void setRenderAssetHandle(const std::string& renderAssetHandle) {
    set("render_asset", renderAssetHandle);
    setIsDirty();
  }
  std::string getRenderAssetHandle() const {
    return get<std::string>("render_asset");
  }

  /**
   * @brief Sets whether this object uses file-based mesh render object or
   * primitive render shapes
   * @param renderAssetIsPrimitive whether this object's render asset is a
   * primitive or not
   */
  void setRenderAssetIsPrimitive(bool renderAssetIsPrimitive) {
    set("renderAssetIsPrimitive", renderAssetIsPrimitive);
  }
  /**
   * @brief Get whether this object uses file-based mesh render object or
   * primitive render shapes
   * @return whether this object's render asset is a
   * primitive or not
   */
  bool getRenderAssetIsPrimitive() const {
    return get<bool>("renderAssetIsPrimitive");
  }

  void setCollisionAssetHandle(const std::string& collisionAssetHandle) {
    set("collision_asset", collisionAssetHandle);
    setIsDirty();
  }
  std::string getCollisionAssetHandle() const {
    return get<std::string>("collision_asset");
  }

  void setCollisionAssetType(int collisionAssetType) {
    set("collision_asset_type", collisionAssetType);
  }
  int getCollisionAssetType() { return get<int>("collision_asset_type"); }

  void setCollisionAssetSize(const Magnum::Vector3& collisionAssetSize) {
    set("collision_asset_size", collisionAssetSize);
  }
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
    set("collisionAssetIsPrimitive", collisionAssetIsPrimitive);
  }
  /**
   * @brief Gets whether this object uses file-based mesh collision object or
   * primitive(implicit) collision shapes
   * @return whether this object's collision asset is a
   * primitive (implicitly calculated) or a mesh
   */
  bool getCollisionAssetIsPrimitive() const {
    return get<bool>("collisionAssetIsPrimitive");
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
    set("shader_type", shader_type);
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

  bool getIsDirty() const { return get<bool>("__isDirty"); }
  void setIsClean() { set("__isDirty", false); }

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
    return get<bool>("use_frame_for_all_orientation");
  }

 protected:
  /**
   * @brief Whether to use the specified orientation frame for all orientation
   * tasks for this asset.  This will always be true, except if an overriding
   * semantic mesh-specific frame is specified for stages.
   */
  void setUseFrameForAllOrientation(bool useFrameForAllOrientation) {
    set("use_frame_for_all_orientation", useFrameForAllOrientation);
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
  void setIsDirty() { set("__isDirty", true); }

 public:
  ESP_SMART_POINTERS(AbstractObjectAttributes)

};  // class AbstractObjectAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTOBJECTATTRIBUTES_H_
