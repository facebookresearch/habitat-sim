// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_

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

/**
 * @brief Specific Attributes instance describing a rigid object, constructed
 * with a default set of object-specific required attributes.
 */
class ObjectAttributes : public AbstractObjectAttributes {
 public:
  explicit ObjectAttributes(const std::string& handle = "");
  // center of mass (COM)
  void setCOM(const Magnum::Vector3& com) { set("COM", com); }
  Magnum::Vector3 getCOM() const { return get<Magnum::Vector3>("COM"); }

  // whether com is provided or not
  void setComputeCOMFromShape(bool computeCOMFromShape) {
    set("compute_COM_from_shape", computeCOMFromShape);
  }
  bool getComputeCOMFromShape() const {
    return get<bool>("compute_COM_from_shape");
  }

  void setMass(double mass) { set("mass", mass); }
  double getMass() const { return get<double>("mass"); }

  // inertia diagonal
  void setInertia(const Magnum::Vector3& inertia) { set("inertia", inertia); }
  Magnum::Vector3 getInertia() const { return get<Magnum::Vector3>("inertia"); }

  void setLinearDamping(double linearDamping) {
    set("linear_damping", linearDamping);
  }
  double getLinearDamping() const { return get<double>("linear_damping"); }

  void setAngularDamping(double angularDamping) {
    set("angular_damping", angularDamping);
  }
  double getAngularDamping() const { return get<double>("angular_damping"); }

  // if true override other settings and use render mesh bounding box as
  // collision object
  void setBoundingBoxCollisions(bool useBoundingBoxForCollision) {
    set("use_bounding_box_for_collision", useBoundingBoxForCollision);
  }
  bool getBoundingBoxCollisions() const {
    return get<bool>("use_bounding_box_for_collision");
  }

  // if true join all mesh components of an asset into a unified collision
  // object
  void setJoinCollisionMeshes(bool joinCollisionMeshes) {
    set("join_collision_meshes", joinCollisionMeshes);
  }
  bool getJoinCollisionMeshes() const {
    return get<bool>("join_collision_meshes");
  }

  void setSemanticId(int semanticId) { set("semantic_id", semanticId); }

  uint32_t getSemanticId() const { return get<int>("semantic_id"); }

 protected:
  /**
   * @brief Write object-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  /**
   * @brief get AbstractObject specific info header
   */
  std::string getAbstractObjectInfoHeaderInternal() const override {
    return "Mass,COM XYZ,I XX YY ZZ,Angular Damping,"
           "Linear Damping,Semantic ID";
  }
  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(ObjectAttributes)

};  // class ObjectAttributes

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

#endif  // ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
