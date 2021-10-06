// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_

#include "AttributesBase.h"

#include "esp/assets/Asset.h"

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
  /**
   * @brief Constant static map to provide mappings from string tags to
   * @ref esp::assets::AssetType values.  This will be used to map values
   * set in json for mesh type to @ref esp::assets::AssetType.  Keys must
   * be lowercase.
   */
  static const std::map<std::string, esp::assets::AssetType> AssetTypeNamesMap;

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
   * @brief set default up orientation for object/stage mesh
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
   * @brief set default forward orientation for object/stage mesh
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
    // Unknown is default value - should never be returned since setter verifies
    // value
    return ObjectInstanceShaderType::Unknown;
  }

  /**
   * @brief If true then use flat shading regardless of what shader-type is
   * specified by materials or other configs.
   */
  void setForceFlatShading(bool force_flat_shading) {
    set("force_flat_shading", force_flat_shading);
  }
  bool getForceFlatShading() const { return get<bool>("force_flat_shading"); }

  bool getIsDirty() const { return get<bool>("__isDirty"); }
  void setIsClean() { set("__isDirty", false); }

 protected:
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
  void setHouseFilename(const std::string& houseFilename) {
    set("houseFilename", houseFilename);
    setIsDirty();
  }
  std::string getHouseFilename() const {
    return get<std::string>("houseFilename");
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
  int getSemanticAssetType() { return get<int>("semantic_asset_type"); }

  void setLoadSemanticMesh(bool loadSemanticMesh) {
    set("loadSemanticMesh", loadSemanticMesh);
  }
  bool getLoadSemanticMesh() { return get<bool>("loadSemanticMesh"); }

  void setNavmeshAssetHandle(const std::string& navmeshAssetHandle) {
    set("navmeshAssetHandle", navmeshAssetHandle);
    setIsDirty();
  }
  std::string getNavmeshAssetHandle() const {
    return get<std::string>("navmeshAssetHandle");
  }

  /**
   * @brief set lighting setup for stage.  Default value comes from
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
   * @brief set frustum culling for stage.  Default value comes from
   * @ref esp::sim::SimulatorConfiguration, is overridden by any value set in
   * json, if exists.
   */
  void setFrustumCulling(bool frustumCulling) {
    set("frustum_culling", frustumCulling);
  }
  bool getFrustumCulling() const { return get<bool>("frustum_culling"); }

 protected:
  /**
   * @brief get AbstractObject specific info header
   */
  std::string getAbstractObjectInfoHeaderInternal() const override {
    return "Navmesh Handle,Gravity XYZ,Origin XYZ,Light Setup,";
  }

  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractObjectInfoInternal() const override {
    return Cr::Utility::formatString("{},{},{},{}", getNavmeshAssetHandle(),
                                     getAsString("gravity"),
                                     getAsString("origin"), getLightSetupKey());
  }

 public:
  ESP_SMART_POINTERS(StageAttributes)

};  // class StageAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
