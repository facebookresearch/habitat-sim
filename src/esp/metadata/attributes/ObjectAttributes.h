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
 * @brief This enum class defines the possible shader options for rendering
 * instances of objects or stages in Habitat-sim.
 */
enum class ObjectInstanceShaderType {
  /**
   * Represents an unknown/unspecified value for the shader type to use. Resort
   * to defaults for object type.
   */
  Unknown = ID_UNDEFINED,
  /**
   * Override any config-specified or default shader-type values to use the
   * material-specified shader.
   */
  Material,
  /**
   * Refers to flat shading, pure color and no lighting.  This is often used for
   * textured objects
   */
  Flat,
  /**
   * Refers to phong shading with pure diffuse color.
   */
  Phong,
  /**
   * Refers to using a shader built with physically-based rendering models.
   */
  PBR,
};

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

  static const std::map<std::string, ObjectInstanceShaderType>
      ShaderTypeNamesMap;

  AbstractObjectAttributes(const std::string& classKey,
                           const std::string& handle);

  ~AbstractObjectAttributes() override = default;

  /**
   * @brief Scale of the ojbect
   */
  void setScale(const Magnum::Vector3& scale) { setVec3("scale", scale); }
  Magnum::Vector3 getScale() const { return getVec3("scale"); }

  /**
   * @brief collision shape inflation margin
   */
  void setMargin(double margin) { setDouble("margin", margin); }
  double getMargin() const { return getDouble("margin"); }

  // if object should be checked for collisions - if other objects can collide
  // with this object
  void setIsCollidable(bool isCollidable) {
    setBool("is_collidable", isCollidable);
  }
  bool getIsCollidable() const { return getBool("is_collidable"); }

  /**
   * @brief set default up orientation for object/stage mesh
   */
  void setOrientUp(const Magnum::Vector3& orientUp) {
    setVec3("orient_up", orientUp);
  }
  /**
   * @brief get default up orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientUp() const { return getVec3("orient_up"); }
  /**
   * @brief set default forward orientation for object/stage mesh
   */
  void setOrientFront(const Magnum::Vector3& orientFront) {
    setVec3("orient_front", orientFront);
  }
  /**
   * @brief get default forward orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientFront() const { return getVec3("orient_front"); }

  /**
   * @brief Sets how many units map to a meter.
   */
  void setUnitsToMeters(double unitsToMeters) {
    setDouble("units_to_meters", unitsToMeters);
  }
  /**
   * @brief Gets how many units map to a meter.
   */
  double getUnitsToMeters() const { return getDouble("units_to_meters"); }

  void setFrictionCoefficient(double frictionCoefficient) {
    setDouble("friction_coefficient", frictionCoefficient);
  }
  double getFrictionCoefficient() const {
    return getDouble("friction_coefficient");
  }

  void setRestitutionCoefficient(double restitutionCoefficient) {
    setDouble("restitution_coefficient", restitutionCoefficient);
  }
  double getRestitutionCoefficient() const {
    return getDouble("restitution_coefficient");
  }
  void setRenderAssetType(int renderAssetType) {
    setInt("render_asset_type", renderAssetType);
  }
  int getRenderAssetType() { return getInt("render_asset_type"); }

  void setRenderAssetHandle(const std::string& renderAssetHandle) {
    setString("render_asset", renderAssetHandle);
    setIsDirty();
  }
  std::string getRenderAssetHandle() const { return getString("render_asset"); }

  /**
   * @brief Sets whether this object uses file-based mesh render object or
   * primitive render shapes
   * @param renderAssetIsPrimitive whether this object's render asset is a
   * primitive or not
   */
  void setRenderAssetIsPrimitive(bool renderAssetIsPrimitive) {
    setBool("renderAssetIsPrimitive", renderAssetIsPrimitive);
  }

  bool getRenderAssetIsPrimitive() const {
    return getBool("renderAssetIsPrimitive");
  }

  void setCollisionAssetHandle(const std::string& collisionAssetHandle) {
    setString("collision_asset", collisionAssetHandle);
    setIsDirty();
  }
  std::string getCollisionAssetHandle() const {
    return getString("collision_asset");
  }

  void setCollisionAssetType(int collisionAssetType) {
    setInt("collision_asset_type", collisionAssetType);
  }
  int getCollisionAssetType() { return getInt("collision_asset_type"); }

  void setCollisionAssetSize(const Magnum::Vector3& collisionAssetSize) {
    setVec3("collision_asset_size", collisionAssetSize);
  }
  Magnum::Vector3 getCollisionAssetSize() const {
    return getVec3("collision_asset_size");
  }

  /**
   * @brief Sets whether this object uses file-based mesh collision object or
   * primitive(implicit) collision shapes
   * @param collisionAssetIsPrimitive whether this object's collision asset is a
   * primitive (implicitly calculated) or a mesh
   */
  void setCollisionAssetIsPrimitive(bool collisionAssetIsPrimitive) {
    setBool("collisionAssetIsPrimitive", collisionAssetIsPrimitive);
  }
  /**
   * @brief Gets whether this object uses file-based mesh collision object or
   * primitive(implicit) collision shapes
   * @return whether this object's collision asset is a
   * primitive (implicitly calculated) or a mesh
   */
  bool getCollisionAssetIsPrimitive() const {
    return getBool("collisionAssetIsPrimitive");
  }

  /**
   * @brief Sets whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   */
  void setUseMeshCollision(bool useMeshCollision) {
    setBool("use_mesh_collision", useMeshCollision);
  }

  /**
   * @brief Gets whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   * @return Whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   */
  bool getUseMeshCollision() const { return getBool("use_mesh_collision"); }

  /**
   * @brief Set the default shader to use for an object or stage.  This may be
   * overridden by a scene instance specification.
   */
  void setShaderType(int shader_type) { setInt("shader_type", shader_type); }

  /**
   * @brief Get the default shader to use for an object or stage.  This may be
   * overridden by a scene instance specification.
   */
  int getShaderType() const { return getInt("shader_type"); }

  // if true use phong illumination model instead of flat shading
  void setRequiresLighting(bool requiresLighting) {
    setBool("requires_lighting", requiresLighting);
  }
  bool getRequiresLighting() const { return getBool("requires_lighting"); }

  bool getIsDirty() const { return getBool("__isDirty"); }
  void setIsClean() { setBool("__isDirty", false); }

 protected:
  void setIsDirty() { setBool("__isDirty", true); }

 public:
  ESP_SMART_POINTERS(AbstractObjectAttributes)

};  // class AbstractObjectAttributes

/**
 * @brief Specific Attributes instance describing an object, constructed with a
 * default set of object-specific required attributes
 */
class ObjectAttributes : public AbstractObjectAttributes {
 public:
  explicit ObjectAttributes(const std::string& handle = "");
  // center of mass (COM)
  void setCOM(const Magnum::Vector3& com) { setVec3("COM", com); }
  Magnum::Vector3 getCOM() const { return getVec3("COM"); }

  // whether com is provided or not
  void setComputeCOMFromShape(bool computeCOMFromShape) {
    setBool("compute_COM_from_shape", computeCOMFromShape);
  }
  bool getComputeCOMFromShape() const {
    return getBool("compute_COM_from_shape");
  }

  void setMass(double mass) { setDouble("mass", mass); }
  double getMass() const { return getDouble("mass"); }

  // inertia diagonal
  void setInertia(const Magnum::Vector3& inertia) {
    setVec3("inertia", inertia);
  }
  Magnum::Vector3 getInertia() const { return getVec3("inertia"); }

  void setLinearDamping(double linearDamping) {
    setDouble("linear_damping", linearDamping);
  }
  double getLinearDamping() const { return getDouble("linear_damping"); }

  void setAngularDamping(double angularDamping) {
    setDouble("angular_damping", angularDamping);
  }
  double getAngularDamping() const { return getDouble("angular_damping"); }

  // if true override other settings and use render mesh bounding box as
  // collision object
  void setBoundingBoxCollisions(bool useBoundingBoxForCollision) {
    setBool("use_bounding_box_for_collision", useBoundingBoxForCollision);
  }
  bool getBoundingBoxCollisions() const {
    return getBool("use_bounding_box_for_collision");
  }

  // if true join all mesh components of an asset into a unified collision
  // object
  void setJoinCollisionMeshes(bool joinCollisionMeshes) {
    setBool("join_collision_meshes", joinCollisionMeshes);
  }
  bool getJoinCollisionMeshes() const {
    return getBool("join_collision_meshes");
  }

  /**
   * @brief If not visible can add dynamic non-rendered object into a scene
   * object.  If is not visible then should not add object to drawables.
   */
  void setIsVisible(bool isVisible) { setBool("is_visible", isVisible); }
  bool getIsVisible() const { return getBool("is_visible"); }

  void setSemanticId(uint32_t semanticId) { setInt("semantic_id", semanticId); }

  uint32_t getSemanticId() const { return getInt("semantic_id"); }

 public:
  ESP_SMART_POINTERS(ObjectAttributes)

};  // class ObjectAttributes

///////////////////////////////////////
// stage attributes

/**
 * @brief Specific Attributes instance describing a stage, constructed with a
 * default set of stage-specific required attributes
 */
class StageAttributes : public AbstractObjectAttributes {
 public:
  explicit StageAttributes(const std::string& handle = "");

  void setOrigin(const Magnum::Vector3& origin) { setVec3("origin", origin); }
  Magnum::Vector3 getOrigin() const { return getVec3("origin"); }

  void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }
  void setHouseFilename(const std::string& houseFilename) {
    setString("houseFilename", houseFilename);
    setIsDirty();
  }
  std::string getHouseFilename() const { return getString("houseFilename"); }
  void setSemanticAssetHandle(const std::string& semanticAssetHandle) {
    setString("semantic_asset", semanticAssetHandle);
    setIsDirty();
  }
  std::string getSemanticAssetHandle() const {
    return getString("semantic_asset");
  }
  void setSemanticAssetType(int semanticAssetType) {
    setInt("semantic_asset_type", semanticAssetType);
  }
  int getSemanticAssetType() { return getInt("semantic_asset_type"); }

  void setLoadSemanticMesh(bool loadSemanticMesh) {
    setBool("loadSemanticMesh", loadSemanticMesh);
  }
  bool getLoadSemanticMesh() { return getBool("loadSemanticMesh"); }

  void setNavmeshAssetHandle(const std::string& navmeshAssetHandle) {
    setString("navmeshAssetHandle", navmeshAssetHandle);
    setIsDirty();
  }
  std::string getNavmeshAssetHandle() const {
    return getString("navmeshAssetHandle");
  }

  /**
   * @brief set lighting setup for stage.  Default value comes from
   * @ref esp::sim::SimulatorConfiguration, is overridden by any value set in
   * json, if exists.
   */
  void setLightSetup(const std::string& lightSetup) {
    setString("light_setup", lightSetup);
    setRequiresLighting(lightSetup != NO_LIGHT_KEY);
  }
  std::string getLightSetup() { return getString("light_setup"); }

  /**
   * @brief set frustum culling for stage.  Default value comes from
   * @ref esp::sim::SimulatorConfiguration, is overridden by any value set in
   * json, if exists.
   */
  void setFrustumCulling(bool frustumCulling) {
    setBool("frustum_culling", frustumCulling);
  }
  bool getFrustumCulling() const { return getBool("frustum_culling"); }

 public:
  ESP_SMART_POINTERS(StageAttributes)

};  // class StageAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_OBJECTATTRIBUTES_H_
