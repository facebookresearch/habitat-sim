// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ATTRIBUTES_OBJECTATTRIBUTES_H_
#define ESP_ASSETS_ATTRIBUTES_OBJECTATTRIBUTES_H_

#include "AttributesBase.h"

#include "esp/assets/Asset.h"

namespace esp {
namespace assets {
namespace attributes {

/**
 * @brief base attributes object holding attributes shared by all
 * ObjectAttributes and StageAttributes objects; Should be
 * treated as if is abstract - should never be instanced directly
 */
class AbstractObjectAttributes : public AbstractAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref
   * AbstractObjectAttributes Has values that are different than defaults so
   * this can be used to test json loading. These values are set to be
   * purposefully invalid, for testing purposes, and so should not be used.
   */
  static const std::string JSONConfigTestString;

  /**
   * @brief Constant static map to provide mappings from string tags to @ref
   * AssetType values.  This will be used to map values set in json for mesh
   * type to @ref AssetTypes.  Keys must be lowercase.
   */
  static const std::map<std::string, esp::assets::AssetType> AssetTypeNamesMap;
  AbstractObjectAttributes(const std::string& classKey,
                           const std::string& handle);

  virtual ~AbstractObjectAttributes() = default;
  void setScale(const Magnum::Vector3& scale) { setVec3("scale", scale); }
  Magnum::Vector3 getScale() const { return getVec3("scale"); }

  /**
   * @brief collision shape inflation margin
   */
  void setMargin(double margin) { setDouble("margin", margin); }
  double getMargin() const { return getDouble("margin"); }

  /**
   * @brief set default up orientation for object/stage mesh
   */
  void setOrientUp(const Magnum::Vector3& orientUp) {
    setVec3("orientUp", orientUp);
  }
  /**
   * @brief get default up orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientUp() const { return getVec3("orientUp"); }
  /**
   * @brief set default forwardd orientation for object/stage mesh
   */
  void setOrientFront(const Magnum::Vector3& orientFront) {
    setVec3("orientFront", orientFront);
  }
  /**
   * @brief get default forwardd orientation for object/stage mesh
   */
  Magnum::Vector3 getOrientFront() const { return getVec3("orientFront"); }

  // units to meters mapping
  void setUnitsToMeters(double unitsToMeters) {
    setDouble("unitsToMeters", unitsToMeters);
  }
  double getUnitsToMeters() const { return getDouble("unitsToMeters"); }

  void setFrictionCoefficient(double frictionCoefficient) {
    setDouble("frictionCoefficient", frictionCoefficient);
  }
  double getFrictionCoefficient() const {
    return getDouble("frictionCoefficient");
  }

  void setRestitutionCoefficient(double restitutionCoefficient) {
    setDouble("restitutionCoefficient", restitutionCoefficient);
  }
  double getRestitutionCoefficient() const {
    return getDouble("restitutionCoefficient");
  }
  void setRenderAssetType(int renderAssetType) {
    setInt("renderAssetType", renderAssetType);
  }
  int getRenderAssetType() { return getInt("renderAssetType"); }

  void setRenderAssetHandle(const std::string& renderAssetHandle) {
    setString("renderAssetHandle", renderAssetHandle);
    setIsDirty();
  }
  std::string getRenderAssetHandle() const {
    return getString("renderAssetHandle");
  }

  /**
   * @brief Sets whether this object uses file-based mesh render object or
   * primitive render shapes
   * @param renderAssetIsPrimitive whether this object's render asset is a
   * primitive or not
   */
  void setRenderAssetIsPrimitive(bool renderAssetIsPrimitive) {
    setBool("renderAssetIsPrimitive", renderAssetIsPrimitive);
  }

  void setCollisionAssetType(int collisionAssetType) {
    setInt("collisionAssetType", collisionAssetType);
  }
  int getCollisionAssetType() { return getInt("collisionAssetType"); }

  bool getRenderAssetIsPrimitive() const {
    return getBool("renderAssetIsPrimitive");
  }

  void setCollisionAssetHandle(const std::string& collisionAssetHandle) {
    setString("collisionAssetHandle", collisionAssetHandle);
    setIsDirty();
  }
  std::string getCollisionAssetHandle() const {
    return getString("collisionAssetHandle");
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

  bool getCollisionAssetIsPrimitive() const {
    return getBool("collisionAssetIsPrimitive");
  }

  /**
   * @brief whether this object uses mesh collision or primitive(implicit)
   * collision calculation.
   */
  void setUseMeshCollision(bool useMeshCollision) {
    setBool("useMeshCollision", useMeshCollision);
  }

  bool getUseMeshCollision() const { return getBool("useMeshCollision"); }

  // if true use phong illumination model instead of flat shading
  void setRequiresLighting(bool requiresLighting) {
    setBool("requiresLighting", requiresLighting);
  }
  bool getRequiresLighting() const { return getBool("requiresLighting"); }

  bool getIsDirty() const { return getBool("__isDirty"); }
  void setIsClean() { setBool("__isDirty", false); }

 protected:
  void setIsDirty() { setBool("__isDirty", true); }

 public:
  ESP_SMART_POINTERS(AbstractObjectAttributes)

};  // class AbstractObjectAttributes

/**
 * @brief Specific Attributes instance which is constructed with a base set of
 * object required attributes
 */
class ObjectAttributes : public AbstractObjectAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref ObjectAttributes.
   * Has values that are different than defaults so this can be used to test
   * json loading. These values are set to be purposefully invalid, for testing
   * purposes, and so should not be used.
   */
  static const std::string JSONConfigTestString;

  ObjectAttributes(const std::string& handle = "");
  // center of mass (COM)
  void setCOM(const Magnum::Vector3& com) { setVec3("COM", com); }
  Magnum::Vector3 getCOM() const { return getVec3("COM"); }

  // whether com is provided or not
  void setComputeCOMFromShape(bool computeCOMFromShape) {
    setBool("computeCOMFromShape", computeCOMFromShape);
  }
  bool getComputeCOMFromShape() const { return getBool("computeCOMFromShape"); }

  void setMass(double mass) { setDouble("mass", mass); }
  double getMass() const { return getDouble("mass"); }

  // inertia diagonal
  void setInertia(const Magnum::Vector3& inertia) {
    setVec3("inertia", inertia);
  }
  Magnum::Vector3 getInertia() const { return getVec3("inertia"); }

  void setLinearDamping(double linearDamping) {
    setDouble("linearDamping", linearDamping);
  }
  double getLinearDamping() const { return getDouble("linearDamping"); }

  void setAngularDamping(double angularDamping) {
    setDouble("angularDamping", angularDamping);
  }
  double getAngularDamping() const { return getDouble("angularDamping"); }

  // if true override other settings and use render mesh bounding box as
  // collision object
  void setBoundingBoxCollisions(bool useBoundingBoxForCollision) {
    setBool("useBoundingBoxForCollision", useBoundingBoxForCollision);
  }
  bool getBoundingBoxCollisions() const {
    return getBool("useBoundingBoxForCollision");
  }

  // if true join all mesh components of an asset into a unified collision
  // object
  void setJoinCollisionMeshes(bool joinCollisionMeshes) {
    setBool("joinCollisionMeshes", joinCollisionMeshes);
  }
  bool getJoinCollisionMeshes() const { return getBool("joinCollisionMeshes"); }

  /**
   * @brief If not visible can add dynamic non-rendered object into a scene
   * object.  If is not visible then should not add object to drawables.
   */
  void setIsVisible(bool isVisible) { setBool("isVisible", isVisible); }
  bool getIsVisible() const { return getBool("isVisible"); }

  void setSemanticId(uint32_t semanticId) { setInt("semanticId", semanticId); }

  uint32_t getSemanticId() const { return getInt("semanticId"); }

  // if object should be checked for collisions - if other objects can collide
  // with this object
  void setIsCollidable(bool isCollidable) {
    setBool("isCollidable", isCollidable);
  }
  bool getIsCollidable() { return getBool("isCollidable"); }

 public:
  ESP_SMART_POINTERS(ObjectAttributes)

};  // class ObjectAttributes

///////////////////////////////////////
// stage attributes

//! attributes for a single stage
class StageAttributes : public AbstractObjectAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref StageAttributes
   * Has values that are different than defaults so this can be used to test
   * json loading. These values are set to be purposefully invalid, for testing
   * purposes, and so should not be used.
   */
  static const std::string JSONConfigTestString;
  StageAttributes(const std::string& handle = "");

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
    setString("semanticAssetHandle", semanticAssetHandle);
    setIsDirty();
  }
  std::string getSemanticAssetHandle() const {
    return getString("semanticAssetHandle");
  }
  void setSemanticAssetType(int semanticAssetType) {
    setInt("semanticAssetType", semanticAssetType);
  }
  int getSemanticAssetType() { return getInt("semanticAssetType"); }

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
   * @brief set lighting setup for scene.  Default value comes from
   * @ref SimulatorConfiguration, is overridden by any value set in json, if
   * exists.
   */
  void setLightSetup(const std::string& lightSetup) {
    setString("lightSetup", lightSetup);
  }
  std::string getLightSetup() { return getString("lightSetup"); }

  void setFrustrumCulling(bool frustrumCulling) {
    setBool("frustrumCulling", frustrumCulling);
  }
  bool getFrustrumCulling() const { return getBool("frustrumCulling"); }

 public:
  ESP_SMART_POINTERS(StageAttributes)

};  // class StageAttributes

}  // namespace attributes
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_OBJECTATTRIBUTES_H_
