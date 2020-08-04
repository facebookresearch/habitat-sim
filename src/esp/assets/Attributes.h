// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ATTRIBUTES_H_
#define ESP_ASSETS_ATTRIBUTES_H_

#include <Magnum/Magnum.h>
#include <map>
#include <string>
#include <vector>
#include "Magnum/Math/Math.h"
#include "Magnum/Types.h"
#include "esp/core/Configuration.h"

namespace esp {
namespace assets {

/**
 * @brief Base class for all implemented attributes.
 */
class AbstractAttributes : public esp::core::Configuration {
 public:
  AbstractAttributes(const std::string& attributesClassKey,
                     const std::string& handle)
      : Configuration() {
    setAttributesClassKey(attributesClassKey);
    setHandle(handle);
  }

  virtual ~AbstractAttributes() = default;
  /**
   * @brief Get this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   */
  std::string getClassKey() const { return getString("attributesClassKey"); }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  virtual void setHandle(const std::string& handle) {
    setString("handle", handle);
  }
  std::string getHandle() const { return getString("handle"); }

  /**
   * @brief directory where files used to construct attributes can be found.
   */
  virtual void setFileDirectory(const std::string& fileDirectory) {
    setString("fileDirectory", fileDirectory);
  }
  std::string getFileDirectory() const { return getString("fileDirectory"); }

  void setID(int ID) { setInt("ID", ID); }
  int getID() const { return getInt("ID"); }

  /**
   * @brief Returns configuration to be used with PrimitiveImporter to
   * instantiate Primitives.  Names in getter/setters chosen to match parameter
   * name expectations in PrimitiveImporter.
   *
   * @return a reference to the underlying configuration group for this
   * attributes object
   */
  const Corrade::Utility::ConfigurationGroup& getConfigGroup() const {
    return cfg;
  }

 protected:
  /**
   * @brief Set this attributes' class.  Should only be set from constructor.
   * Used as key in constructor function pointer maps in AttributesManagers.
   * @param attributesClassKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setAttributesClassKey(const std::string& attributesClassKey) {
    setString("attributesClassKey", attributesClassKey);
  }

 public:
  ESP_SMART_POINTERS(AbstractAttributes)
};  // class AbstractAttributes

/**
 * @brief base attributes object holding attributes shared by all
 * PhysicsObjectAttributes and PhysicsSceneAttributes objects; Should be treated
 * as if is abstract - should never be instanced directly
 */
class AbstractPhysicsAttributes : public AbstractAttributes {
 public:
  AbstractPhysicsAttributes(const std::string& classKey,
                            const std::string& handle);

  virtual ~AbstractPhysicsAttributes() = default;
  void setScale(const Magnum::Vector3& scale) { setVec3("scale", scale); }
  Magnum::Vector3 getScale() const { return getVec3("scale"); }

  /**
   * @brief collision shape inflation margin
   */
  void setMargin(double margin) { setDouble("margin", margin); }
  double getMargin() const { return getDouble("margin"); }

  /**
   * @brief set default up orientation for object/scene mesh
   */
  void setOrientUp(const Magnum::Vector3& orientUp) {
    setVec3("orientUp", orientUp);
  }
  /**
   * @brief get default up orientation for object/scene mesh
   */
  Magnum::Vector3 getOrientUp() const { return getVec3("orientUp"); }
  /**
   * @brief set default forwardd orientation for object/scene mesh
   */
  void setOrientFront(const Magnum::Vector3& orientFront) {
    setVec3("orientFront", orientFront);
  }
  /**
   * @brief get default forwardd orientation for object/scene mesh
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
  std::string getBoolDispStr(bool val) const {
    return (val ? "true" : "false");
  }

 public:
  ESP_SMART_POINTERS(AbstractPhysicsAttributes)

};  // class AbstractPhysicsAttributes

/**
 * @brief Specific Attributes instance which is constructed with a base set of
 * physics object required attributes
 */
class PhysicsObjectAttributes : public AbstractPhysicsAttributes {
 public:
  PhysicsObjectAttributes(const std::string& handle = "");
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
  ESP_SMART_POINTERS(PhysicsObjectAttributes)

};  // class PhysicsObjectAttributes

///////////////////////////////////////
// scene and physics manager attributes

//! attributes for a single physical scene
class PhysicsSceneAttributes : public AbstractPhysicsAttributes {
 public:
  PhysicsSceneAttributes(const std::string& handle = "");

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
  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // class PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public AbstractAttributes {
 public:
  PhysicsManagerAttributes(const std::string& handle = "");

  void setSimulator(const std::string& simulator) {
    setString("simulator", simulator);
  }
  std::string getSimulator() const { return getString("simulator"); }

  void setTimestep(double timestep) { setDouble("timestep", timestep); }
  double getTimestep() const { return getDouble("timestep"); }

  void setMaxSubsteps(int maxSubsteps) { setInt("maxSubsteps", maxSubsteps); }
  int getMaxSubsteps() const { return getInt("maxSubsteps"); }

  void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

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

 public:
  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // class PhysicsManagerAttributes

///////////////////////////////////
// primitive object attributes

//! attributes describing primitve render/collision objects - abstract class
//! without pure virtual methods
class AbstractPrimitiveAttributes : public AbstractAttributes {
 public:
  AbstractPrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName,
                              const std::string& attributesClassKey)
      : AbstractAttributes(attributesClassKey, "") {
    setIsWireframe(isWireframe);
    setPrimObjType(primObjType);
    setPrimObjClassName(primObjClassName);
    setFileDirectory("none");

    if (!isWireframe) {  // solid
      // do not call setters since they call buildHandle, which does not
      // exist - is abstract in base class
      setBool("textureCoordinates", false);
      setBool("tangents", false);
    }
  }  // ctor

  // necessary since abstract
  virtual ~AbstractPrimitiveAttributes() = default;

  // handle is set internally based on attributes configuration
  // setting externally is prohibited
  void setHandle(const std::string&) override {}

  bool getIsWireframe() const { return getBool("isWireframe"); }

  // only solid prims can use texture coords
  void setUseTextureCoords(bool useTextureCoords) {
    if (!getIsWireframe()) {  // check if solid
      setBool("textureCoordinates", useTextureCoords);
      buildHandle();  // build handle based on config
    }
  }
  bool getUseTextureCoords() const { return getBool("textureCoordinates"); }

  // only solid prims have option to use tangents
  void setUseTangents(bool tangents) {
    if (!getIsWireframe()) {  // check if solid
      setBool("tangents", tangents);
      buildHandle();  // build handle based on config
    }
  }
  bool getUseTangents() const { return getBool("tangents"); }

  // only circular prims set number of rings - NOTE : capsule sets rings
  // separately for hemispheres and cylinder
  // set virtual so cannot be deleted in capsule attributes
  void setNumRings(int rings) {
    setInt("rings", rings);
    buildHandle();  // build handle based on config
  }
  int getNumRings() const { return getInt("rings"); }

  void setNumSegments(int segments) {
    setInt("segments", segments);
    buildHandle();  // build handle based on config
  }
  int getNumSegments() const { return getInt("segments"); }
  // capsule, cone and cylinder use halfLength
  void setHalfLength(double halfLength) { setDouble("halfLength", halfLength); }
  double getHalfLength() const { return getDouble("halfLength"); }

  std::string getPrimObjClassName() const {
    return getString("primObjClassName");
  }

  int getPrimObjType() const { return getInt("primObjType"); }
  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type.
   * AbstractPrimitiveAttributes is never valid for any primitive - should be
   * overridden in prim-specific class.
   * @return whether or not the template holds valid data for desired primitive
   * type.
   */
  virtual bool isValidTemplate() { return false; }

  /**
   * @brief Handle for primitive attribute-based templates should reflect
   * the parameters used to construct the primitive, and so should only be set
   * internally or when relevant values are set manually.
   */
  void buildHandle() {
    std::ostringstream oHndlStrm;
    oHndlStrm << getPrimObjClassName() << buildHandleDetail();
    setString("handle", oHndlStrm.str());
  }

 protected:
  /**
   * @brief Verifies that @ref val is larger than, and a multiple of, divisor
   * div
   * @param val the value to check
   * @param div the divsior (value to verify is greater than and a multiple of)
   * - will be either 2 or 4 for primitives value checking
   * @return whether check passes
   */
  bool isValueMultipleOfDivisor(int val, int div) {
    return (val >= div) && (val % div == 0);
  }

  // helper for handle construction
  std::string getBoolDispStr(bool val) const {
    return (val ? "true" : "false");
  }
  virtual std::string buildHandleDetail() = 0;

 private:
  // Should never change, only set by ctor
  void setPrimObjClassName(std::string primObjClassName) {
    setString("primObjClassName", primObjClassName);
  }

  // Should never change, only set by ctor
  void setPrimObjType(int primObjType) { setInt("primObjType", primObjType); }

  // not used to construct prim mesh, so setting this does not require
  // modification to handle.  Should never change, only set by ctor
  void setIsWireframe(bool isWireframe) { setBool("isWireframe", isWireframe); }

 public:
  ESP_SMART_POINTERS(AbstractPrimitiveAttributes)
};  // class AbstractPrimitiveAttributes

//! attributes describing primitive capsule objects
class CapsulePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CapsulePrimitiveAttributes(bool isWireframe,
                             int primObjType,
                             const std::string& primObjClassName);

  void setHemisphereRings(int hemisphereRings) {
    setInt("hemisphereRings", hemisphereRings);
    buildHandle();  // build handle based on config
  }
  int getHemisphereRings() const { return getInt("hemisphereRings"); }

  void setCylinderRings(int cylinderRings) {
    setInt("cylinderRings", cylinderRings);
    buildHandle();  // build handle based on config
  }
  int getCylinderRings() const { return getInt("cylinderRings"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2));

    return (getCylinderRings() > 0 && getHemisphereRings() > 0 && wfCheck &&
            getHalfLength() > 0);
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_hemiRings_" << getHemisphereRings() << "_cylRings_"
              << getCylinderRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

 public:
  ESP_SMART_POINTERS(CapsulePrimitiveAttributes)
};  // class CapsulePrimitiveAttributes

class ConePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  ConePrimitiveAttributes(bool isWireframe,
                          int primObjType,
                          const std::string& primObjClassName);

  // only solid cones can have end capped
  void setCapEnd(bool capEnd) {
    setBool("capEnd", capEnd);
    buildHandle();  // build handle based on config
  }
  bool getCapEnd() const { return getBool("capEnd"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2 && getNumRings() > 0));

    return (getHalfLength() > 0 && wfCheck);
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_segments_" << getNumSegments() << "_halfLen_"
              << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_rings_" << getNumRings() << "_useTexCoords_"
                << getBoolDispStr(getUseTextureCoords()) << "_useTangents_"
                << getBoolDispStr(getUseTangents()) << "_capEnd_"
                << getBoolDispStr(getCapEnd());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

 public:
  ESP_SMART_POINTERS(ConePrimitiveAttributes)
};  // class ConePrimitiveAttributes

class CubePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CubePrimitiveAttributes(bool isWireframe,
                          int primObjType,
                          const std::string& primObjClassName)
      : AbstractPrimitiveAttributes(isWireframe,
                                    primObjType,
                                    primObjClassName,
                                    "CubePrimitiveAttributes") {
    buildHandle();  // build handle based on config
  }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type. Cube
   * primitives require no values and so this attributes is always valid.
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override { return true; }

 protected:
  std::string buildHandleDetail() override { return ""; }

 public:
  ESP_SMART_POINTERS(CubePrimitiveAttributes)
};  // class CubePrimitiveAttributes

class CylinderPrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CylinderPrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName);

  // only solid culinders can have ends capped
  void setCapEnds(bool capEnds) {
    setBool("capEnds", capEnds);
    buildHandle();  // build handle based on config
  }
  bool getCapEnds() const { return getBool("capEnds"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValueMultipleOfDivisor(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2));
    return getNumRings() > 0 && getHalfLength() > 0 && wfCheck;
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents())
                << "_capEnds_" << getBoolDispStr(getCapEnds());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

 public:
  ESP_SMART_POINTERS(CylinderPrimitiveAttributes)
};  // class CylinderPrimitiveAttributes

class IcospherePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  // note there is no magnum primitive implementation of a wireframe icosphere
  IcospherePrimitiveAttributes(bool isWireframe,
                               int primObjType,
                               const std::string& primObjClassName)
      : AbstractPrimitiveAttributes(isWireframe,
                                    primObjType,
                                    primObjClassName,
                                    "IcospherePrimitiveAttributes") {
    // setting manually because wireframe icosphere does not currently support
    // subdiv > 1 and setSubdivisions checks for wireframe
    setInt("subdivisions", 1);
    buildHandle();  // build handle based on config
  }
  // only solid icospheres will support subdivision - wireframes default to 1
  void setSubdivisions(int subdivisions) {
    if (!getIsWireframe()) {
      setInt("subdivisions", subdivisions);
      buildHandle();  // build handle based on config
    }
  }
  int getSubdivisions() const { return getInt("subdivisions"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override {
    return (getIsWireframe() || (!getIsWireframe() && getSubdivisions() >= 0));
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    // wireframe subdivision currently does not change
    // but think about the possibilities.
    oHndlStrm << "_subdivs_" << getSubdivisions();
    return oHndlStrm.str();
  }  // buildHandleDetail

 public:
  ESP_SMART_POINTERS(IcospherePrimitiveAttributes)
};  // class IcospherePrimitiveAttributes

class UVSpherePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  UVSpherePrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName);

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  bool isValidTemplate() override {
    return ((getIsWireframe() &&
             isValueMultipleOfDivisor(getNumSegments(), 4) &&
             isValueMultipleOfDivisor(getNumRings(), 2)) ||
            (!getIsWireframe() && getNumRings() > 1 && getNumSegments() > 2));
  }

 protected:
  std::string buildHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildHandleDetail

 public:
  ESP_SMART_POINTERS(UVSpherePrimitiveAttributes)
};  // class UVSpherePrimitiveAttributes

///////////////////////////////////////
// end primitive object attributes

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_H_
