// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ATTRIBUTES_H_
#define ESP_ASSETS_ATTRIBUTES_H_

//#pragma once  //remove since attributes.h might be found in other directories

#include <Magnum/Magnum.h>
#include <map>
#include <string>
#include <vector>
#include "Magnum/Math/Math.h"
#include "Magnum/Types.h"
#include "esp/core/Configuration.h"
#include "esp/gfx/magnum.h"

namespace esp {
namespace assets {

/**
 * @brief Base class for all implemented attributes.
 */
class AbstractAttributes : public esp::core::Configuration {
 public:
  AbstractAttributes(const std::string& originHandle = "") : Configuration() {
    setOriginHandle(originHandle);
  }

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param originHandle the handle to set.
   */
  void setOriginHandle(const std::string& originHandle) {
    setString("originHandle", originHandle);
  }
  std::string getOriginHandle() const { return getString("originHandle"); }
  void setObjectTemplateID(int objectTemplateID) {
    setInt("objectTemplateID", objectTemplateID);
  }
  int getObjectTemplateID() const { return getInt("objectTemplateID"); }

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
  AbstractPhysicsAttributes(const std::string& originHandle = "");
  // forcing this class to be abstract - note still needs definition
  // can't do this because of pybind issues, currently
  // virtual ~AbstractPhysAttributes() = 0;

  void setScale(const Magnum::Vector3& scale) { setVec3("scale", scale); }
  Magnum::Vector3 getScale() const { return getVec3("scale"); }

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

  bool getIsDirty() const { return getBool("__isDirty"); }
  void setIsClean() { setBool("__isDirty", false); }

 protected:
  void setIsDirty() { setBool("__isDirty", true); }
  std::string getBoolDispStr(bool val) const {
    return (val ? "true" : "false");
  }

 public:
  ESP_SMART_POINTERS(AbstractPhysicsAttributes)

};  // namespace assets

/**
 * @brief Specific Attributes instance which is constructed with a base set of
 * physics object required attributes
 */
class PhysicsObjectAttributes : public AbstractPhysicsAttributes {
 public:
  PhysicsObjectAttributes(const std::string& originHandle = "");
  // center of mass (COM)
  void setCOM(const Magnum::Vector3& com) { setVec3("COM", com); }
  Magnum::Vector3 getCOM() const { return getVec3("COM"); }

  // collision shape inflation margin
  void setMargin(double margin) { setDouble("margin", margin); }
  double getMargin() const { return getDouble("margin"); }

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

  // if true use phong illumination model instead of flat shading
  void setRequiresLighting(bool requiresLighting) {
    setBool("requiresLighting", requiresLighting);
  }
  bool getRequiresLighting() const { return getBool("requiresLighting"); }

  // if object is visible
  void setIsVisible(bool isVisible) { setBool("isVisible", isVisible); }
  bool getIsVisible() const { return getBool("isVisible"); }

  // if object should be checked for collisions - if other objects can collide
  // with this object
  void setIsCollidable(bool isCollidable) {
    setBool("isCollidable", isCollidable);
  }
  bool getIsCollidable() { return getBool("isCollidable"); }

  ESP_SMART_POINTERS(PhysicsObjectAttributes)

};  // end PhysicsObjectAttributes class

///////////////////////////////////////
// scene and physics manager attributes

//! attributes for a single physical scene
class PhysicsSceneAttributes : public AbstractPhysicsAttributes {
 public:
  PhysicsSceneAttributes(const std::string& originHandle = "");

  void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public AbstractAttributes {
 public:
  PhysicsManagerAttributes() : PhysicsManagerAttributes("") {}
  PhysicsManagerAttributes(const std::string& originHandle);

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

  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // end PhysicsManagerAttributes

///////////////////////////////////
// primitive object attributes

//! attributes describing primitve render/collision objects - abstract class
//! without pure virtual methods
class AbstractPrimitiveAttributes : public AbstractAttributes {
 public:
  AbstractPrimitiveAttributes(bool isWireframe,
                              int primObjType,
                              const std::string& primObjClassName)
      : AbstractAttributes("") {
    setIsWireframe(isWireframe);
    setPrimObjType(primObjType);
    setPrimObjClassName(primObjClassName);

    if (!isWireframe) {  // solid
      setUseTextureCoords(false);
      setUseTangents(false);
    }
  }  // ctor

  // forcing this class to be abstract - note still needs definition of
  // destructor : Cannot use this due to pybind issues
  // virtual ~AbstractPrimitiveAttributes() = 0;

  // originHandle is set internally based on attributes configuration
  // setting externally is prohibited
  void setOriginHandle(const std::string&) {}

  bool getIsWireframe() const { return getBool("isWireframe"); }

  // only solid prims can use texture coords
  void setUseTextureCoords(bool useTextureCoords) {
    if (!getIsWireframe()) {  // check if solid
      setBool("textureCoordinates", useTextureCoords);
      buildOriginHandle();  // build handle based on config
    }
  }
  bool getUseTextureCoords() const { return getBool("textureCoordinates"); }

  // only solid prims have option to use tangents
  void setUseTangents(bool tangents) {
    if (!getIsWireframe()) {  // check if solid
      setBool("tangents", tangents);
      buildOriginHandle();  // build handle based on config
    }
  }
  bool getUseTangents() const { return getBool("tangents"); }

  // only circular prims set number of rings - NOTE : capsule sets rings
  // separately for hemispheres and cylinder
  // set virtual so cannot be deleted in capsule attributes
  void setNumRings(int rings) {
    setInt("rings", rings);
    buildOriginHandle();  // build handle based on config
  }
  int getNumRings() const { return getInt("rings"); }

  void setNumSegments(int segments) {
    setInt("segments", segments);
    buildOriginHandle();  // build handle based on config
  }
  int getNumSegments() const { return getInt("segments"); }
  // capsule, cone and cylinder use halfLength
  void setHalfLength(double halfLength) { setDouble("halfLength", halfLength); }
  double getHalfLength() const { return getDouble("halfLength"); }

  /**
   * @brief Returns configuration to be used with PrimitiveImporter to
   * instantiate Primitives.  Names in getter/setters chosen to match parameter
   * name expectations in PrimitiveImporter.
   *
   * @return the underlying configuration group for this attributes object
   */
  Corrade::Utility::ConfigurationGroup getConfigGroup() const { return cfg; }

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

 private:
  // Should never change, only set by ctor
  void setPrimObjClassName(std::string primObjClassName) {
    setString("primObjClassName", primObjClassName);
  }

  // Should never change, only set by ctor
  void setPrimObjType(int primObjType) { setInt("primObjType", primObjType); }

  // not used to construct prim mesh, so setting this does not require
  // modification to origin handle.  Should never change, only set by ctor
  void setIsWireframe(bool isWireframe) { setBool("isWireframe", isWireframe); }

 protected:
  /**
   * @brief Verifies that @ref val is larger than, and a multiple of, divisor
   * div
   * @param val the value to check
   * @param div the divsior (value to verify is greater than and a multiple of)
   * - will be either 2 or 4 for primitives value checking
   * @return whether check passes
   */
  bool isValMultOfDiv(int val, int div) {
    return (val >= div) && (val % div == 0);
  }

  /**
   * @brief Origin handle for primitive attribute-based templates should reflect
   * the parameters used to construct the primitive, and so should only be set
   * internally
   */
  void buildOriginHandle() {
    std::ostringstream oHndlStrm;
    oHndlStrm << getPrimObjClassName() << buildOriginHandleDetail();
    setString("originHandle", oHndlStrm.str());
  }
  // helper for origin handle construction
  std::string getBoolDispStr(bool val) const {
    return (val ? "true" : "false");
  }
  virtual std::string buildOriginHandleDetail() { return ""; }

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
    buildOriginHandle();  // build handle based on config
  }
  int getHemisphereRings() const { return getInt("hemisphereRings"); }

  void setCylinderRings(int cylinderRings) {
    setInt("cylinderRings", cylinderRings);
    buildOriginHandle();  // build handle based on config
  }
  int getCylinderRings() const { return getInt("cylinderRings"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  virtual bool isValidTemplate() override {
    bool wfCheck = ((getIsWireframe() && isValMultOfDiv(getNumSegments(), 4)) ||
                    (!getIsWireframe() && getNumSegments() > 2));

    return (getCylinderRings() > 0 && getHemisphereRings() > 0 && wfCheck &&
            getHalfLength() > 0);
  }

 protected:
  virtual std::string buildOriginHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_hemiRings_" << getHemisphereRings() << "_cylRings_"
              << getCylinderRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildOriginHandleDetail

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
    buildOriginHandle();  // build handle based on config
  }
  bool getCapEnd() const { return getBool("capEnd"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  virtual bool isValidTemplate() override {
    bool wfCheck =
        ((getIsWireframe() && isValMultOfDiv(getNumSegments(), 4)) ||
         (!getIsWireframe() && getNumSegments() > 2 && getNumRings() > 0));

    return (getHalfLength() > 0 && wfCheck);
  }

 protected:
  virtual std::string buildOriginHandleDetail() override {
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
  }  // buildOriginHandleDetail

  ESP_SMART_POINTERS(ConePrimitiveAttributes)
};  // class ConePrimitiveAttributes

class CubePrimitiveAttributes : public AbstractPrimitiveAttributes {
 public:
  CubePrimitiveAttributes(bool isWireframe,
                          int primObjType,
                          const std::string& primObjClassName)
      : AbstractPrimitiveAttributes(isWireframe,
                                    primObjType,
                                    primObjClassName) {
    buildOriginHandle();  // build handle based on config
  }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type. Cube
   * primitives require no values and so this attributes is always valid.
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  virtual bool isValidTemplate() override { return true; }

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
    buildOriginHandle();  // build handle based on config
  }
  bool getCapEnds() const { return getBool("capEnds"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  virtual bool isValidTemplate() override {
    bool wfCheck = ((getIsWireframe() && isValMultOfDiv(getNumSegments(), 4)) ||
                    (!getIsWireframe() && getNumSegments() > 2));
    return getNumRings() > 0 && getHalfLength() > 0 && wfCheck;
  }

 protected:
  virtual std::string buildOriginHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments()
              << "_halfLen_" << getHalfLength();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents())
                << "_capEnds_" << getBoolDispStr(getCapEnds());
    }
    return oHndlStrm.str();
  }  // buildOriginHandleDetail

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
                                    primObjClassName) {
    // setting manually because wireframe icosphere does not currently support
    // subdiv > 1 and setSubdivisions checks for wireframe
    setInt("subdivisions", 1);
    buildOriginHandle();  // build handle based on config
  }
  // only solid icospheres will support subdivision - wireframes default to 1
  void setSubdivisions(int subdivisions) {
    if (!getIsWireframe()) {
      setInt("subdivisions", subdivisions);
      buildOriginHandle();  // build handle based on config
    }
  }
  int getSubdivisions() const { return getInt("subdivisions"); }

  /**
   * @brief This will determine if the stated template has the required
   * quantities needed to instantiate a primitive properly of desired type
   * @return whether or not the template holds valid data for desired primitive
   * type
   */
  virtual bool isValidTemplate() override {
    return (getIsWireframe() || (!getIsWireframe() && getSubdivisions() >= 0));
  }

 protected:
  virtual std::string buildOriginHandleDetail() override {
    std::ostringstream oHndlStrm;
    // wireframe subdivision currently does not change
    // but think about the possibilities.
    oHndlStrm << "_subdivs_" << getSubdivisions();
    return oHndlStrm.str();
  }  // buildOriginHandleDetail

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
  virtual bool isValidTemplate() override {
    return ((getIsWireframe() && isValMultOfDiv(getNumSegments(), 4) &&
             isValMultOfDiv(getNumRings(), 2)) ||
            (!getIsWireframe() && getNumRings() > 1 && getNumSegments() > 2));
  }

 protected:
  virtual std::string buildOriginHandleDetail() override {
    std::ostringstream oHndlStrm;
    oHndlStrm << "_rings_" << getNumRings() << "_segments_" << getNumSegments();
    if (!getIsWireframe()) {
      oHndlStrm << "_useTexCoords_" << getBoolDispStr(getUseTextureCoords())
                << "_useTangents_" << getBoolDispStr(getUseTangents());
    }
    return oHndlStrm.str();
  }  // buildOriginHandleDetail

  ESP_SMART_POINTERS(UVSpherePrimitiveAttributes)
};  // class UVSpherePrimitiveAttributes

///////////////////////////////////////
// end primitive object attributes

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_H_