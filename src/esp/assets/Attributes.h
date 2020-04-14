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
 * @brief base attributes object holding attributes shared by all
 * PhysicsXXXAttributes objects; Is abstract - should never be instanced
 */
class AbstractPhysAttributes : public esp::core::Configuration {
 public:
  AbstractPhysAttributes(const std::string& originHandle);
  // forcing this class to be abstract - note still needs definition
  virtual ~AbstractPhysAttributes() = 0;
  void setOriginHandle(const std::string& originHandle) {
    setString("originHandle", originHandle);
  }
  std::string getOriginHandle() const { return getString("originHandle"); }
  void setObjectTemplateID(int objectTemplateID) {
    setInt("objectTemplateID", objectTemplateID);
  }
  int getObjectTemplateID() const { return getInt("objectTemplateID"); }

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

  void setRenderMeshHandle(const std::string& renderMeshHandle) {
    setString("renderMeshHandle", renderMeshHandle);
  }
  std::string getRenderMeshHandle() const {
    return getString("renderMeshHandle");
  }

  void setCollisionMeshHandle(const std::string& collisionMeshHandle) {
    setString("collisionMeshHandle", collisionMeshHandle);
  }
  std::string getCollisionMeshHandle() const {
    return getString("collisionMeshHandle");
  }

  ESP_SMART_POINTERS(AbstractPhysAttributes)

};  // namespace assets

/**
 * @brief Specific Attributes instance which is constructed with a base set of
 * physics object required attributes
 */
class PhysicsObjectAttributes : public AbstractPhysAttributes {
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

  void setScale(const Magnum::Vector3& scale) { setVec3("scale", scale); }
  Magnum::Vector3 getScale() const { return getVec3("scale"); }

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

///////////////////////////////////
// primitive objects

//! attributes describing primitve objects - abstract class without pure virtual
//! methods
class AbstractPhysPrimObjAttributes : public PhysicsObjectAttributes {
 public:
  AbstractPhysPrimObjAttributes(bool isWireframe,
                                int primType,
                                const std::string& originHndl)
      : PhysicsObjectAttributes(originHndl) {
    setIsWireframe(isWireframe);
    setPrimObjType(primType);
  }  // ctor
  // forcing this class to be abstract - note still needs definition of
  // destructor
  virtual ~AbstractPhysPrimObjAttributes() = 0;

  void setIsWireframe(bool isWireframe) { setBool("isWireframe", isWireframe); }
  bool getIsWireframe() { return getBool("isWireframe"); }

  void setPrimObjType(int primObjType) { setInt("primObjType", primObjType); }
  int getPrimObjType() { return getInt("primObjType"); }

  ESP_SMART_POINTERS(AbstractPhysPrimObjAttributes)
};  // class PhysicsPrimitiveObjAttributes

//! attributes describing primitive capsule objects
class PhysicsCapsulePrimAttributes : public AbstractPhysPrimObjAttributes {
  PhysicsCapsulePrimAttributes(bool isWireframe,
                               int primType,
                               const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsCapsulePrimAttributes)
};  // class PhysicsCapsulePrimAttributes

class PhysicsConePrimAttributes : public AbstractPhysPrimObjAttributes {
  PhysicsConePrimAttributes(bool isWireframe,
                            int primType,
                            const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsConePrimAttributes)
};  // class PhysicsConePrimAttributes

class PhysicsCubePrimAttributes : public AbstractPhysPrimObjAttributes {
  PhysicsCubePrimAttributes(bool isWireframe,
                            int primType,
                            const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsCubePrimAttributes)
};  // class PhysicsCubePrimAttributes

class PhysicsCylinderPrimAttributes : public AbstractPhysPrimObjAttributes {
  PhysicsCylinderPrimAttributes(bool isWireframe,
                                int primType,
                                const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsCylinderPrimAttributes)
};  // class PhysicsCylinderPrimAttributes

class PhysicsIcospherePrimAttributes : public AbstractPhysPrimObjAttributes {
  // note there is no magnum primitive implementation of a wireframe icosphere
  PhysicsIcospherePrimAttributes(bool isWireframe,
                                 int primType,
                                 const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsIcospherePrimAttributes)
};  // class PhysicsIcospherePrimAttributes

class PhysicsUVSpherePrimAttributes : public AbstractPhysPrimObjAttributes {
  PhysicsUVSpherePrimAttributes(bool isWireframe,
                                int primType,
                                const std::string& originHndl)
      : AbstractPhysPrimObjAttributes(isWireframe, primType, originHndl) {}

  ESP_SMART_POINTERS(PhysicsUVSpherePrimAttributes)
};  // class PhysicsUVSpherePrimAttributes

///////////////////////////////////////
// scene and physics manager attributes

//! attributes for a single physical scene
class PhysicsSceneAttributes : public AbstractPhysAttributes {
 public:
  PhysicsSceneAttributes(const std::string& originHandle = "");

  void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public esp::core::Configuration {
 public:
  PhysicsManagerAttributes(const std::string& originHandle = "");

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
  void setOriginHandle(const std::string& originHandle) {
    setString("originHandle", originHandle);
  }
  std::string getOriginHandle() const { return getString("originHandle"); }
  void setObjectTemplateID(int objectTemplateID) {
    setInt("objectTemplateID", objectTemplateID);
  }
  int getObjectTemplateID() const { return getInt("objectTemplateID"); }

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

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_H_