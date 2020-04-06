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
 * @brief Specific Attributes instance which is constructed with a base set of
 * physics object required attributes
 */
class PhysicsObjectAttributes : public esp::core::Configuration {
 public:
  PhysicsObjectAttributes() : Configuration() {
    // fill necessary attribute defaults
    setMass(1.0);
    setMargin(0.01);
    setScale({1.0, 1.0, 1.0});
    setCOM({0, 0, 0});
    setInertia({0, 0, 0});
    setFrictionCoefficient(0.5);
    setRestitutionCoefficient(0.1);
    setLinearDamping(0.2);
    setAngularDamping(0.2);
    setOriginHandle("");
    setRenderMeshHandle("");
    setCollisionMeshHandle("");
    setBoundingBoxCollisions(false);
    setJoinCollisionMeshes(true);
    setRequiresLighting(true);
  }

  // default value getter/setter methods

  // center of mass (COM)
  inline void setCOM(const Magnum::Vector3& com) { setVec3("COM", com); }
  inline Magnum::Vector3 getCOM() const { return getVec3("COM"); }

  // collision shape inflation margin
  inline void setMargin(double margin) { setDouble("margin", margin); }
  inline double getMargin() const { return getDouble("margin"); }

  inline void setMass(double mass) { setDouble("mass", mass); }
  inline double getMass() const { return getDouble("mass"); }

  // inertia diagonal
  inline void setInertia(const Magnum::Vector3& inertia) {
    setVec3("inertia", inertia);
  }
  inline Magnum::Vector3 getInertia() const { return getVec3("inertia"); }

  inline void setScale(const Magnum::Vector3& scale) {
    setVec3("scale", scale);
  }
  inline Magnum::Vector3 getScale() const { return getVec3("scale"); }

  inline void setFrictionCoefficient(double frictionCoefficient) {
    setDouble("frictionCoefficient", frictionCoefficient);
  }
  inline double getFrictionCoefficient() const {
    return getDouble("frictionCoefficient");
  }

  inline void setRestitutionCoefficient(double restitutionCoefficient) {
    setDouble("restitutionCoefficient", restitutionCoefficient);
  }
  inline double getRestitutionCoefficient() const {
    return getDouble("restitutionCoefficient");
  }

  inline void setLinearDamping(double linearDamping) {
    setDouble("linearDamping", linearDamping);
  }
  inline double getLinearDamping() const { return getDouble("linearDamping"); }

  inline void setAngularDamping(double angularDamping) {
    setDouble("angularDamping", angularDamping);
  }
  inline double getAngularDamping() const {
    return getDouble("angularDamping");
  }

  inline void setOriginHandle(const std::string& originHandle) {
    setString("originHandle", originHandle);
  }
  inline std::string getOriginHandle() const {
    return getString("originHandle");
  }

  inline void setRenderMeshHandle(const std::string& renderMeshHandle) {
    setString("renderMeshHandle", renderMeshHandle);
  }
  inline std::string getRenderMeshHandle() const {
    return getString("renderMeshHandle");
  }

  inline void setCollisionMeshHandle(const std::string& collisionMeshHandle) {
    setString("collisionMeshHandle", collisionMeshHandle);
  }
  inline std::string getCollisionMeshHandle() const {
    return getString("collisionMeshHandle");
  }

  inline void setObjectTemplateID(int objectTemplateID) {
    setInt("objectTemplateID", objectTemplateID);
  }

  inline int getObjectTemplateID() const { return getInt("objectTemplateID"); }

  // if true override other settings and use render mesh bounding box as
  // collision object
  inline void setBoundingBoxCollisions(bool useBoundingBoxForCollision) {
    setBool("useBoundingBoxForCollision", useBoundingBoxForCollision);
  }
  inline bool getBoundingBoxCollisions() const {
    return getBool("useBoundingBoxForCollision");
  }

  // if true join all mesh components of an asset into a unified collision
  // object
  inline void setJoinCollisionMeshes(bool joinCollisionMeshes) {
    setBool("joinCollisionMeshes", joinCollisionMeshes);
  }
  inline bool getJoinCollisionMeshes() const {
    return getBool("joinCollisionMeshes");
  }

  // if true use phong illumination model instead of flat shading
  inline void setRequiresLighting(bool requiresLighting) {
    setBool("requiresLighting", requiresLighting);
  }
  inline bool getRequiresLighting() const {
    return getBool("requiresLighting");
  }

  ESP_SMART_POINTERS(PhysicsObjectAttributes)

};  // end PhysicsObjectAttributes class

//! attributes for a single physical scene
class PhysicsSceneAttributes : public esp::core::Configuration {
 public:
  PhysicsSceneAttributes() : Configuration() {
    setGravity({0, -9.8, 0});
    setFrictionCoefficient(0.4);
    setRestitutionCoefficient(0.05);
    setRenderMeshHandle("");
    setCollisionMeshHandle("");
  }

  inline void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  inline Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

  inline void setFrictionCoefficient(double frictionCoefficient) {
    setDouble("frictionCoefficient", frictionCoefficient);
  }
  inline double getFrictionCoefficient() const {
    return getDouble("frictionCoefficient");
  }

  inline void setRestitutionCoefficient(double restitutionCoefficient) {
    setDouble("restitutionCoefficient", restitutionCoefficient);
  }
  inline double getRestitutionCoefficient() const {
    return getDouble("restitutionCoefficient");
  }

  inline void setRenderMeshHandle(const std::string& renderMeshHandle) {
    setString("renderMeshHandle", renderMeshHandle);
  }
  inline std::string getRenderMeshHandle() const {
    return getString("renderMeshHandle");
  }

  inline void setCollisionMeshHandle(const std::string& collisionMeshHandle) {
    setString("collisionMeshHandle", collisionMeshHandle);
  }
  inline std::string getCollisionMeshHandle() const {
    return getString("collisionMeshHandle");
  }

  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public esp::core::Configuration {
 public:
  PhysicsManagerAttributes() : Configuration() {
    setSimulator("none");
    setTimestep(0.01);
    setMaxSubsteps(10);
  }

  inline void setSimulator(const std::string& simulator) {
    setString("simulator", simulator);
  }
  inline std::string getSimulator() const { return getString("simulator"); }

  inline void setTimestep(double timestep) { setDouble("timestep", timestep); }
  inline double getTimestep() const { return getDouble("timestep"); }

  inline void setMaxSubsteps(int maxSubsteps) {
    setInt("maxSubsteps", maxSubsteps);
  }
  inline int getMaxSubsteps() const { return getInt("maxSubsteps"); }

  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // end PhysicsManagerAttributes

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_H_