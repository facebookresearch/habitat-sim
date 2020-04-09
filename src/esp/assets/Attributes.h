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
  PhysicsObjectAttributes();

  // default value getter/setter methods

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

  void setLinearDamping(double linearDamping) {
    setDouble("linearDamping", linearDamping);
  }
  double getLinearDamping() const { return getDouble("linearDamping"); }

  void setAngularDamping(double angularDamping) {
    setDouble("angularDamping", angularDamping);
  }
  double getAngularDamping() const { return getDouble("angularDamping"); }

  void setOriginHandle(const std::string& originHandle) {
    setString("originHandle", originHandle);
  }
  std::string getOriginHandle() const { return getString("originHandle"); }

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

  void setObjectTemplateID(int objectTemplateID) {
    setInt("objectTemplateID", objectTemplateID);
  }

  int getObjectTemplateID() const { return getInt("objectTemplateID"); }

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

  ESP_SMART_POINTERS(PhysicsObjectAttributes)

};  // end PhysicsObjectAttributes class

//! attributes for a single physical scene
class PhysicsSceneAttributes : public esp::core::Configuration {
 public:
  PhysicsSceneAttributes();

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

  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public esp::core::Configuration {
 public:
  PhysicsManagerAttributes();

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

  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // end PhysicsManagerAttributes

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_H_