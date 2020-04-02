// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

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
  void setCOM(const Magnum::Vector3& com);
  Magnum::Vector3 getCOM() const;

  // collision shape inflation margin
  void setMargin(double margin);
  double getMargin() const;

  void setMass(double mass);
  double getMass() const;

  // inertia diagonal
  void setInertia(const Magnum::Vector3& inertia);
  Magnum::Vector3 getInertia() const;

  void setScale(const Magnum::Vector3& scale);
  Magnum::Vector3 getScale() const;

  void setFrictionCoefficient(double frictionCoefficient);
  double getFrictionCoefficient() const;

  void setRestitutionCoefficient(double restitutionCoefficient);
  double getRestitutionCoefficient() const;

  void setLinearDamping(double linearDamping);
  double getLinearDamping() const;

  void setAngularDamping(double angularDamping);
  double getAngularDamping() const;

  void setOriginHandle(const std::string& originHandle);
  std::string getOriginHandle() const;

  void setRenderMeshHandle(const std::string& renderMeshHandle);
  std::string getRenderMeshHandle() const;

  void setCollisionMeshHandle(const std::string& collisionMeshHandle);
  std::string getCollisionMeshHandle() const;

  void setObjectTemplateID(int objectTemplateID);
  int getObjectTemplateID() const;

  // if true override other settings and use render mesh bounding box as
  // collision object
  void setBoundingBoxCollisions(bool useBoundingBoxForCollision);
  bool getBoundingBoxCollisions() const;

  // if true join all mesh components of an asset into a unified collision
  // object
  void setJoinCollisionMeshes(bool joinCollisionMeshes);
  bool getJoinCollisionMeshes() const;

  // if true use phong illumination model instead of flat shading
  void setRequiresLighting(bool requiresLighting);
  bool getRequiresLighting() const;

  ESP_SMART_POINTERS(PhysicsObjectAttributes)

};  // end PhysicsObjectAttributes class

//! attributes for a single physical scene
class PhysicsSceneAttributes : public esp::core::Configuration {
 public:
  PhysicsSceneAttributes();

  void setGravity(const Magnum::Vector3& gravity);
  Magnum::Vector3 getGravity() const;

  void setFrictionCoefficient(double frictionCoefficient);
  double getFrictionCoefficient() const;

  void setRestitutionCoefficient(double restitutionCoefficient);
  double getRestitutionCoefficient() const;

  void setRenderMeshHandle(const std::string& renderMeshHandle);
  std::string getRenderMeshHandle() const;

  void setCollisionMeshHandle(const std::string& collisionMeshHandle);
  std::string getCollisionMeshHandle() const;

  ESP_SMART_POINTERS(PhysicsSceneAttributes)

};  // end PhysicsSceneAttributes

//! attributes for a single physics manager
class PhysicsManagerAttributes : public esp::core::Configuration {
 public:
  PhysicsManagerAttributes();

  void setSimulator(const std::string& simulator);
  std::string getSimulator() const;

  void setTimestep(double timestep);
  double getTimestep() const;

  void setMaxSubsteps(int maxSubsteps);
  int getMaxSubsteps() const;

  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // end PhysicsManagerAttributes

}  // namespace assets
}  // namespace esp
