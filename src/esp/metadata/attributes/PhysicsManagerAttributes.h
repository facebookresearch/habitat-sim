// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

//! attributes for a single physics manager
class PhysicsManagerAttributes : public AbstractAttributes {
 public:
  explicit PhysicsManagerAttributes(const std::string& handle = "");

  void setSimulator(const std::string& simulator) {
    setString("physics_simulator", simulator);
  }
  std::string getSimulator() const { return getString("physics_simulator"); }

  void setTimestep(double timestep) { setDouble("timestep", timestep); }
  double getTimestep() const { return getDouble("timestep"); }

  void setMaxSubsteps(int maxSubsteps) { setInt("max_substeps", maxSubsteps); }
  int getMaxSubsteps() const { return getInt("max_substeps"); }

  void setGravity(const Magnum::Vector3& gravity) {
    setVec3("gravity", gravity);
  }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

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

 public:
  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // class PhysicsManagerAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
