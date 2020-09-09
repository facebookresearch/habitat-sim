// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
#define ESP_ASSETS_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace assets {
namespace attributes {

//! attributes for a single physics manager
class PhysicsManagerAttributes : public AbstractAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref
   * PhysicsManagerAttributes. Has values that are different than defaults so
   * this can be used to test json loading. These values are set to be
   * purposefully invalid, for testing purposes, and so should not be used.
   */
  static const std::string JSONConfigTestString;
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

}  // namespace attributes
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
