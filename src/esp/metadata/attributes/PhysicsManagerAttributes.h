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
    set("physics_simulator", simulator);
  }
  std::string getSimulator() const { return getString("physics_simulator"); }

  void setTimestep(double timestep) { set("timestep", timestep); }
  double getTimestep() const { return getDouble("timestep"); }

  void setMaxSubsteps(int maxSubsteps) { set("max_substeps", maxSubsteps); }
  int getMaxSubsteps() const { return getInt("max_substeps"); }

  void setGravity(const Magnum::Vector3& gravity) { set("gravity", gravity); }
  Magnum::Vector3 getGravity() const { return getVec3("gravity"); }

  void setFrictionCoefficient(double frictionCoefficient) {
    set("friction_coefficient", frictionCoefficient);
  }
  double getFrictionCoefficient() const {
    return getDouble("friction_coefficient");
  }

  void setRestitutionCoefficient(double restitutionCoefficient) {
    set("restitution_coefficient", restitutionCoefficient);
  }
  double getRestitutionCoefficient() const {
    return getDouble("restitution_coefficient");
  }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override {
    return "Simulator Type, Timestep, Max Substeps, Gravity XYZ, Friction "
           "Coefficient, Restitution Coefficient,";
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override {
    return getSimulator()
        .append(1, ',')
        .append(std::to_string(getTimestep()))
        .append(1, ',')
        .append(std::to_string(getMaxSubsteps()))
        .append(1, ',')
        .append(getVec3AsString("gravity"))
        .append(1, ',')
        .append(std::to_string(getFrictionCoefficient()))
        .append(1, ',')
        .append(std::to_string(getRestitutionCoefficient()));
  }

 public:
  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // class PhysicsManagerAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
