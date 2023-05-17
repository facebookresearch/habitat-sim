// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief attributes class describing essential and default quantities used to
 * instantiate a physics manager.
 */
class PhysicsManagerAttributes : public AbstractAttributes {
 public:
  explicit PhysicsManagerAttributes(const std::string& handle = "");

  /**
   * @brief Sets the string name for the physics simulation engine we wish to
   * use.
   */
  void setSimulator(const std::string& simulator) {
    set("physics_simulator", simulator);
  }
  /**
   * @brief Gets the config-specified string name for the physics simulation
   * engine we wish to use.
   */
  std::string getSimulator() const {
    return get<std::string>("physics_simulator");
  }

  /**
   * @brief Sets the simulation timestep to use for dynamic simulation.
   */
  void setTimestep(double timestep) { set("timestep", timestep); }

  /**
   * @brief Get the simulation timestep to use for dynamic simulation.
   */
  double getTimestep() const { return get<double>("timestep"); }

  /**
   * @brief Currently not supported.
   */
  void setMaxSubsteps(int maxSubsteps) { set("max_substeps", maxSubsteps); }
  /**
   * @brief Currently not supported.
   */
  int getMaxSubsteps() const { return get<int>("max_substeps"); }

  /**
   * @brief Set Simulator-wide gravity.
   */
  void setGravity(const Magnum::Vector3& gravity) { set("gravity", gravity); }
  Magnum::Vector3 getGravity() const { return get<Magnum::Vector3>("gravity"); }

  void setFrictionCoefficient(double frictionCoefficient) {
    set("friction_coefficient", frictionCoefficient);
  }
  double getFrictionCoefficient() const {
    return get<double>("friction_coefficient");
  }

  void setRestitutionCoefficient(double restitutionCoefficient) {
    set("restitution_coefficient", restitutionCoefficient);
  }
  double getRestitutionCoefficient() const {
    return get<double>("restitution_coefficient");
  }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * PhysicsManagerAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override {
    return "Simulator Type,Timestep,Max Substeps,Gravity XYZ,Friction "
           "Coefficient,Restitution Coefficient,";
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override {
    return Cr::Utility::formatString(
        "{},{},{},{},{},{}", getSimulator(), getAsString("timestep"),
        getAsString("max_substeps"), getAsString("gravity"),
        getAsString("friction_coefficient"),
        getAsString("restitution_coefficient"));
  }

 public:
  ESP_SMART_POINTERS(PhysicsManagerAttributes)
};  // class PhysicsManagerAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_PHYSICSMANAGERATTRIBUTES_H_
