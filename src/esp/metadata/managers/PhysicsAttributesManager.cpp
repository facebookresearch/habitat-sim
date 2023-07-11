// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace esp {

namespace metadata {

using attributes::PhysicsManagerAttributes;
namespace managers {

PhysicsManagerAttributes::ptr PhysicsAttributesManager::createObject(
    const std::string& physicsFilename,
    bool registerTemplate) {
  std::string msg;
  PhysicsManagerAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      physicsFilename, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << msg << " physics manager attributes created"
        << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // PhysicsAttributesManager::createObject

void PhysicsAttributesManager::setValsFromJSONDoc(
    PhysicsManagerAttributes::ptr physicsManagerAttributes,
    const io::JsonGenericValue&
        jsonConfig) {  // load the simulator preference - default is "none"
                       // simulator, set in
  // attributes ctor.
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "physics_simulator",
      [physicsManagerAttributes](const std::string& simulator) {
        physicsManagerAttributes->setSimulator(simulator);
      });

  // load the physics timestep
  io::jsonIntoSetter<double>(jsonConfig, "timestep",
                             [physicsManagerAttributes](double timestep) {
                               physicsManagerAttributes->setTimestep(timestep);
                             });

  // load the max substeps between time step
  io::jsonIntoSetter<int>(
      jsonConfig, "max_substeps", [physicsManagerAttributes](int max_substeps) {
        physicsManagerAttributes->setMaxSubsteps(max_substeps);
      });
  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "friction_coefficient",
      [physicsManagerAttributes](double friction_coefficient) {
        physicsManagerAttributes->setFrictionCoefficient(friction_coefficient);
      });

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "restitution_coefficient",
      [physicsManagerAttributes](double restitution_coefficient) {
        physicsManagerAttributes->setRestitutionCoefficient(
            restitution_coefficient);
      });

  // load world gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      [physicsManagerAttributes](const Magnum::Vector3& gravity) {
        physicsManagerAttributes->setGravity(gravity);
      });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(physicsManagerAttributes, jsonConfig);

}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
