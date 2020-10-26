// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/json.h"

using std::placeholders::_1;
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
    LOG(INFO) << msg << " physics manager attributes created"
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
      std::bind(&PhysicsManagerAttributes::setSimulator,
                physicsManagerAttributes, _1));

  // load the physics timestep
  io::jsonIntoSetter<double>(jsonConfig, "timestep",
                             std::bind(&PhysicsManagerAttributes::setTimestep,
                                       physicsManagerAttributes, _1));

  // load the max substeps between time step
  io::jsonIntoSetter<int>(jsonConfig, "max_substeps",
                          std::bind(&PhysicsManagerAttributes::setMaxSubsteps,
                                    physicsManagerAttributes, _1));
  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "friction_coefficient",
      std::bind(&PhysicsManagerAttributes::setFrictionCoefficient,
                physicsManagerAttributes, _1));

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "restitution_coefficient",
      std::bind(&PhysicsManagerAttributes::setRestitutionCoefficient,
                physicsManagerAttributes, _1));

  // load world gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      std::bind(&PhysicsManagerAttributes::setGravity, physicsManagerAttributes,
                _1));

}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
