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

PhysicsManagerAttributes::ptr PhysicsAttributesManager::loadFromJSONDoc(
    const std::string& templateName,
    const io::JsonDocument& jsonConfig) {
  // Attributes descriptor for physics world
  PhysicsManagerAttributes::ptr physicsManagerAttributes =
      initNewObjectInternal(templateName);

  // load the simulator preference - default is "none" simulator, set in
  // attributes ctor.
  io::jsonIntoSetter<std::string>(
      jsonConfig, "physics simulator",
      std::bind(&PhysicsManagerAttributes::setSimulator,
                physicsManagerAttributes, _1));

  // load the physics timestep
  io::jsonIntoSetter<double>(jsonConfig, "timestep",
                             std::bind(&PhysicsManagerAttributes::setTimestep,
                                       physicsManagerAttributes, _1));

  // load the max substeps between time step
  io::jsonIntoSetter<int>(jsonConfig, "max substeps",
                          std::bind(&PhysicsManagerAttributes::setMaxSubsteps,
                                    physicsManagerAttributes, _1));
  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "friction coefficient",
      std::bind(&PhysicsManagerAttributes::setFrictionCoefficient,
                physicsManagerAttributes, _1));

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "restitution coefficient",
      std::bind(&PhysicsManagerAttributes::setRestitutionCoefficient,
                physicsManagerAttributes, _1));

  // load world gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      std::bind(&PhysicsManagerAttributes::setGravity, physicsManagerAttributes,
                _1));

  return physicsManagerAttributes;
}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
