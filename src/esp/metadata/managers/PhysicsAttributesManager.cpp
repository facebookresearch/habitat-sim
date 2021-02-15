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
  io::jsonIntoConstSetter<std::string>(jsonConfig, "physics_simulator",
                                       [physicsManagerAttributes](auto&& PH1) {
                                         physicsManagerAttributes->setSimulator(
                                             std::forward<decltype(PH1)>(PH1));
                                       });

  // load the physics timestep
  io::jsonIntoSetter<double>(
      jsonConfig, "timestep", [physicsManagerAttributes](auto&& PH1) {
        physicsManagerAttributes->setTimestep(std::forward<decltype(PH1)>(PH1));
      });

  // load the max substeps between time step
  io::jsonIntoSetter<int>(jsonConfig, "max_substeps",
                          [physicsManagerAttributes](auto&& PH1) {
                            physicsManagerAttributes->setMaxSubsteps(
                                std::forward<decltype(PH1)>(PH1));
                          });
  // load the friction coefficient
  io::jsonIntoSetter<double>(jsonConfig, "friction_coefficient",
                             [physicsManagerAttributes](auto&& PH1) {
                               physicsManagerAttributes->setFrictionCoefficient(
                                   std::forward<decltype(PH1)>(PH1));
                             });

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonConfig, "restitution_coefficient",
      [physicsManagerAttributes](auto&& PH1) {
        physicsManagerAttributes->setRestitutionCoefficient(
            std::forward<decltype(PH1)>(PH1));
      });

  // load world gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity", [physicsManagerAttributes](auto&& PH1) {
        physicsManagerAttributes->setGravity(std::forward<decltype(PH1)>(PH1));
      });

}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
