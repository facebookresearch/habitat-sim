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
    Attrs::PhysicsManagerAttributes::ptr physicsManagerAttributes,
    const io::JsonGenericValue&
        jsonConfig) {  // load the simulator preference - default is "none"
                       // simulator, set in
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

  // load the rigid object library metadata (no physics init yet...)
  if (jsonConfig.HasMember("rigid object paths") &&
      jsonConfig["rigid object paths"].IsArray()) {
    std::string configDirectory = physicsManagerAttributes->getFileDirectory();

    const auto& paths = jsonConfig["rigid object paths"];
    for (rapidjson::SizeType i = 0; i < paths.Size(); i++) {
      if (!paths[i].IsString()) {
        LOG(ERROR) << "PhysicsAttributesManager::createAttributesTemplate "
                      ":Invalid value in physics scene config -rigid object "
                      "library- array "
                   << i;
        continue;
      }

      std::string absolutePath =
          Cr::Utility::Directory::join(configDirectory, paths[i].GetString());
      // load all object templates available as configs in absolutePath
      objectAttributesMgr_->loadAllConfigsFromPath(absolutePath, true);
    }
  }  // if load rigid object library metadata

}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
