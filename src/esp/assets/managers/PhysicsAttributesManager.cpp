// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
namespace Cr = Corrade;
namespace esp {

namespace assets {

namespace managers {

PhysicsManagerAttributes::ptr
PhysicsAttributesManager::createAttributesTemplate(
    const std::string& physicsFilename,
    bool registerTemplate) {
  PhysicsManagerAttributes::ptr attrs;
  std::string msg;
  if (this->isValidFileName(physicsFilename)) {
    // check if physicsFilename corresponds to an actual file descriptor
    attrs =
        createFileBasedAttributesTemplate(physicsFilename, registerTemplate);
    msg = "File (" + physicsFilename + ") Based";
  } else {
    // if name is not file descriptor, return default attributes.
    attrs = createDefaultAttributesTemplate(physicsFilename, registerTemplate);
    msg = "File (" + physicsFilename + ") not found so new, default";
  }

  if (nullptr != attrs) {
    LOG(INFO) << msg << " physics manager attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // PhysicsAttributesManager::createAttributesTemplate

PhysicsManagerAttributes::ptr
PhysicsAttributesManager::createDefaultAttributesTemplate(
    const std::string& physicsFilename,
    bool registerTemplate) {
  // Attributes descriptor for physics world
  PhysicsManagerAttributes::ptr physicsManagerAttributes =
      PhysicsManagerAttributes::create(physicsFilename);

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(physicsManagerAttributes,
                                                  physicsFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return physicsManagerAttributes;
}  // PhysicsAttributesManager::createDefaultAttributesTemplate

PhysicsManagerAttributes::ptr
PhysicsAttributesManager::createFileBasedAttributesTemplate(
    const std::string& physicsFilename,
    bool registerTemplate) {
  // Attributes descriptor for physics world
  PhysicsManagerAttributes::ptr physicsManagerAttributes =
      PhysicsManagerAttributes::create(physicsFilename);

  // Load the global physics manager config JSON here
  io::JsonDocument jsonConfig;
  bool success = this->verifyLoadJson(physicsFilename, jsonConfig);
  if (!success) {
    LOG(ERROR)
        << " Aborting "
           "PhysicsAttributesManager::createFileBasedAttributesTemplate.";
    return nullptr;
  }

  // load the simulator preference - default is "none" simulator, set in
  // attributes ctor.
  success = io::jsonIntoSetter<std::string>(
      jsonConfig, "physics simulator",
      std::bind(&PhysicsManagerAttributes::setSimulator,
                physicsManagerAttributes, _1));

  // load the physics timestep
  success = io::jsonIntoSetter<double>(
      jsonConfig, "timestep",
      std::bind(&PhysicsManagerAttributes::setTimestep,
                physicsManagerAttributes, _1));

  // load the friction coefficient
  success = io::jsonIntoSetter<double>(
      jsonConfig, "friction coefficient",
      std::bind(&PhysicsManagerAttributes::setFrictionCoefficient,
                physicsManagerAttributes, _1));

  // load the restitution coefficient
  success = io::jsonIntoSetter<double>(
      jsonConfig, "restitution coefficient",
      std::bind(&PhysicsManagerAttributes::setRestitutionCoefficient,
                physicsManagerAttributes, _1));

  // load world gravity
  success = io::jsonIntoArraySetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      std::bind(&PhysicsManagerAttributes::setGravity, physicsManagerAttributes,
                _1));

  // load the rigid object library metadata (no physics init yet...)
  if (jsonConfig.HasMember("rigid object paths") &&
      jsonConfig["rigid object paths"].IsArray()) {
    std::string configDirectory =
        physicsFilename.substr(0, physicsFilename.find_last_of("/"));

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
      objectAttributesMgr_->loadObjectConfigs(absolutePath);
    }
  }  // if load rigid object library metadata

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(physicsManagerAttributes,
                                                  physicsFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }

  return physicsManagerAttributes;
}  // PhysicsAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace assets
}  // namespace esp