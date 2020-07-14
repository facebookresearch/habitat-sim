// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsAttributesManager.h"
#include "AttributesManagerBase.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/io/io.h"
#include "esp/io/json.h"

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
  if (Cr::Utility::Directory::exists(physicsFilename)) {
    // check if physicsFilename corresponds to an actual file descriptor
    attrs =
        createFileBasedAttributesTemplate(physicsFilename, registerTemplate);
    msg = "File Based";
  } else {
    // if name is not file descriptor, return default attributes.
    attrs = createDefaultAttributesTemplate(physicsFilename, registerTemplate);
    msg = "New default";
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
  io::JsonDocument physicsWorldConfig = io::parseJsonFile(physicsFilename);

  // load the simulator preference
  // default is "none" simulator
  if (physicsWorldConfig.HasMember("physics simulator")) {
    if (physicsWorldConfig["physics simulator"].IsString()) {
      physicsManagerAttributes->setSimulator(
          physicsWorldConfig["physics simulator"].GetString());
    }
  }

  // load the physics timestep
  if (physicsWorldConfig.HasMember("timestep")) {
    if (physicsWorldConfig["timestep"].IsNumber()) {
      physicsManagerAttributes->setTimestep(
          physicsWorldConfig["timestep"].GetDouble());
    }
  }

  if (physicsWorldConfig.HasMember("friction coefficient") &&
      physicsWorldConfig["friction coefficient"].IsNumber()) {
    physicsManagerAttributes->setFrictionCoefficient(
        physicsWorldConfig["friction coefficient"].GetDouble());
  } else {
    LOG(ERROR) << "PhysicsAttributesManager::createAttributesTemplate : "
                  "Invalid value "
                  "in physics manager config - friction coefficient";
  }

  if (physicsWorldConfig.HasMember("restitution coefficient") &&
      physicsWorldConfig["restitution coefficient"].IsNumber()) {
    physicsManagerAttributes->setRestitutionCoefficient(
        physicsWorldConfig["restitution coefficient"].GetDouble());
  } else {
    LOG(ERROR) << "PhysicsAttributesManager::createAttributesTemplate : "
                  "Invalid value "
                  "in physics manager config - restitution coefficient";
  }

  // load gravity
  if (physicsWorldConfig.HasMember("gravity")) {
    if (physicsWorldConfig["gravity"].IsArray()) {
      Magnum::Vector3 grav;
      for (rapidjson::SizeType i = 0; i < physicsWorldConfig["gravity"].Size();
           i++) {
        if (!physicsWorldConfig["gravity"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << "Invalid value in physics gravity array";
          break;
        } else {
          grav[i] = physicsWorldConfig["gravity"][i].GetDouble();
        }
      }
      physicsManagerAttributes->setGravity(grav);
    }
  }

  // load the rigid object library metadata (no physics init yet...)
  if (physicsWorldConfig.HasMember("rigid object paths") &&
      physicsWorldConfig["rigid object paths"].IsArray()) {
    std::string configDirectory =
        physicsFilename.substr(0, physicsFilename.find_last_of("/"));

    const auto& paths = physicsWorldConfig["rigid object paths"];
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