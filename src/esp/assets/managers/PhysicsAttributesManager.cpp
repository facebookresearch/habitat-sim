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

const PhysicsManagerAttributes::ptr
PhysicsAttributesManager::createAttributesTemplate(
    const std::string& physicsFilename,
    bool registerTemplate) {
  CORRADE_ASSERT(
      Cr::Utility::Directory::exists(physicsFilename),
      "PhysicsAttributesManager::createAttributesTemplate : Specified Filename "
      ":" << physicsFilename
          << "cannot be found.  Aborting",
      nullptr);

  // Load the global scene config JSON here
  io::JsonDocument scenePhysicsConfig = io::parseJsonFile(physicsFilename);
  // In-memory representation of scene meta data
  PhysicsManagerAttributes::ptr physicsManagerAttributes =
      PhysicsManagerAttributes::create(physicsFilename);

  // load the simulator preference
  // default is "none" simulator
  if (scenePhysicsConfig.HasMember("physics simulator")) {
    if (scenePhysicsConfig["physics simulator"].IsString()) {
      physicsManagerAttributes->setSimulator(
          scenePhysicsConfig["physics simulator"].GetString());
    }
  }

  // load the physics timestep
  if (scenePhysicsConfig.HasMember("timestep")) {
    if (scenePhysicsConfig["timestep"].IsNumber()) {
      physicsManagerAttributes->setTimestep(
          scenePhysicsConfig["timestep"].GetDouble());
    }
  }

  if (scenePhysicsConfig.HasMember("friction coefficient") &&
      scenePhysicsConfig["friction coefficient"].IsNumber()) {
    physicsManagerAttributes->setFrictionCoefficient(
        scenePhysicsConfig["friction coefficient"].GetDouble());
  } else {
    LOG(ERROR) << " Invalid value in scene config - friction coefficient";
  }

  if (scenePhysicsConfig.HasMember("restitution coefficient") &&
      scenePhysicsConfig["restitution coefficient"].IsNumber()) {
    physicsManagerAttributes->setRestitutionCoefficient(
        scenePhysicsConfig["restitution coefficient"].GetDouble());
  } else {
    LOG(ERROR) << " Invalid value in scene config - restitution coefficient";
  }

  // load gravity
  if (scenePhysicsConfig.HasMember("gravity")) {
    if (scenePhysicsConfig["gravity"].IsArray()) {
      Magnum::Vector3 grav;
      for (rapidjson::SizeType i = 0; i < scenePhysicsConfig["gravity"].Size();
           i++) {
        if (!scenePhysicsConfig["gravity"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << "Invalid value in physics gravity array";
          break;
        } else {
          grav[i] = scenePhysicsConfig["gravity"][i].GetDouble();
        }
      }
      physicsManagerAttributes->setGravity(grav);
    }
  }

  // load the rigid object library metadata (no physics init yet...)
  if (scenePhysicsConfig.HasMember("rigid object paths") &&
      scenePhysicsConfig["rigid object paths"].IsArray()) {
    std::string configDirectory =
        physicsFilename.substr(0, physicsFilename.find_last_of("/"));

    const auto& paths = scenePhysicsConfig["rigid object paths"];
    for (rapidjson::SizeType i = 0; i < paths.Size(); i++) {
      if (!paths[i].IsString()) {
        LOG(ERROR) << "Invalid value in physics scene config -rigid object "
                      "library- array "
                   << i;
        continue;
      }

      std::string absolutePath =
          Cr::Utility::Directory::join(configDirectory, paths[i].GetString());
      std::vector<std::string> validConfigPaths =
          buildObjectConfigPaths(absolutePath);
      for (auto& path : validConfigPaths) {
        physicsManagerAttributes->addStringToGroup("objectLibraryPaths", path);
      }
    }
  }  // if load rigid object library metadata

  if (registerTemplate) {
    registerAttributesTemplate(physicsManagerAttributes, physicsFilename);
  }

  return physicsManagerAttributes;
}  // PhysicsAttributesManager::createAttributesTemplate

std::vector<std::string> PhysicsAttributesManager::buildObjectConfigPaths(
    const std::string& path) {
  std::vector<std::string> paths;

  namespace Directory = Cr::Utility::Directory;
  std::string objPhysPropertiesFilename = path;
  if (!Corrade::Utility::String::endsWith(objPhysPropertiesFilename,
                                          ".phys_properties.json")) {
    objPhysPropertiesFilename = path + ".phys_properties.json";
  }
  const bool dirExists = Directory::isDirectory(path);
  const bool fileExists = Directory::exists(objPhysPropertiesFilename);

  if (!dirExists && !fileExists) {
    LOG(WARNING) << "Cannot find " << path << " or "
                 << objPhysPropertiesFilename << ". Aborting parse.";
    return paths;
  }

  if (fileExists) {
    paths.push_back(objPhysPropertiesFilename);
  }

  if (dirExists) {
    LOG(INFO) << "Parsing object library directory: " + path;
    for (auto& file : Directory::list(path, Directory::Flag::SortAscending)) {
      std::string absoluteSubfilePath = Directory::join(path, file);
      if (Cr::Utility::String::endsWith(absoluteSubfilePath,
                                        ".phys_properties.json")) {
        paths.push_back(absoluteSubfilePath);
      }
    }
  }

  return paths;
}  // PhysicsAttributesManager::buildObjectConfigPaths

}  // namespace managers
}  // namespace assets
}  // namespace esp