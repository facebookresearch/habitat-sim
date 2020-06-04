// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributesManager.h"
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
PhysicsObjectAttributes::ptr ObjectAttributesManager::createAttributesTemplate(
    const std::string& attributesTemplateHandle,
    bool registerTemplate) {
  PhysicsObjectAttributes::ptr objAttributes;
  if (assetAttributesMgr_->getTemplateLibHasHandle(attributesTemplateHandle)) {
    objAttributes = buildPrimBasedPhysObjTemplate(attributesTemplateHandle);
  } else {
    objAttributes = parseAndLoadPhysObjTemplate(attributesTemplateHandle);
  }
  if (registerTemplate) {
    registerAttributesTemplate(objAttributes, attributesTemplateHandle);
  }
  return objAttributes;
}  // ObjectAttributesManager::createAttributesTemplate

int ObjectAttributesManager::registerAttributesTemplate(
    PhysicsObjectAttributes::ptr objectTemplate,
    const std::string& objectTemplateHandle) {
  CORRADE_ASSERT(
      objectTemplate->getRenderAssetHandle() != "",
      "ResourceManager::registerObjectTemplate : Attributes template named"
          << objectTemplateHandle
          << "does not have a valid render asset handle specified. Aborting.",
      ID_UNDEFINED);
  // In case not constructed with origin handle as parameter
  objectTemplate->setOriginHandle(objectTemplateHandle);
  std::map<int, std::string>* mapToUse;
  // Handles for rendering and collision assets
  std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle = objectTemplate->getCollisionAssetHandle();

  if (templateLibrary_.count(renderAssetHandle) > 0) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map to
    // physicsSynthObjTmpltLibByID_
    objectTemplate->setRenderAssetIsPrimitive(true);
    mapToUse = &physicsSynthObjTmpltLibByID_;
  } else if (Corrade::Utility::Directory::exists(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and map set to
    // physicsFileObjTmpltLibByID_ - use primitiveImporter to check if file
    // exists
    objectTemplate->setRenderAssetIsPrimitive(false);
    mapToUse = &physicsFileObjTmpltLibByID_;
  } else {
    // If renderAssetHandle is neither valid file name nor existing primitive
    // attributes template hande, fail
    LOG(ERROR) << "ResourceManager::registerObjectTemplate : Render asset "
                  "template handle : "
               << renderAssetHandle
               << " specified in object template with handle : "
               << objectTemplateHandle
               << " does not correspond to existing file or primitive render "
                  "asset.  Aborting. ";
    return ID_UNDEFINED;
  }

  if (templateLibrary_.count(collisionAssetHandle) > 0) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    objectTemplate->setCollisionAssetIsPrimitive(true);
  } else if (Corrade::Utility::Directory::exists(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    objectTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    objectTemplate->setCollisionAssetHandle(renderAssetHandle);
    objectTemplate->setCollisionAssetIsPrimitive(
        objectTemplate->getRenderAssetIsPrimitive());
  }

  // Clear dirty flag from when asset handles are changed
  objectTemplate->setIsClean();

  // Add object template to template library
  int objectTemplateID =
      addTemplateToLibrary(objectTemplate, objectTemplateHandle);

  mapToUse->emplace(objectTemplateID, objectTemplateHandle);

  return objectTemplateID;
}  // ObjectAttributesManager::registerAttributesTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::parseAndLoadPhysObjTemplate(
    const std::string& objPhysConfigFilename) {
  // check for duplicate load
  const bool objTemplateExists =
      templateLibrary_.count(objPhysConfigFilename) > 0;
  if (objTemplateExists) {
    return templateLibrary_.at(objPhysConfigFilename);
  }

  // 1. parse the config file
  io::JsonDocument objPhysicsConfig;
  if (Corrade::Utility::Directory::exists(objPhysConfigFilename)) {
    try {
      objPhysicsConfig = io::parseJsonFile(objPhysConfigFilename);
    } catch (...) {
      LOG(ERROR) << "Failed to parse JSON: " << objPhysConfigFilename
                 << ". Aborting parseAndLoadPhysObjTemplate.";
      return nullptr;
    }
  } else {
    LOG(ERROR) << "File " << objPhysConfigFilename
               << " does not exist. Aborting parseAndLoadPhysObjTemplate.";
    return nullptr;
  }

  // 2. construct a physicsObjectMetaData
  auto physicsObjectAttributes =
      PhysicsObjectAttributes::create(objPhysConfigFilename);

  // NOTE: these paths should be relative to the properties file
  std::string propertiesFileDirectory =
      objPhysConfigFilename.substr(0, objPhysConfigFilename.find_last_of("/"));

  // 3. load physical properties to override defaults (set in
  // PhysicsObjectMetaData.h) load the mass
  if (objPhysicsConfig.HasMember("mass")) {
    if (objPhysicsConfig["mass"].IsNumber()) {
      physicsObjectAttributes->setMass(objPhysicsConfig["mass"].GetDouble());
    }
  }

  // optional set bounding box as collision object
  if (objPhysicsConfig.HasMember("use bounding box for collision")) {
    if (objPhysicsConfig["use bounding box for collision"].IsBool()) {
      physicsObjectAttributes->setBoundingBoxCollisions(
          objPhysicsConfig["use bounding box for collision"].GetBool());
    }
  }

  // load the center of mass (in the local frame of the object)
  // if COM is provided, use it for mesh shift
  if (objPhysicsConfig.HasMember("COM")) {
    if (objPhysicsConfig["COM"].IsArray()) {
      Magnum::Vector3 COM;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["COM"].Size(); i++) {
        if (!objPhysicsConfig["COM"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << " Invalid value in object physics config - COM array";
          break;
        } else {
          COM[i] = objPhysicsConfig["COM"][i].GetDouble();
        }
      }
      physicsObjectAttributes->setCOM(COM);
      // set a flag which we can find later so we don't override the desired
      // COM with BB center.
      physicsObjectAttributes->setBool("COM_provided", true);
    }
  }

  // scaling
  if (objPhysicsConfig.HasMember("scale")) {
    if (objPhysicsConfig["scale"].IsArray()) {
      Magnum::Vector3 scale;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["scale"].Size();
           i++) {
        if (!objPhysicsConfig["scale"][i].IsNumber()) {
          // invalid config
          LOG(ERROR) << " Invalid value in object physics config - scale array";
          break;
        } else {
          scale[i] = objPhysicsConfig["scale"][i].GetDouble();
        }
      }
      physicsObjectAttributes->setScale(scale);
    }
  }

  // load the inertia diagonal
  if (objPhysicsConfig.HasMember("inertia")) {
    if (objPhysicsConfig["inertia"].IsArray()) {
      Magnum::Vector3 inertia;
      for (rapidjson::SizeType i = 0; i < objPhysicsConfig["inertia"].Size();
           i++) {
        if (!objPhysicsConfig["inertia"][i].IsNumber()) {
          // invalid config
          LOG(ERROR)
              << " Invalid value in object physics config - inertia array";
          break;
        } else {
          inertia[i] = objPhysicsConfig["inertia"][i].GetDouble();
        }
      }
      physicsObjectAttributes->setInertia(inertia);
    }
  }

  // load the friction coefficient
  if (objPhysicsConfig.HasMember("friction coefficient")) {
    if (objPhysicsConfig["friction coefficient"].IsNumber()) {
      physicsObjectAttributes->setFrictionCoefficient(
          objPhysicsConfig["friction coefficient"].GetDouble());
    } else {
      LOG(ERROR)
          << " Invalid value in object physics config - friction coefficient";
    }
  }

  // load the restitution coefficient
  if (objPhysicsConfig.HasMember("restitution coefficient")) {
    if (objPhysicsConfig["restitution coefficient"].IsNumber()) {
      physicsObjectAttributes->setRestitutionCoefficient(
          objPhysicsConfig["restitution coefficient"].GetDouble());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - restitution "
                    "coefficient";
    }
  }

  //! Get collision configuration options if specified
  if (objPhysicsConfig.HasMember("join collision meshes")) {
    if (objPhysicsConfig["join collision meshes"].IsBool()) {
      physicsObjectAttributes->setJoinCollisionMeshes(
          objPhysicsConfig["join collision meshes"].GetBool());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - join "
                    "collision meshes";
    }
  }

  // if object will be flat or phong shaded
  if (objPhysicsConfig.HasMember("requires lighting")) {
    if (objPhysicsConfig["requires lighting"].IsBool()) {
      physicsObjectAttributes->setRequiresLighting(
          objPhysicsConfig["requires lighting"].GetBool());
    } else {
      LOG(ERROR)
          << " Invalid value in object physics config - requires lighting";
    }
  }

  // 4. parse render and collision mesh filepaths
  std::string renderMeshFilename = "";
  std::string collisionMeshFilename = "";

  if (objPhysicsConfig.HasMember("render mesh")) {
    if (objPhysicsConfig["render mesh"].IsString()) {
      renderMeshFilename = Cr::Utility::Directory::join(
          propertiesFileDirectory, objPhysicsConfig["render mesh"].GetString());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - render mesh";
    }
  }
  if (objPhysicsConfig.HasMember("collision mesh")) {
    if (objPhysicsConfig["collision mesh"].IsString()) {
      collisionMeshFilename = Cr::Utility::Directory::join(
          propertiesFileDirectory,
          objPhysicsConfig["collision mesh"].GetString());
    } else {
      LOG(ERROR) << " Invalid value in object physics config - collision mesh";
    }
  }

  physicsObjectAttributes->setRenderAssetHandle(renderMeshFilename);
  physicsObjectAttributes->setCollisionAssetHandle(collisionMeshFilename);
  physicsObjectAttributes->setUseMeshCollision(true);

  return physicsObjectAttributes;
}  // parseAndLoadPhysObjTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::buildPrimBasedPhysObjTemplate(
    const std::string& primAssetHandle) {
  // verify that a primitive asset with the given handle exists

  CORRADE_ASSERT(assetAttributesMgr_->getTemplateLibHasHandle(primAssetHandle),
                 " No primitive with handle '"
                     << primAssetHandle
                     << "' exists so cannot build physical object.  Aborting.",
                 nullptr);

  // verify that a template with this asset's name does not exist
  if (templateLibrary_.count(primAssetHandle) > 0) {
    return templateLibrary_.at(primAssetHandle);
  }

  // construct a PhysicsObjectAttributes
  auto physicsObjectAttributes =
      PhysicsObjectAttributes::create(primAssetHandle);
  // set margin to be 0
  physicsObjectAttributes->setMargin(0.0);
  // make smaller as default size - prims are approx meter in size
  physicsObjectAttributes->setScale({0.1, 0.1, 0.1});

  // set render mesh handle
  physicsObjectAttributes->setRenderAssetHandle(primAssetHandle);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  physicsObjectAttributes->setCollisionAssetHandle(primAssetHandle);
  physicsObjectAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a collision
  // primitive mesh needs to be configured and set in MeshMetaData and
  // CollisionMesh
  return physicsObjectAttributes;
}  // ResourceManager::buildPrimBasedPhysObjTemplate

}  // namespace managers
}  // namespace assets
}  // namespace esp