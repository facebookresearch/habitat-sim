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
  PhysicsObjectAttributes::ptr attrs;
  std::string msg;
  if (assetAttributesMgr_->getTemplateLibHasHandle(attributesTemplateHandle)) {
    // if attributesTemplateHandle == some existing primitive attributes, then
    // this is a primitive-based object we are building
    attrs = createPrimBasedAttributesTemplate(attributesTemplateHandle,
                                              registerTemplate);
    msg = "Primitive Asset Based";
  } else if (Corrade::Utility::Directory::exists(attributesTemplateHandle)) {
    // if attributesTemplateHandle == some existing file then
    // assume this is a file-based object template we are building.
    attrs = createFileBasedAttributesTemplate(attributesTemplateHandle,
                                              registerTemplate);
    msg = "File Based";
  } else {
    // if neither of these is true, then build an empty template and assign the
    // passed handle to its origin handle and its render asset handle
    attrs = createEmptyAttributesTemplate(attributesTemplateHandle,
                                          registerTemplate);
    msg = "New blank";
  }
  if (nullptr != attrs) {
    LOG(INFO) << msg << " object attributes created "
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;

}  // ObjectAttributesManager::createAttributesTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAttrTemplateHandle,
    bool registerTemplate) {
  PhysicsObjectAttributes::ptr objAttributes =
      buildPrimBasedPhysObjTemplate(primAttrTemplateHandle);

  if (nullptr != objAttributes && registerTemplate) {
    int attrID =
        registerAttributesTemplate(objAttributes, primAttrTemplateHandle);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return objAttributes;
}  // ObjectAttributesManager::createPrimBasedAttributesTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::createFileBasedAttributesTemplate(
    const std::string& filename,
    bool registerTemplate) {
  // this is a file-based object template we are building.
  PhysicsObjectAttributes::ptr objAttributes =
      parseAndLoadPhysObjTemplate(filename);

  if (nullptr != objAttributes && registerTemplate) {
    int attrID = registerAttributesTemplate(objAttributes, filename);
    // some error occurred
    if (attrID == ID_UNDEFINED) {
      return nullptr;
    }
  }
  return objAttributes;
}  // ObjectAttributesManager::createFileBasedAttributesTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::createEmptyAttributesTemplate(
    const std::string& templateName,
    bool registerTemplate) {
  // construct a PhysicsObjectAttributes
  PhysicsObjectAttributes::ptr objAttributes =
      PhysicsObjectAttributes::create(templateName);
  // set render mesh handle as a default
  objAttributes->setRenderAssetHandle(templateName);
  if (registerTemplate) {
    int attrID = registerAttributesTemplate(objAttributes, templateName);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return objAttributes;
}  // ObjectAttributesManager::createEmptyAttributesTemplate

int ObjectAttributesManager::registerAttributesTemplateFinalize(
    PhysicsObjectAttributes::ptr objectTemplate,
    const std::string& objectTemplateHandle) {
  if (objectTemplate->getRenderAssetHandle() == "") {
    LOG(ERROR)
        << "ObjectAttributesManager::registerAttributesTemplate : Attributes "
           "template named"
        << objectTemplateHandle
        << "does not have a valid render asset handle specified. Aborting.";
    return ID_UNDEFINED;
  }

  std::map<int, std::string>* mapToUse;
  // Handles for rendering and collision assets
  std::string renderAssetHandle = objectTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle = objectTemplate->getCollisionAssetHandle();

  if (assetAttributesMgr_->getTemplateLibHasHandle(renderAssetHandle) > 0) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    objectTemplate->setRenderAssetIsPrimitive(true);
    mapToUse = &physicsSynthObjTmpltLibByID_;
  } else if (Corrade::Utility::Directory::exists(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    objectTemplate->setRenderAssetIsPrimitive(false);
    mapToUse = &physicsFileObjTmpltLibByID_;
  } else {
    // If renderAssetHandle is neither valid file name nor existing primitive
    // attributes template hande, fail
    // by here always fail
    LOG(ERROR)
        << "ObjectAttributesManager::registerAttributesTemplate : Render asset "
           "template handle : "
        << renderAssetHandle << " specified in object template with handle : "
        << objectTemplateHandle
        << " does not correspond to existing file or primitive render "
           "asset.  Aborting. ";
    return ID_UNDEFINED;
  }

  if (assetAttributesMgr_->getTemplateLibHasHandle(collisionAssetHandle) > 0) {
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
      this->addTemplateToLibrary(objectTemplate, objectTemplateHandle);

  mapToUse->emplace(objectTemplateID, objectTemplateHandle);

  return objectTemplateID;
}  // ObjectAttributesManager::registerAttributesTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::parseAndLoadPhysObjTemplate(
    const std::string& objPhysConfigFilename) {
  // 1. parse the config file
  io::JsonDocument objPhysicsConfig;
  if (Corrade::Utility::Directory::exists(objPhysConfigFilename)) {
    try {
      objPhysicsConfig = io::parseJsonFile(objPhysConfigFilename);
    } catch (...) {
      // by here always fail

      LOG(ERROR) << "Failed to parse JSON: " << objPhysConfigFilename
                 << ". Aborting parseAndLoadPhysObjTemplate.";
      return nullptr;
    }
  } else {
    // by here always fail
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
      physicsObjectAttributes->setComputeCOMFromShape(false);
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
}  // ObjectAttributesManager::parseAndLoadPhysObjTemplate

PhysicsObjectAttributes::ptr
ObjectAttributesManager::buildPrimBasedPhysObjTemplate(
    const std::string& primAssetHandle) {
  // verify that a primitive asset with the given handle exists
  if (!assetAttributesMgr_->getTemplateLibHasHandle(primAssetHandle)) {
    LOG(ERROR) << "ObjectAttributesManager::buildPrimBasedPhysObjTemplate : No "
                  "primitive with handle '"
               << primAssetHandle
               << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
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
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh
  return physicsObjectAttributes;
}  // ObjectAttributesManager::buildPrimBasedPhysObjTemplate

std::vector<int> ObjectAttributesManager::loadAllFileBasedTemplates(
    const std::vector<std::string>& tmpltFilenames) {
  std::vector<int> resIDs(tmpltFilenames.size(), ID_UNDEFINED);
  for (int i = 0; i < tmpltFilenames.size(); ++i) {
    auto objPhysPropertiesFilename = tmpltFilenames[i];
    LOG(INFO) << "Loading file-based object template: "
              << objPhysPropertiesFilename;
    auto tmplt =
        createFileBasedAttributesTemplate(objPhysPropertiesFilename, true);
    resIDs[i] = tmplt->getID();
  }
  LOG(INFO) << "Loaded file-based object templates: "
            << std::to_string(physicsFileObjTmpltLibByID_.size());
  return resIDs;
}  // ResourceManager::loadAllObjectTemplates

}  // namespace managers
}  // namespace assets
}  // namespace esp