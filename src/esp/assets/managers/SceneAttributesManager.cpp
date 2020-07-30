// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/String.h>

#include "AttributesManagerBase.h"
#include "SceneAttributesManager.h"

#include "esp/assets/Asset.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
namespace esp {
namespace assets {

namespace managers {

const std::map<std::string, esp::assets::AssetType>
    SceneAttributesManager::AssetTypeNamesMap = {
        {"mp3d", AssetType::MP3D_MESH},
        {"navmesh", AssetType::NAVMESH},
        {"ptex", AssetType::FRL_PTEX_MESH},
        {"semantic", AssetType::INSTANCE_MESH},
        {"suncg", AssetType::SUNCG_SCENE},
};

PhysicsSceneAttributes::ptr SceneAttributesManager::createAttributesTemplate(
    const std::string& sceneAttributesHandle,
    bool registerTemplate) {
  PhysicsSceneAttributes::ptr attrs;
  std::string msg;
  std::string strHandle = Cr::Utility::String::lowercase(sceneAttributesHandle);
  bool fileExists = (this->isValidFileName(sceneAttributesHandle));
  if (objectAttributesMgr_->isValidPrimitiveAttributes(sceneAttributesHandle)) {
    // if sceneAttributesHandle == some existing primitive attributes, then
    // this is a primitive-based scene (i.e. a plane) we are building
    attrs = createPrimBasedAttributesTemplate(sceneAttributesHandle,
                                              registerTemplate);
    msg = "Primitive Asset (" + sceneAttributesHandle + ") Based";

  } else if (fileExists) {
    if ((strHandle.find(".json") != std::string::npos) && fileExists) {
      // check if sceneAttributesHandle corresponds to an actual, existing json
      // scene file descriptor.
      attrs = createFileBasedAttributesTemplate(sceneAttributesHandle,
                                                registerTemplate);
      msg = "JSON File (" + sceneAttributesHandle + ") Based";
    } else {
      // if name is not json file descriptor but still appropriate file
      attrs = createBackCompatAttributesTemplate(sceneAttributesHandle,
                                                 registerTemplate);
      msg = "File (" + sceneAttributesHandle + ") Based";
    }

  } else {
    // if name is not file descriptor, return default attributes.
    attrs = createDefaultAttributesTemplate(sceneAttributesHandle,
                                            registerTemplate);
    msg = "New default";
  }

  if (nullptr != attrs) {
    LOG(INFO) << msg << " scene attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;

}  // SceneAttributesManager::createAttributesTemplate

int SceneAttributesManager::registerAttributesTemplateFinalize(
    PhysicsSceneAttributes::ptr sceneAttributesTemplate,
    const std::string& sceneAttributesHandle) {
  if (sceneAttributesTemplate->getRenderAssetHandle() == "") {
    LOG(ERROR)
        << "SceneAttributesManager::registerAttributesTemplateFinalize : "
           "Attributes template named"
        << sceneAttributesHandle
        << "does not have a valid render asset handle specified. Aborting.";
    return ID_UNDEFINED;
  }

  // Handles for rendering and collision assets
  std::string renderAssetHandle =
      sceneAttributesTemplate->getRenderAssetHandle();
  std::string collisionAssetHandle =
      sceneAttributesTemplate->getCollisionAssetHandle();

  // verify these represent legitimate assets
  if (objectAttributesMgr_->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    sceneAttributesTemplate->setRenderAssetIsPrimitive(true);
  } else if (this->isValidFileName(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file system
    // - if so then setRenderAssetIsPrimitive to false and set map of IDs->Names
    // to physicsFileObjTmpltLibByID_ - verify file  exists
    sceneAttributesTemplate->setRenderAssetIsPrimitive(false);
  } else {
    // If renderAssetHandle is not valid file name needs to  fail
    LOG(ERROR)
        << "SceneAttributesManager::registerAttributesTemplateFinalize "
           ": Render asset template handle : "
        << renderAssetHandle << " specified in scene template with handle : "
        << sceneAttributesHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Aborting. ";
    return ID_UNDEFINED;
  }

  if (objectAttributesMgr_->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(true);
  } else if (this->isValidFileName(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    // Else, means no collision data specified, use specified render data
    LOG(INFO)
        << "SceneAttributesManager::registerAttributesTemplateFinalize "
           ": Collision asset template handle : "
        << collisionAssetHandle << " specified in scene template with handle : "
        << sceneAttributesHandle
        << " does not correspond to any existing file or primitive render "
           "asset.  Overriding with given render asset handle : "
        << renderAssetHandle << ". ";

    sceneAttributesTemplate->setCollisionAssetHandle(renderAssetHandle);
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(
        sceneAttributesTemplate->getRenderAssetIsPrimitive());
  }
  // Clear dirty flag from when asset handles are changed
  sceneAttributesTemplate->setIsClean();

  // adds template to library, and returns either the ID of the existing
  // template referenced by sceneAttributesHandle, or the next available ID
  // if not found.
  int sceneTemplateID = this->addTemplateToLibrary(sceneAttributesTemplate,
                                                   sceneAttributesHandle);
  return sceneTemplateID;
}  // SceneAttributesManager::registerAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createDefaultAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scene
  PhysicsSceneAttributes::ptr sceneAttributesTemplate =
      initNewAttribsInternal(PhysicsSceneAttributes::create(sceneFilename));

  sceneAttributesTemplate->setRenderAssetHandle(sceneFilename);
  sceneAttributesTemplate->setCollisionAssetHandle(sceneFilename);

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributesTemplate,
                                                  sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return sceneAttributesTemplate;
}  // SceneAttributesManager::createDefaultAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAssetHandle,
    bool registerTemplate) {
  // verify that a primitive asset with the given handle exists
  if (!objectAttributesMgr_->isValidPrimitiveAttributes(primAssetHandle)) {
    LOG(ERROR)
        << "SceneAttributesManager::createPrimBasedAttributesTemplate : No "
           "primitive with handle '"
        << primAssetHandle
        << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
  }

  // construct a sceneAttributes
  auto sceneAttributes =
      initNewAttribsInternal(PhysicsSceneAttributes::create(primAssetHandle));
  // set margin to be 0
  sceneAttributes->setMargin(0.0);
  // make smaller as default size - prims are approx meter in size
  sceneAttributes->setScale({0.1, 0.1, 0.1});

  // set render mesh handle
  sceneAttributes->setRenderAssetHandle(primAssetHandle);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  sceneAttributes->setCollisionAssetHandle(primAssetHandle);
  sceneAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  if (nullptr != sceneAttributes && registerTemplate) {
    int attrID =
        this->registerAttributesTemplate(sceneAttributes, primAssetHandle);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return sceneAttributes;
}  // SceneAttributesManager::createPrimBasedAttributesTemplate

PhysicsSceneAttributes::ptr
SceneAttributesManager::createBackCompatAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scene
  PhysicsSceneAttributes::ptr sceneAttributes =
      initNewAttribsInternal(PhysicsSceneAttributes::create(sceneFilename));

  if (registerTemplate) {
    int attrID =
        this->registerAttributesTemplate(sceneAttributes, sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }

  return sceneAttributes;
}  // SceneAttributesManager::createBackCompatAttributesTemplate

PhysicsSceneAttributes::ptr SceneAttributesManager::initNewAttribsInternal(
    PhysicsSceneAttributes::ptr newAttributes) {
  this->setFileDirectoryFromHandle(newAttributes);
  std::string sceneFilename = newAttributes->getHandle();

  // set defaults that config files or other constructive processes might
  // override
  newAttributes->setRenderAssetHandle(sceneFilename);
  newAttributes->setCollisionAssetHandle(sceneFilename);
  newAttributes->setUseMeshCollision(true);

  // set defaults for navmesh default handles and semantic mesh default handles
  std::string navmeshFilename = io::changeExtension(sceneFilename, ".navmesh");
  if (cfgFilepaths_.count("navmesh")) {
    navmeshFilename = cfgFilepaths_.at("navmesh");
  }
  if (Corrade::Utility::Directory::exists(navmeshFilename)) {
    newAttributes->setNavmeshAssetHandle(navmeshFilename);
  }

  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (cfgFilepaths_.count("house")) {
    houseFilename = cfgFilepaths_.at("house");
  }
  if (!Corrade::Utility::Directory::exists(houseFilename)) {
    houseFilename = io::changeExtension(sceneFilename, ".scn");
  }
  if (Corrade::Utility::Directory::exists(houseFilename)) {
    const std::string semanticMeshFilename =
        io::removeExtension(houseFilename) + "_semantic.ply";
    if (Corrade::Utility::Directory::exists(houseFilename)) {
      newAttributes->setSemanticAssetHandle(semanticMeshFilename);
    }
  }

  using Corrade::Utility::String::endsWith;
  // set default origin and orientation values based on file name
  // from AssetInfo::fromPath
  newAttributes->setOrientUp({0, 1, 0});
  newAttributes->setOrientFront({0, 0, -1});
  if (endsWith(sceneFilename, "_semantic.ply")) {
    newAttributes->setRenderAssetType(
        static_cast<int>(AssetType::INSTANCE_MESH));
  } else if (endsWith(sceneFilename, "mesh.ply")) {
    newAttributes->setRenderAssetType(
        static_cast<int>(AssetType::FRL_PTEX_MESH));
    newAttributes->setOrientUp({0, 0, 1});
    newAttributes->setOrientFront({0, 1, 0});
  } else if (endsWith(sceneFilename, "house.json")) {
    newAttributes->setRenderAssetType(static_cast<int>(AssetType::SUNCG_SCENE));
  } else if (endsWith(sceneFilename, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    newAttributes->setRenderAssetType(static_cast<int>(AssetType::MP3D_MESH));
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    newAttributes->setOrientUp({0, 0, 1});
    newAttributes->setOrientFront({0, 1, 0});
  }
  // set default physical quantities specified in physics manager attributes
  if (physicsAttributesManager_->getTemplateLibHasHandle(
          physicsManagerAttributesHandle_)) {
    auto physMgrAttributes = physicsAttributesManager_->getTemplateByHandle(
        physicsManagerAttributesHandle_);
    newAttributes->setGravity(physMgrAttributes->getGravity());
    newAttributes->setFrictionCoefficient(
        physMgrAttributes->getFrictionCoefficient());
    newAttributes->setRestitutionCoefficient(
        physMgrAttributes->getRestitutionCoefficient());
  }
  return newAttributes;
}  // SceneAttributesManager::initNewAttribsInternal

PhysicsSceneAttributes::ptr
SceneAttributesManager::createFileBasedAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Load the scene config JSON here
  io::JsonDocument jsonConfig;
  bool success = this->verifyLoadJson(sceneFilename, jsonConfig);
  if (!success) {
    LOG(ERROR) << " Aborting "
                  "SceneAttributesManager::createBackCompatAttributesTemplate.";
    return nullptr;
  }

  // construct a PhysicsSceneAttributes and populate with any
  // AbstractPhysicsAttributes fields found in json.
  auto sceneAttributes =
      this->createPhysicsAttributesFromJson<PhysicsSceneAttributes>(
          sceneFilename, jsonConfig);

  // directory location where scene files are found
  std::string sceneLocFileDir = sceneAttributes->getFileDirectory();

  // now parse scene-specific fields
  // load scene specific gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      std::bind(&PhysicsSceneAttributes::setGravity, sceneAttributes, _1));

  // load scene specific origin
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "origin",
      std::bind(&PhysicsSceneAttributes::setOrigin, sceneAttributes, _1));

  // load scene specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "up",
      std::bind(&PhysicsSceneAttributes::setOrientUp, sceneAttributes, _1));

  // load scene specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "front",
      std::bind(&PhysicsSceneAttributes::setOrientFront, sceneAttributes, _1));

  // populate specified semantic file name if specified in json
  std::string semanticFName = "";
  std::string navmeshFName = "";
  if (io::jsonIntoVal<std::string>(jsonConfig, "semantic mesh",
                                   semanticFName)) {
    semanticFName =
        Cr::Utility::Directory::join(sceneLocFileDir, semanticFName);
  }
  if (semanticFName != "") {
    sceneAttributes->setSemanticAssetHandle(semanticFName);
  }

  if (io::jsonIntoVal<std::string>(jsonConfig, "nav mesh", navmeshFName)) {
    navmeshFName = Cr::Utility::Directory::join(sceneLocFileDir, navmeshFName);
  }
  if (navmeshFName != "") {
    sceneAttributes->setNavmeshAssetHandle(navmeshFName);
  }

  // populate mesh types if present
  int val = convertJsonStringToAssetType(jsonConfig, "render mesh type");
  if (val != -1) {  // if not found do not override current value
    sceneAttributes->setRenderAssetType(val);
  }
  val = convertJsonStringToAssetType(jsonConfig, "collision mesh type");
  if (val != -1) {  // if not found do not override current value
    sceneAttributes->setCollisionAssetType(val);
  }
  val = convertJsonStringToAssetType(jsonConfig, "semantic mesh type type");
  if (val != -1) {  // if not found do not override current value
    sceneAttributes->setSemanticAssetType(val);
  }

  if (registerTemplate) {
    int attrID =
        this->registerAttributesTemplate(sceneAttributes, sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }

  return sceneAttributes;
}  // SceneAttributesManager::createBackCompatAttributesTemplate

int SceneAttributesManager::convertJsonStringToAssetType(
    io::JsonDocument& jsonDoc,
    const char* jsonTag) {
  std::string tmpVal = "";
  io::jsonIntoVal<std::string>(jsonDoc, jsonTag, tmpVal);
  if (tmpVal.length() != 0) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    if (AssetTypeNamesMap.count(tmpVal)) {
      return static_cast<int>(AssetTypeNamesMap.at(tmpVal));
    } else {
      return static_cast<int>(AssetType::UNKNOWN);
    }
  }
  // -1 i
  return -1;

}  // SceneAttributesManager::convertJsonStringToAssetType

}  // namespace managers
}  // namespace assets
}  // namespace esp
