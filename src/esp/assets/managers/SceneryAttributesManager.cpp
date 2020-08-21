// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/String.h>

#include "AttributesManagerBase.h"
#include "SceneryAttributesManager.h"

#include "esp/assets/Asset.h"
#include "esp/assets/ResourceManager.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;
namespace esp {
namespace assets {

namespace managers {

SceneryAttributesManager::SceneryAttributesManager(
    assets::ResourceManager& resourceManager,
    ObjectAttributesManager::ptr objectAttributesMgr,
    PhysicsAttributesManager::ptr physicsAttributesManager)
    : AttributesManager<PhysicsSceneryAttributes::ptr>::AttributesManager(
          resourceManager,
          "Physical Scene"),
      objectAttributesMgr_(objectAttributesMgr),
      physicsAttributesManager_(physicsAttributesManager),
      cfgLightSetup_(assets::ResourceManager::NO_LIGHT_KEY) {
  buildCtorFuncPtrMaps();
}

PhysicsSceneryAttributes::ptr
SceneryAttributesManager::createAttributesTemplate(
    const std::string& sceneryAttributesHandle,
    bool registerTemplate) {
  PhysicsSceneryAttributes::ptr attrs;
  std::string msg;
  std::string strHandle =
      Cr::Utility::String::lowercase(sceneryAttributesHandle);
  bool fileExists = (this->isValidFileName(sceneryAttributesHandle));
  if (objectAttributesMgr_->isValidPrimitiveAttributes(
          sceneryAttributesHandle)) {
    // if sceneryAttributesHandle == some existing primitive attributes, then
    // this is a primitive-based scenery (i.e. a plane) we are building
    attrs = createPrimBasedAttributesTemplate(sceneryAttributesHandle,
                                              registerTemplate);
    msg = "Primitive Asset (" + sceneryAttributesHandle + ") Based";

  } else if (fileExists) {
    if ((strHandle.find("scene_config.json") != std::string::npos) &&
        fileExists) {
      // check if sceneryAttributesHandle corresponds to an actual, existing
      // json scenery file descriptor.
      attrs = createFileBasedAttributesTemplate(sceneryAttributesHandle,
                                                registerTemplate);
      msg = "JSON File (" + sceneryAttributesHandle + ") Based";
    } else {
      // if name is not json file descriptor but still appropriate file
      attrs = createBackCompatAttributesTemplate(sceneryAttributesHandle,
                                                 registerTemplate);
      msg = "File (" + sceneryAttributesHandle + ") Based";
    }

  } else {
    // if name is not file descriptor, return default attributes.
    attrs = createDefaultAttributesTemplate(sceneryAttributesHandle,
                                            registerTemplate);
    msg = "New default";
  }

  if (nullptr != attrs) {
    LOG(INFO) << msg << " scenery attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;

}  // SceneryAttributesManager::createAttributesTemplate

int SceneryAttributesManager::registerAttributesTemplateFinalize(
    PhysicsSceneryAttributes::ptr sceneAttributesTemplate,
    const std::string& sceneryAttributesHandle) {
  if (sceneAttributesTemplate->getRenderAssetHandle() == "") {
    LOG(ERROR)
        << "SceneryAttributesManager::registerAttributesTemplateFinalize : "
           "Attributes template named"
        << sceneryAttributesHandle
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
  } else if (std::string::npos != sceneryAttributesHandle.find("NONE")) {
    // Render asset handle will be NONE as well - force type to be unknown
    sceneAttributesTemplate->setRenderAssetType(
        static_cast<int>(AssetType::UNKNOWN));
    sceneAttributesTemplate->setRenderAssetIsPrimitive(false);
  } else {
    // If renderAssetHandle is not valid file name needs to  fail
    LOG(ERROR)
        << "SceneryAttributesManager::registerAttributesTemplateFinalize "
           ": Render asset template handle : "
        << renderAssetHandle << " specified in scenery template with handle : "
        << sceneryAttributesHandle
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
  } else if (std::string::npos != sceneryAttributesHandle.find("NONE")) {
    // Render asset handle will be NONE as well - force type to be unknown
    sceneAttributesTemplate->setCollisionAssetType(
        static_cast<int>(AssetType::UNKNOWN));
    sceneAttributesTemplate->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    // Else, means no collision data specified, use specified render data
    LOG(INFO)
        << "SceneryAttributesManager::registerAttributesTemplateFinalize "
           ": Collision asset template handle : "
        << collisionAssetHandle
        << " specified in scenery template with handle : "
        << sceneryAttributesHandle
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
  // template referenced by sceneryAttributesHandle, or the next available ID
  // if not found.
  int sceneTemplateID = this->addTemplateToLibrary(sceneAttributesTemplate,
                                                   sceneryAttributesHandle);
  return sceneTemplateID;
}  // SceneryAttributesManager::registerAttributesTemplate

PhysicsSceneryAttributes::ptr
SceneryAttributesManager::createDefaultAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scenery
  PhysicsSceneryAttributes::ptr sceneAttributesTemplate =
      initNewAttribsInternal(PhysicsSceneryAttributes::create(sceneFilename));

  if (registerTemplate) {
    int attrID = this->registerAttributesTemplate(sceneAttributesTemplate,
                                                  sceneFilename);
    if (attrID == ID_UNDEFINED) {
      // some error occurred
      return nullptr;
    }
  }
  return sceneAttributesTemplate;
}  // SceneryAttributesManager::createDefaultAttributesTemplate

PhysicsSceneryAttributes::ptr
SceneryAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAssetHandle,
    bool registerTemplate) {
  // verify that a primitive asset with the given handle exists
  if (!objectAttributesMgr_->isValidPrimitiveAttributes(primAssetHandle)) {
    LOG(ERROR)
        << "SceneryAttributesManager::createPrimBasedAttributesTemplate : No "
           "primitive with handle '"
        << primAssetHandle
        << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
  }

  // construct a sceneryAttributes
  auto sceneryAttributes =
      initNewAttribsInternal(PhysicsSceneryAttributes::create(primAssetHandle));
  // set margin to be 0
  sceneryAttributes->setMargin(0.0);

  // set render mesh handle
  int primType = static_cast<int>(AssetType::PRIMITIVE);
  sceneryAttributes->setRenderAssetType(primType);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  sceneryAttributes->setCollisionAssetType(primType);
  sceneryAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  return this->postCreateRegister(sceneryAttributes, registerTemplate);
}  // SceneryAttributesManager::createPrimBasedAttributesTemplate

PhysicsSceneryAttributes::ptr
SceneryAttributesManager::createBackCompatAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Attributes descriptor for scenery
  PhysicsSceneryAttributes::ptr sceneryAttributes =
      initNewAttribsInternal(PhysicsSceneryAttributes::create(sceneFilename));

  return this->postCreateRegister(sceneryAttributes, registerTemplate);
}  // SceneryAttributesManager::createBackCompatAttributesTemplate

PhysicsSceneryAttributes::ptr SceneryAttributesManager::initNewAttribsInternal(
    PhysicsSceneryAttributes::ptr newAttributes) {
  this->setFileDirectoryFromHandle(newAttributes);

  std::string sceneFilename = newAttributes->getHandle();

  // set defaults that config files or other constructive processes might
  // override
  newAttributes->setRenderAssetHandle(sceneFilename);
  newAttributes->setCollisionAssetHandle(sceneFilename);
  newAttributes->setUseMeshCollision(true);

  // set defaults from SimulatorConfig values; these can also be overridden by
  // json, for example.
  newAttributes->setLightSetup(cfgLightSetup_);
  newAttributes->setRequiresLighting(cfgLightSetup_ !=
                                     assets::ResourceManager::NO_LIGHT_KEY);
  // set value from config so not necessary to be passed as argument
  newAttributes->setFrustrumCulling(cfgFrustrumCulling_);

  // set defaults for navmesh default handles and semantic mesh default handles
  std::string navmeshFilename = io::changeExtension(sceneFilename, ".navmesh");
  if (cfgFilepaths_.count("navmesh")) {
    navmeshFilename = cfgFilepaths_.at("navmesh");
  }
  if (Corrade::Utility::Directory::exists(navmeshFilename)) {
    newAttributes->setNavmeshAssetHandle(navmeshFilename);
  }
  // Build default semantic descriptor file name
  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (cfgFilepaths_.count("house")) {
    houseFilename = cfgFilepaths_.at("house");
  }
  if (!Corrade::Utility::Directory::exists(houseFilename)) {
    houseFilename = io::changeExtension(sceneFilename, ".scn");
  }
  newAttributes->setHouseFilename(houseFilename);
  // Build default semantic mesh file name
  const std::string semanticMeshFilename =
      io::removeExtension(houseFilename) + "_semantic.ply";
  newAttributes->setSemanticAssetHandle(semanticMeshFilename);

  // set default origin and orientation values based on file name
  // from AssetInfo::fromPath
  // set defaults for passed render asset handles
  setDefaultFileNameBasedAttributes(
      newAttributes, true, newAttributes->getRenderAssetHandle(),
      std::bind(&AbstractPhysicsAttributes::setRenderAssetType, newAttributes,
                _1));
  // set defaults for passed collision asset handles
  setDefaultFileNameBasedAttributes(
      newAttributes, false, newAttributes->getCollisionAssetHandle(),
      std::bind(&AbstractPhysicsAttributes::setCollisionAssetType,
                newAttributes, _1));

  // set defaults for passed semantic asset handles
  setDefaultFileNameBasedAttributes(
      newAttributes, false, newAttributes->getSemanticAssetHandle(),
      std::bind(&PhysicsSceneryAttributes::setSemanticAssetType, newAttributes,
                _1));

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
}  // SceneryAttributesManager::initNewAttribsInternal

void SceneryAttributesManager::setDefaultFileNameBasedAttributes(
    PhysicsSceneryAttributes::ptr attributes,
    bool setFrame,
    const std::string& fileName,
    std::function<void(int)> meshTypeSetter) {
  // TODO : support future mesh-name specific type setting?
  using Corrade::Utility::String::endsWith;

  Magnum::Vector3 up, up1{0, 1, 0}, up2{0, 0, 1};
  Magnum::Vector3 fwd, fwd1{0, 0, -1}, fwd2{0, 1, 0};

  // set default origin and orientation values based on file name
  // from AssetInfo::fromPath
  up = up1;
  fwd = fwd1;
  if (endsWith(fileName, "_semantic.ply")) {
    meshTypeSetter(static_cast<int>(AssetType::INSTANCE_MESH));
  } else if (endsWith(fileName, "mesh.ply")) {
    meshTypeSetter(static_cast<int>(AssetType::FRL_PTEX_MESH));
    up = up2;
    fwd = fwd2;
  } else if (endsWith(fileName, "house.json")) {
    meshTypeSetter(static_cast<int>(AssetType::SUNCG_SCENE));
  } else if (endsWith(fileName, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    meshTypeSetter(static_cast<int>(AssetType::MP3D_MESH));
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    up = up2;
    fwd = fwd2;
  } else {
    meshTypeSetter(static_cast<int>(AssetType::UNKNOWN));
  }
  if (setFrame) {
    attributes->setOrientUp(up);
    attributes->setOrientFront(fwd);
  }
}  // SceneryAttributesManager::setDefaultFileNameBasedAttributes

PhysicsSceneryAttributes::ptr
SceneryAttributesManager::createFileBasedAttributesTemplate(
    const std::string& sceneFilename,
    bool registerTemplate) {
  // Load the scenery config JSON here
  io::JsonDocument jsonConfig;
  bool success = this->verifyLoadJson(sceneFilename, jsonConfig);
  if (!success) {
    LOG(ERROR)
        << "SceneryAttributesManager::createFileBasedAttributesTemplate : "
           "Failure reading json "
        << sceneFilename << ". Aborting.";
    return nullptr;
  }

  // construct a PhysicsSceneryAttributes and populate with any
  // AbstractPhysicsAttributes fields found in json.
  auto sceneryAttributes =
      this->createPhysicsAttributesFromJson<PhysicsSceneryAttributes>(
          sceneFilename, jsonConfig);

  // directory location where scenery files are found
  std::string sceneLocFileDir = sceneryAttributes->getFileDirectory();

  // now parse scenery-specific fields
  // load scenery specific gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity",
      std::bind(&PhysicsSceneryAttributes::setGravity, sceneryAttributes, _1));

  // load scenery specific origin
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "origin",
      std::bind(&PhysicsSceneryAttributes::setOrigin, sceneryAttributes, _1));

  // populate specified semantic file name if specified in json - defaults
  // are overridden only if specified in json.

  std::string navmeshFName = "";
  std::string houseFName = "";
  std::string lightSetup = "";

  // populate semantic mesh type if present
  std::string semanticFName = sceneryAttributes->getSemanticAssetHandle();
  if (this->setJSONAssetHandleAndType(
          sceneryAttributes, jsonConfig, "semantic mesh type", "semantic mesh",
          semanticFName,
          std::bind(&PhysicsSceneryAttributes::setSemanticAssetType,
                    sceneryAttributes, _1))) {
    // if "semantic mesh" is specified in scenery json to non-empty value, set
    // value (override default).
    sceneryAttributes->setSemanticAssetHandle(semanticFName);
    // TODO eventually remove this, but currently semantic mesh must be instance
    sceneryAttributes->setSemanticAssetType(
        static_cast<int>(AssetType::INSTANCE_MESH));
  }

  if (io::jsonIntoVal<std::string>(jsonConfig, "nav mesh", navmeshFName)) {
    navmeshFName = Cr::Utility::Directory::join(sceneLocFileDir, navmeshFName);
    // if "nav mesh" is specified in scenery json set value (override default).
    sceneryAttributes->setNavmeshAssetHandle(navmeshFName);
  }

  if (io::jsonIntoVal<std::string>(jsonConfig, "house filename", houseFName)) {
    houseFName = Cr::Utility::Directory::join(sceneLocFileDir, houseFName);
    // if "house filename" is specified in scenery json, set value (override
    // default).
    sceneryAttributes->setHouseFilename(houseFName);
  }

  if (io::jsonIntoVal<std::string>(jsonConfig, "lighting setup", lightSetup)) {
    // if lighting is specified in scenery json to non-empty value, set value
    // (override default).
    sceneryAttributes->setLightSetup(lightSetup);
  }

  // load the rigid object library metadata (no physics init yet...)
  if (jsonConfig.HasMember("rigid object paths") &&
      jsonConfig["rigid object paths"].IsArray()) {
    std::string configDirectory =
        sceneFilename.substr(0, sceneFilename.find_last_of("/"));

    const auto& paths = jsonConfig["rigid object paths"];
    for (rapidjson::SizeType i = 0; i < paths.Size(); i++) {
      if (!paths[i].IsString()) {
        LOG(ERROR)
            << "SceneryAttributesManager::createAttributesTemplate "
               ":Invalid value in scenery config 'rigid object paths'- array "
            << i;
        continue;
      }

      std::string absolutePath =
          Cr::Utility::Directory::join(configDirectory, paths[i].GetString());
      // load all object templates available as configs in absolutePath
      objectAttributesMgr_->loadObjectConfigs(absolutePath, true);
    }
  }  // if load rigid object library metadata

  return this->postCreateRegister(sceneryAttributes, registerTemplate);
}  // SceneryAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace assets
}  // namespace esp
