// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/String.h>

#include <utility>

#include "AbstractObjectAttributesManagerBase.h"
#include "StageAttributesManager.h"

#include "esp/assets/Asset.h"
#include "esp/assets/ResourceManager.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
using assets::AssetType;
namespace metadata {

using attributes::AbstractObjectAttributes;
using attributes::StageAttributes;
namespace managers {

StageAttributesManager::StageAttributesManager(
    ObjectAttributesManager::ptr objectAttributesMgr,
    PhysicsAttributesManager::ptr physicsAttributesManager)
    : AbstractObjectAttributesManager<StageAttributes,
                                      core::ManagedObjectAccess::Copy>::
          AbstractObjectAttributesManager("Stage", "stage_config.json"),
      objectAttributesMgr_(std::move(objectAttributesMgr)),
      physicsAttributesManager_(std::move(physicsAttributesManager)),
      cfgLightSetup_(NO_LIGHT_KEY) {
  // build this manager's copy constructor map
  this->copyConstructorMap_["StageAttributes"] =
      &StageAttributesManager::createObjectCopy<attributes::StageAttributes>;
  // create none-type stage attributes and set as undeletable
  // based on default
  auto tmplt = this->postCreateRegister(
      StageAttributesManager::initNewObjectInternal("NONE", false), true);
  std::string tmpltHandle = tmplt->getHandle();
  this->undeletableObjectNames_.insert(tmpltHandle);
}  // StageAttributesManager::ctor

int StageAttributesManager::registerObjectFinalize(
    StageAttributes::ptr stageAttributes,
    const std::string& stageAttributesHandle,
    bool forceRegistration) {
  if (stageAttributes->getRenderAssetHandle() == "") {
    ESP_ERROR()
        << "Attributes template named" << stageAttributesHandle
        << "does not have a valid render asset handle specified. Aborting.";
    return ID_UNDEFINED;
  }

  // Handles for rendering and collision assets
  std::string renderAssetHandle = stageAttributes->getRenderAssetHandle();
  std::string collisionAssetHandle = stageAttributes->getCollisionAssetHandle();

  // verify these represent legitimate assets
  if (StageAttributesManager::isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    stageAttributes->setRenderAssetIsPrimitive(true);
  } else if (this->isValidFileName(renderAssetHandle)) {
    // Check if renderAssetHandle is valid file name and is found in file
    // system
    // - if so then setRenderAssetIsPrimitive to false and set map of
    // IDs->Names to physicsFileObjTmpltLibByID_ - verify file  exists
    stageAttributes->setRenderAssetIsPrimitive(false);
  } else if (std::string::npos != stageAttributesHandle.find("NONE")) {
    // Render asset handle will be NONE as well - force type to be unknown
    stageAttributes->setRenderAssetType(static_cast<int>(AssetType::UNKNOWN));
    stageAttributes->setRenderAssetIsPrimitive(false);
  } else if (forceRegistration) {
    ESP_WARNING()
        << "Render asset template handle :" << renderAssetHandle
        << "specified in stage template with handle :" << stageAttributesHandle
        << "does not correspond to any existing file or primitive render "
           "asset. This attributes is not in a valid state.";
  } else {
    // If renderAssetHandle is not valid file name needs to  fail
    ESP_ERROR()
        << "Render asset template handle :" << renderAssetHandle
        << "specified in stage template with handle :" << stageAttributesHandle
        << "does not correspond to any existing file or primitive render "
           "asset.  Aborting.";
    return ID_UNDEFINED;
  }

  if (StageAttributesManager::isValidPrimitiveAttributes(
          collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    stageAttributes->setCollisionAssetIsPrimitive(true);
  } else if (this->isValidFileName(collisionAssetHandle)) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    stageAttributes->setCollisionAssetIsPrimitive(false);
  } else if (std::string::npos != stageAttributesHandle.find("NONE")) {
    // Collision asset handle will be NONE as well - force type to be unknown
    stageAttributes->setCollisionAssetType(
        static_cast<int>(AssetType::UNKNOWN));
    stageAttributes->setCollisionAssetIsPrimitive(false);
  } else {
    // Else, means no collision data specified, use specified render data
    ESP_DEBUG()
        << "Collision asset template handle :" << collisionAssetHandle
        << "specified in stage template with handle :" << stageAttributesHandle
        << "does not correspond to any existing file or primitive render "
           "asset.  Overriding with given render asset handle :"
        << renderAssetHandle << ".";

    stageAttributes->setCollisionAssetHandle(renderAssetHandle);
    stageAttributes->setCollisionAssetIsPrimitive(
        stageAttributes->getRenderAssetIsPrimitive());
  }
  // Clear dirty flag from when asset handles are changed
  stageAttributes->setIsClean();

  // adds template to library, and returns either the ID of the existing
  // template referenced by stageAttributesHandle, or the next available ID
  // if not found.
  int stageTemplateID =
      this->addObjectToLibrary(stageAttributes, stageAttributesHandle);
  return stageTemplateID;
}  // StageAttributesManager::registerAttributesTemplate

StageAttributes::ptr StageAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAssetHandle,
    bool registerTemplate) {
  // verify that a primitive asset with the given handle exists
  if (!StageAttributesManager::isValidPrimitiveAttributes(primAssetHandle)) {
    ESP_ERROR() << "No primitive with handle '" << Mn::Debug::nospace
                << primAssetHandle << Mn::Debug::nospace
                << "' exists so cannot build physical object.  Aborting.";
    return nullptr;
  }

  // construct a stageAttributes
  auto stageAttributes = initNewObjectInternal(primAssetHandle, false);
  // set margin to be 0
  stageAttributes->setMargin(0.0);

  // set render mesh handle
  int primType = static_cast<int>(AssetType::PRIMITIVE);
  stageAttributes->setRenderAssetType(primType);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  stageAttributes->setCollisionAssetType(primType);
  stageAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  return this->postCreateRegister(stageAttributes, registerTemplate);
}  // StageAttributesManager::createPrimBasedAttributesTemplate

StageAttributes::ptr StageAttributesManager::initNewObjectInternal(
    const std::string& attributesHandle,
    bool builtFromConfig) {
  // If default template exists from some source, create this template as a
  // copy
  StageAttributes::ptr newAttributes =
      this->constructFromDefault(attributesHandle);
  bool createNewAttributes = (nullptr == newAttributes);
  if (createNewAttributes) {
    newAttributes = StageAttributes::create(attributesHandle);
  }
  // attempt to set source directory if exists
  this->setFileDirectoryFromHandle(newAttributes);

  // set defaults that config files or other constructive processes might
  // override
  newAttributes->setUseMeshCollision(true);

  // set defaults from SimulatorConfig values; these can also be overridden by
  // json, for example.
  newAttributes->setLightSetup(cfgLightSetup_);
  newAttributes->setRequiresLighting(cfgLightSetup_ != NO_LIGHT_KEY);
  // set value from config so not necessary to be passed as argument
  newAttributes->setFrustumCulling(cfgFrustumCulling_);

  // only set handle defaults if attributesHandle is not a config file (which
  // would never be a valid render or collision asset name).  Otherise, expect
  // handles to be set when config is read.
  if (!builtFromConfig) {
    newAttributes->setRenderAssetHandle(attributesHandle);
    newAttributes->setCollisionAssetHandle(attributesHandle);

    // set defaults for navmesh default handles and semantic mesh default
    // handles
    std::string navmeshFilename =
        io::changeExtension(attributesHandle, ".navmesh");

    if (Corrade::Utility::Directory::exists(navmeshFilename)) {
      newAttributes->setNavmeshAssetHandle(navmeshFilename);
    }
    // Build default semantic descriptor file name
    std::string houseFilename = io::changeExtension(attributesHandle, ".house");

    if (!Corrade::Utility::Directory::exists(houseFilename)) {
      houseFilename = io::changeExtension(attributesHandle, ".scn");
    }
    newAttributes->setHouseFilename(houseFilename);

    // Build default semantic mesh file name
    const std::string semanticMeshFilename =
        io::removeExtension(houseFilename) + "_semantic.ply";
    newAttributes->setSemanticAssetHandle(semanticMeshFilename);

    // set default origin and orientation values based on file name
    // from AssetInfo::fromPath
    // set defaults for passed render asset handles
    StageAttributesManager::setDefaultAssetNameBasedAttributes(
        newAttributes, createNewAttributes,
        newAttributes->getRenderAssetHandle(), [newAttributes](auto&& PH1) {
          newAttributes->setRenderAssetType(std::forward<decltype(PH1)>(PH1));
        });
    // set defaults for passed collision asset handles
    StageAttributesManager::setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getCollisionAssetHandle(),
        [newAttributes](auto&& PH1) {
          newAttributes->setCollisionAssetType(
              std::forward<decltype(PH1)>(PH1));
        });

    // set defaults for passed semantic asset handles
    StageAttributesManager::setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getSemanticAssetHandle(),
        [newAttributes](auto&& PH1) {
          newAttributes->setSemanticAssetType(std::forward<decltype(PH1)>(PH1));
        });
  }
  // set default physical quantities specified in physics manager attributes
  if (physicsAttributesManager_->getObjectLibHasHandle(
          physicsManagerAttributesHandle_)) {
    auto physMgrAttributes = physicsAttributesManager_->getObjectByHandle(
        physicsManagerAttributesHandle_);
    newAttributes->setGravity(physMgrAttributes->getGravity());
    newAttributes->setFrictionCoefficient(
        physMgrAttributes->getFrictionCoefficient());
    newAttributes->setRestitutionCoefficient(
        physMgrAttributes->getRestitutionCoefficient());
  }
  return newAttributes;
}  // StageAttributesManager::initNewObjectInternal

void StageAttributesManager::setDefaultAssetNameBasedAttributes(
    StageAttributes::ptr attributes,
    bool setFrame,
    const std::string& fileName,
    std::function<void(int)> assetTypeSetter) {
  // TODO : support future mesh-name specific type setting?
  using Corrade::Utility::String::endsWith;

  Magnum::Vector3 up, up1{0, 1, 0}, up2{0, 0, 1};
  Magnum::Vector3 fwd, fwd1{0, 0, -1}, fwd2{0, 1, 0};

  // set default origin and orientation values based on file name
  // from AssetInfo::fromPath
  up = up1;
  fwd = fwd1;
  if (endsWith(fileName, "_semantic.ply")) {
    assetTypeSetter(static_cast<int>(AssetType::INSTANCE_MESH));
  } else if (endsWith(fileName, "mesh.ply")) {
    assetTypeSetter(static_cast<int>(AssetType::FRL_PTEX_MESH));
    up = up2;
    fwd = fwd2;
  } else if (endsWith(fileName, "house.json")) {
    assetTypeSetter(static_cast<int>(AssetType::SUNCG_SCENE));
  } else if (endsWith(fileName, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    assetTypeSetter(static_cast<int>(AssetType::MP3D_MESH));
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    up = up2;
    fwd = fwd2;
  } else if (StageAttributesManager::isValidPrimitiveAttributes(fileName)) {
    assetTypeSetter(static_cast<int>(AssetType::PRIMITIVE));
  } else {
    assetTypeSetter(static_cast<int>(AssetType::UNKNOWN));
  }
  if (setFrame) {
    attributes->setOrientUp(up);
    attributes->setOrientFront(fwd);
  }
}  // StageAttributesManager::setDefaultAssetNameBasedAttributes

void StageAttributesManager::setValsFromJSONDoc(
    attributes::StageAttributes::ptr stageAttributes,
    const io::JsonGenericValue& jsonConfig) {
  this->loadAbstractObjectAttributesFromJson(stageAttributes, jsonConfig);

  // directory location where stage files are found
  std::string stageLocFileDir = stageAttributes->getFileDirectory();

  // now parse stage-specific fields.
  // load stage specific gravity
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "gravity", [stageAttributes](auto&& PH1) {
        stageAttributes->setGravity(std::forward<decltype(PH1)>(PH1));
      });

  // load stage specific origin
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "origin", [stageAttributes](auto&& PH1) {
        stageAttributes->setOrigin(std::forward<decltype(PH1)>(PH1));
      });

  // populate specified semantic file name if specified in json - defaults
  // are overridden only if specified in json.

  std::string navmeshFName = "";
  std::string houseFName = "";
  std::string lightSetup = "";

  // populate semantic mesh type if present
  std::string semanticFName = stageAttributes->getSemanticAssetHandle();
  semanticFName = this->setJSONAssetHandleAndType(
      stageAttributes, jsonConfig, "semantic_asset_type", "semantic_asset",
      semanticFName, [stageAttributes](auto&& PH1) {
        stageAttributes->setSemanticAssetType(std::forward<decltype(PH1)>(PH1));
      });
  // if "semantic mesh" is specified in stage json to non-empty value, set
  // value (override default).
  stageAttributes->setSemanticAssetHandle(semanticFName);
  // TODO eventually remove this, but currently semantic mesh must be
  // instance
  stageAttributes->setSemanticAssetType(
      static_cast<int>(AssetType::INSTANCE_MESH));

  if (io::readMember<std::string>(jsonConfig, "nav_asset", navmeshFName)) {
    navmeshFName = Cr::Utility::Directory::join(stageLocFileDir, navmeshFName);
    // if "nav mesh" is specified in stage json set value (override default).
    stageAttributes->setNavmeshAssetHandle(navmeshFName);
  }

  if (io::readMember<std::string>(jsonConfig, "house_filename", houseFName)) {
    houseFName = Cr::Utility::Directory::join(stageLocFileDir, houseFName);
    // if "house filename" is specified in stage json, set value (override
    // default).
    stageAttributes->setHouseFilename(houseFName);
  }

  // check for user defined attributes
  this->parseUserDefinedJsonVals(stageAttributes, jsonConfig);

}  // StageAttributesManager::setValsFromJSONDoc

}  // namespace managers
}  // namespace metadata
}  // namespace esp
