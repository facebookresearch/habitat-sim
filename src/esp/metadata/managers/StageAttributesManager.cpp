// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/String.h>

#include <utility>

#include "AbstractObjectAttributesManager.h"
#include "StageAttributesManager.h"

#include "esp/assets/Asset.h"
#include "esp/assets/ResourceManager.h"
#include "esp/io/Io.h"
#include "esp/io/Json.h"

namespace esp {
using core::managedContainers::ManagedObjectAccess;

namespace metadata {

using attributes::AbstractObjectAttributes;
using attributes::StageAttributes;
namespace managers {

StageAttributesManager::StageAttributesManager(
    PhysicsAttributesManager::ptr physicsAttributesManager)
    : AbstractObjectAttributesManager<StageAttributes,
                                      ManagedObjectAccess::Copy>::
          AbstractObjectAttributesManager("Stage", "stage_config.json"),
      physicsAttributesManager_(std::move(physicsAttributesManager)),
      cfgLightSetup_(NO_LIGHT_KEY) {
  // build this manager's copy constructor map
  this->copyConstructorMap_["StageAttributes"] =
      &StageAttributesManager::createObjCopyCtorMapEntry<
          attributes::StageAttributes>;

}  // StageAttributesManager::ctor

void StageAttributesManager::createDefaultPrimBasedAttributesTemplates() {
  // create none-type stage attributes and set as undeletable
  // based on default
  auto tmplt = this->postCreateRegister(
      StageAttributesManager::initNewObjectInternal("NONE", false), true);
  std::string tmpltHandle = tmplt->getHandle();
  this->addUndeletableObjectName(std::move(tmpltHandle));
}  // StageAttributesManager::createDefaultPrimBasedAttributesTemplates

StageAttributes::ptr StageAttributesManager::createPrimBasedAttributesTemplate(
    const std::string& primAssetHandle,
    bool registerTemplate) {
  // verify that a primitive asset with the given handle exists
  if (!StageAttributesManager::isValidPrimitiveAttributes(primAssetHandle)) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "No primitive with handle '" << primAssetHandle
        << "' exists so cannot build physical object, so "
           "createPrimBasedAttributesTemplate for stage aborted.";
    return nullptr;
  }

  // construct a stageAttributes
  auto stageAttributes = initNewObjectInternal(primAssetHandle, false);
  // set margin to be 0
  stageAttributes->setMargin(0.0);

  // set render mesh handle
  const std::string primType = getAssetTypeName(AssetType::Primitive);
  stageAttributes->setRenderAssetType(primType);
  // set collision mesh/primitive handle and default for primitives to not use
  // mesh collisions
  stageAttributes->setCollisionAssetType(primType);
  stageAttributes->setUseMeshCollision(false);
  // NOTE to eventually use mesh collisions with primitive objects, a
  // collision primitive mesh needs to be configured and set in MeshMetaData
  // and CollisionMesh

  return this->postCreateRegister(std::move(stageAttributes), registerTemplate);
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
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  if (!createNewAttributes) {
    // default exists and was used to create this attributes - investigate any
    // filename fields that may have %%USE_FILENAME%% directive specified in the
    // default attributes.
    // Render asset handle
    setFilenameFromDefaultTag(newAttributes,
                              newAttributes->getRenderAssetHandle(),
                              [newAttributes](const std::string& newHandle) {
                                newAttributes->setRenderAssetHandle(newHandle);
                              });
    // Collision asset handle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getCollisionAssetHandle(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setCollisionAssetHandle(newHandle);
        });
    // navmesh asset handle
    setFilenameFromDefaultTag(newAttributes,
                              newAttributes->getNavmeshAssetHandle(),
                              [newAttributes](const std::string& newHandle) {
                                newAttributes->setNavmeshAssetHandle(newHandle);
                              });
    // Semantic Scene Descriptor text filehandle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getSemanticDescriptorFilename(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setSemanticDescriptorFilename(newHandle);
        });
    // Semantic Scene asset handle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getSemanticAssetHandle(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setSemanticAssetHandle(newHandle);
        });
  }

  // set defaults that config files or other constructive processes might
  // override

  // set defaults from SimulatorConfig values;
  newAttributes->setLightSetupKey(cfgLightSetup_);
  newAttributes->setForceFlatShading(cfgLightSetup_ == NO_LIGHT_KEY);
  // set value from config so not necessary to be passed as argument
  newAttributes->setFrustumCulling(cfgFrustumCulling_);

  // only set handle defaults if attributesHandle is not a config file (which
  // would never be a valid render or collision asset name).  Otherise, expect
  // handles to be set when config is read.
  if (!builtFromConfig) {
    if (newAttributes->getRenderAssetHandle().empty()) {
      newAttributes->setRenderAssetHandle(attributesHandle);
    }
    if (newAttributes->getCollisionAssetHandle().empty()) {
      newAttributes->setCollisionAssetHandle(attributesHandle);
    }

    if (attributesHandle != "NONE") {
      // TODO when all datasets have configuration support, get rid of all these
      // default settings

      // set defaults for navmesh, semantic mesh and lexicon handles
      // start with root stage name, including path but without final extension
      // default navmesh should have .navmesh ext
      if (newAttributes->getNavmeshAssetHandle().empty()) {
        std::string navmeshFilename =
            this->findFilenameUsingCriteria(attributesHandle, {".navmesh"});
        // if present, file was found, so set value (overriding attributes
        // default)
        if (!navmeshFilename.empty()) {
          newAttributes->setNavmeshAssetHandle(navmeshFilename);
        }
      }

      if (newAttributes->getSemanticDescriptorFilename().empty()) {
        // Build default semantic descriptor file name, using extensions of
        std::string ssdFileName = this->findFilenameUsingCriteria(
            attributesHandle, {".house", ".scn", "_semantic.txt"});

        // if not present, set hacky defaults for back compat expectations.
        if (ssdFileName.empty()) {
          if (attributesHandle.find("/replica_dataset") != std::string::npos) {
            // replica hack until dataset gets appropriate configuration support
            ssdFileName = Cr::Utility::Path::join(
                newAttributes->getFileDirectory(), "info_semantic.json");
          } else {
            // hack to support back-compat until configs are implemented for all
            // datasets
            ssdFileName =
                Cr::Utility::Path::splitExtension(attributesHandle).first() +
                ".scn";
          }
        }
        newAttributes->setSemanticDescriptorFilename(ssdFileName);
      }

      if (newAttributes->getSemanticAssetHandle().empty()) {
        // Build default semantic mesh filename as root stage name ending with
        // "_semantic.ply", for back-compat with Mp3d
        std::string semanticMeshFilename = this->findFilenameUsingCriteria(
            attributesHandle, {"_semantic.ply"});
        // if present, file was found, so set value (overriding attributes
        // default)
        if (!semanticMeshFilename.empty()) {
          newAttributes->setSemanticAssetHandle(semanticMeshFilename);
        }
      }
    }  // do not populate defaults for NONE scene

    // set default origin and orientation values based on file name
    // from AssetInfo::fromPath
    // set defaults for passed render asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, createNewAttributes,
        newAttributes->getRenderAssetHandle(), [newAttributes](auto&& PH1) {
          newAttributes->initRenderAssetTypeEnum(
              std::forward<decltype(PH1)>(PH1));
        });
    // set defaults for passed collision asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getCollisionAssetHandle(),
        [newAttributes](auto&& PH1) {
          newAttributes->initCollisionAssetTypeEnum(
              std::forward<decltype(PH1)>(PH1));
        });

    // set defaults for passed semantic asset handles
    this->setDefaultAssetNameBasedAttributes(
        newAttributes, false, newAttributes->getSemanticAssetHandle(),
        [newAttributes](auto&& PH1) {
          newAttributes->initSemanticAssetTypeEnum(
              std::forward<decltype(PH1)>(PH1));
        });
    // TODO : get rid of this once the hardcoded mesh-type handling is removed,
    // but for now force all semantic assets to be instance_mesh
    newAttributes->initSemanticAssetTypeEnum(AssetType::InstanceMesh);
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
    const std::function<void(AssetType)>& assetTypeSetter) {
  // TODO : support future mesh-name specific type setting?
  using Corrade::Utility::String::endsWith;

  Magnum::Vector3 up, up1{0.0, 1.0, 0.0}, up2{0.0, 0.0, 1.0};
  Magnum::Vector3 fwd, fwd1{0.0, 0.0, -1.0}, fwd2{0.0, 1.0, 0.0};

  // set default origin and orientation values based on file name
  // from AssetInfo::fromPath
  up = up1;
  fwd = fwd1;
  if (endsWith(fileName, "_semantic.ply")) {
    assetTypeSetter(AssetType::InstanceMesh);
  } else if (endsWith(fileName, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    assetTypeSetter(AssetType::Mp3dMesh);
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    up = up2;
    fwd = fwd2;
  } else if (this->isValidPrimitiveAttributes(fileName)) {
    assetTypeSetter(AssetType::Primitive);
  } else {
    assetTypeSetter(AssetType::Unknown);
  }
  if (setFrame) {
    attributes->init("up", up);
    attributes->init("front", fwd);
  }
}  // StageAttributesManager::setDefaultAssetNameBasedAttributes

void StageAttributesManager::setValsFromJSONDocInternal(
    attributes::StageAttributes::ptr stageAttributes,
    const io::JsonGenericValue& jsonConfig) {
  this->setAbstractObjectAttributesFromJson(stageAttributes, jsonConfig);

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

  // load stage semantic asset specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "semantic_up", [stageAttributes](const Magnum::Vector3& up) {
        stageAttributes->setSemanticOrientUp(up);
      });

  // load stage semantic asset specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "semantic_front",
      [stageAttributes](const Magnum::Vector3& front) {
        stageAttributes->setSemanticOrientFront(front);
      });

  // load whether the semantic asset for this stage has semantically annotated
  // textures
  io::jsonIntoSetter<bool>(
      jsonConfig, "has_semantic_textures",
      [stageAttributes](bool has_semantic_textures) {
        stageAttributes->setHasSemanticTextures(has_semantic_textures);
      });

  // populate specified semantic file name if specified in json - defaults
  // are overridden only if specified in json.

  // populate semantic mesh type if present
  std::string semanticFName = stageAttributes->getSemanticAssetHandle();
  semanticFName = this->setJSONAssetHandleAndType(
      stageAttributes, jsonConfig, "semantic_asset_type", "semantic_asset",
      semanticFName, [stageAttributes](auto&& PH1) {
        stageAttributes->setSemanticAssetTypeEnum(
            std::forward<decltype(PH1)>(PH1));
      });
  // if "semantic mesh" is specified in stage json to non-empty value, set
  // value (override default).
  stageAttributes->setSemanticAssetHandle(semanticFName);
  // TODO eventually remove this, but currently semantic mesh must be
  // instance
  stageAttributes->initSemanticAssetTypeEnum(AssetType::InstanceMesh);

  std::string navmeshFName = "";
  if (io::readMember<std::string>(jsonConfig, "nav_asset", navmeshFName)) {
    // if "nav mesh" is specified in stage json set value (override default).
    // navmesh filename might already be fully qualified; if not, might just be
    // file name
    if (!Corrade::Utility::Path::exists(navmeshFName) &&
        !navmeshFName.empty()) {
      navmeshFName = Cr::Utility::Path::join(stageLocFileDir, navmeshFName);
    }
    stageAttributes->setNavmeshAssetHandle(navmeshFName);
  }

  std::string semanticSceneDescriptor = "";
  if (io::readMember<std::string>(jsonConfig, "semantic_descriptor_filename",
                                  semanticSceneDescriptor)) {
    // if "semantic_descriptor_filename" is specified in stage json, set value
    // (override default).
    // semanticSceneDescriptor filename might already be fully qualified; if
    // not, might just be file name
    if (!Corrade::Utility::Path::exists(semanticSceneDescriptor) &&
        !semanticSceneDescriptor.empty()) {
      semanticSceneDescriptor =
          Cr::Utility::Path::join(stageLocFileDir, semanticSceneDescriptor);
    }
    stageAttributes->setSemanticDescriptorFilename(semanticSceneDescriptor);
  }
  // check for user defined attributes
  this->parseUserDefinedJsonVals(stageAttributes, jsonConfig);

}  // StageAttributesManager::setValsFromJSONDoc

core::managedContainers::ManagedObjectPreregistration
StageAttributesManager::preRegisterObjectFinalize(
    StageAttributes::ptr stageAttributes,
    const std::string& stageAttributesHandle,
    bool forceRegistration) {
  if (stageAttributes->getRenderAssetHandle().empty()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Attributes template named `" << stageAttributesHandle
        << "` does not have a valid render asset handle specified, so "
           "StageAttributes registration is aborted.";
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  }

  // Handles for rendering assets
  const std::string renderAssetHandle = stageAttributes->getRenderAssetHandle();
  const std::string renderAssetFullPath =
      stageAttributes->getRenderAssetFullPath();
  bool stageIsNone = stageAttributesHandle.find("NONE") != std::string::npos;
  // verify these represent legitimate assets
  if (this->isValidPrimitiveAttributes(renderAssetHandle)) {
    // If renderAssetHandle corresponds to valid/existing primitive attributes
    // then setRenderAssetIsPrimitive to true and set map of IDs->Names to
    // physicsSynthObjTmpltLibByID_
    stageAttributes->setRenderAssetIsPrimitive(true);
    stageAttributes->setRenderAssetFullPath(renderAssetHandle);
  } else if (Cr::Utility::Path::exists(renderAssetHandle) ||
             Cr::Utility::Path::exists(renderAssetFullPath)) {
    // Check if renderAssetHandle is valid file name and is found in file
    // system - if so then setRenderAssetIsPrimitive to false
    stageAttributes->setRenderAssetIsPrimitive(false);
  } else if (stageIsNone) {
    // Render asset handle will be NONE as well - force type to be unknown
    stageAttributes->setRenderAssetTypeEnum(AssetType::Unknown);
    stageAttributes->setRenderAssetIsPrimitive(false);
    stageAttributes->setRenderAssetFullPath("NONE");
  } else if (forceRegistration) {
    ESP_WARNING()
        << "Render asset template handle :" << renderAssetHandle
        << "specified in stage template with handle `" << stageAttributesHandle
        << "` does not correspond to any existing file or primitive render "
           "asset. This attributes is not in a valid state.";
  } else {
    // If renderAssetHandle is not valid file name needs to fail
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Render asset template handle `" << renderAssetHandle
        << "` specified in stage template with handle :"
        << stageAttributesHandle
        << "does not correspond to any existing file or primitive render "
           "asset, so StageAttributes registration is aborted.";
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  }

  // Handles for collision assets
  const std::string collisionAssetHandle =
      stageAttributes->getCollisionAssetHandle();
  const std::string collisionAssetFullPath =
      stageAttributes->getCollisionAssetFullPath();
  if (this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    // If collisionAssetHandle corresponds to valid/existing primitive
    // attributes then setCollisionAssetIsPrimitive to true
    stageAttributes->setCollisionAssetIsPrimitive(true);
    stageAttributes->setCollisionAssetFullPath(collisionAssetHandle);
  } else if (Cr::Utility::Path::exists(collisionAssetHandle) ||
             (Cr::Utility::Path::exists(collisionAssetFullPath))) {
    // Check if collisionAssetHandle is valid file name and is found in file
    // system - if so then setCollisionAssetIsPrimitive to false
    stageAttributes->setCollisionAssetIsPrimitive(false);
  } else if (stageIsNone) {
    // Collision asset handle will be NONE as well - force type to be unknown
    stageAttributes->setCollisionAssetType(
        getAssetTypeName(AssetType::Unknown));
    stageAttributes->setCollisionAssetIsPrimitive(false);
    stageAttributes->setCollisionAssetFullPath("NONE");
  } else {
    // Else, means no collision data specified, use specified render data
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Collision asset template handle : `" << collisionAssetHandle
        << "` specified in stage template with handle : `"
        << stageAttributesHandle
        << "` does not correspond to any existing file or known primitive "
           "asset. Overriding with given render asset handle : `"
        << renderAssetHandle << "`.";

    stageAttributes->setCollisionAssetHandle(
        stageAttributes->getRenderAssetHandle());
    stageAttributes->setCollisionAssetFullPath(
        stageAttributes->getRenderAssetFullPath());
    stageAttributes->setCollisionAssetIsPrimitive(
        stageAttributes->getRenderAssetIsPrimitive());
  }
  // filter all paths properly so that the handles don't have filepaths and the
  // accessors are hidden fields

  this->finalizeAttrPathsBeforeRegister(stageAttributes);
  // Clear dirty flag from when asset handles are changed
  stageAttributes->setFilePathsAreClean();

  return core::managedContainers::ManagedObjectPreregistration::Success;

}  // StageAttributesManager::preRegisterObjectFinalize

void StageAttributesManager::finalizeAttrPathsBeforeRegister(
    const attributes::StageAttributes::ptr& stageAttributes) const {
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "BEFORE : Stage `" << stageAttributes->getHandle() << "`: Render rel `"
      << stageAttributes->getRenderAssetHandle() << "`: Render FQ `"
      << stageAttributes->getRenderAssetFullPath() << "`| Collision rel `"
      << stageAttributes->getCollisionAssetHandle() << "`| Collision FQ `"
      << stageAttributes->getCollisionAssetFullPath() << "`| navmesh rel `"
      << stageAttributes->getNavmeshAssetHandle() << "`| semantic asset rel `"
      << stageAttributes->getSemanticAssetHandle()
      << "`| semantic descriptor rel `"
      << stageAttributes->getSemanticDescriptorFilename() << "`| file dir `"
      << stageAttributes->getFileDirectory() << "`";

  // Render asset filename filter out path and set internal reference to full
  // filepath
  const std::string renderAssetHandle = stageAttributes->getRenderAssetHandle();
  if (!this->isValidPrimitiveAttributes(renderAssetHandle)) {
    this->filterAttribsFilenames(
        stageAttributes, renderAssetHandle,
        stageAttributes->getRenderAssetFullPath(),
        [stageAttributes](const std::string& renderAsset) {
          stageAttributes->setRenderAssetHandle(renderAsset);
        },
        [stageAttributes](const std::string& renderAsset) {
          stageAttributes->setRenderAssetFullPath(renderAsset);
        });
  } else {
    // If handle refs a prim then just copy it over to full path
    stageAttributes->setRenderAssetFullPath(renderAssetHandle);
  }
  // Collision asset filename filter out path and set internal reference to
  // full filepaath
  const std::string collisionAssetHandle =
      stageAttributes->getCollisionAssetHandle();
  if (!this->isValidPrimitiveAttributes(collisionAssetHandle)) {
    this->filterAttribsFilenames(
        stageAttributes, collisionAssetHandle,
        stageAttributes->getCollisionAssetFullPath(),
        [stageAttributes](const std::string& colHndl) {
          stageAttributes->setCollisionAssetHandle(colHndl);
        },
        [stageAttributes](const std::string& colHndl) {
          stageAttributes->setCollisionAssetFullPath(colHndl);
        });
  } else {
    // If handle refs a prim then just copy it over to full path
    stageAttributes->setCollisionAssetFullPath(collisionAssetHandle);
  }
  // filter filepaths of full path qualifiers
  // Navmesh asset filename
  this->filterAttribsFilenames(
      stageAttributes, stageAttributes->getNavmeshAssetHandle(),
      stageAttributes->getNavmeshAssetFullPath(),
      [stageAttributes](const std::string& navmeshAsset) {
        stageAttributes->setNavmeshAssetHandle(navmeshAsset);
      },
      [stageAttributes](const std::string& navmeshAsset) {
        stageAttributes->setNavmeshAssetFullPath(navmeshAsset);
      });
  // Semantic asset filename
  this->filterAttribsFilenames(
      stageAttributes, stageAttributes->getSemanticAssetHandle(),
      stageAttributes->getSemanticAssetFullPath(),
      [stageAttributes](const std::string& semanticAsset) {
        stageAttributes->setSemanticAssetHandle(semanticAsset);
      },
      [stageAttributes](const std::string& semanticAsset) {
        stageAttributes->setSemanticAssetFullPath(semanticAsset);
      });
  // Semantic descriptor filename
  this->filterAttribsFilenames(
      stageAttributes, stageAttributes->getSemanticDescriptorFilename(),
      stageAttributes->getSemanticDescriptorFullPath(),
      [stageAttributes](const std::string& ssd) {
        stageAttributes->setSemanticDescriptorFilename(ssd);
      },
      [stageAttributes](const std::string& ssd) {
        stageAttributes->setSemanticDescriptorFullPath(ssd);
      });

  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "AFTER : Stage `" << stageAttributes->getHandle() << "`: Render rel `"
      << stageAttributes->getRenderAssetHandle() << "`: Render FQ `"
      << stageAttributes->getRenderAssetFullPath() << "`| Collision rel `"
      << stageAttributes->getCollisionAssetHandle() << "`| Collision FQ `"
      << stageAttributes->getCollisionAssetFullPath() << "`| navmesh rel `"
      << stageAttributes->getNavmeshAssetHandle() << "`| semantic asset rel `"
      << stageAttributes->getSemanticAssetHandle()
      << "`| semantic descriptor rel `"
      << stageAttributes->getSemanticDescriptorFilename() << "`| file dir `"
      << stageAttributes->getFileDirectory() << "`";

  // Clear dirty flag from when asset handles are changed
}

}  // namespace managers
}  // namespace metadata
}  // namespace esp
