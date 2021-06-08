// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ResourceManager.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/PointerStl.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/PluginManager/PluginMetadata.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Tags.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/VertexFormat.h>

#include <memory>

#include "esp/geo/geo.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/MaterialUtil.h"
#include "esp/gfx/PbrDrawable.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/io/URDFParser.h"
#include "esp/io/io.h"
#include "esp/io/json.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneGraph.h"

#include "esp/nav/PathFinder.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

#include "CollisionMeshData.h"
#include "GenericInstanceMeshData.h"
#include "GenericMeshData.h"
#include "MeshData.h"

#ifdef ESP_BUILD_PTEX_SUPPORT
#include "PTexMeshData.h"
#include "esp/gfx/PTexMeshDrawable.h"
#include "esp/gfx/PTexMeshShader.h"
#endif

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
using metadata::attributes::AbstractObjectAttributes;
using metadata::attributes::CubePrimitiveAttributes;
using metadata::attributes::ObjectAttributes;
using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::StageAttributes;
using metadata::managers::AssetAttributesManager;
using metadata::managers::ObjectAttributesManager;
using metadata::managers::PhysicsAttributesManager;
using metadata::managers::StageAttributesManager;

namespace assets {

ResourceManager::ResourceManager(
    metadata::MetadataMediator::ptr& _metadataMediator,
    Flags _flags)
    : flags_(_flags),
      metadataMediator_(_metadataMediator)
#ifdef MAGNUM_BUILD_STATIC
      ,
      // avoid using plugins that might depend on different library versions
      importerManager_("nonexistent")
#endif
{
#ifdef ESP_BUILD_WITH_VHACD
  // Destructor is protected, using Clean() and Release() to destruct interface
  // (this is how it is used VHACD examples.)
  interfaceVHACD = VHACD::CreateVHACD();
#endif
  initDefaultLightSetups();
  initDefaultMaterials();
  buildImporters();
}

ResourceManager::~ResourceManager() {
#ifdef ESP_BUILD_WITH_VHACD
  interfaceVHACD->Clean();
  interfaceVHACD->Release();
#endif
}

void ResourceManager::buildImporters() {
  // instantiate a primitive importer
  CORRADE_INTERNAL_ASSERT_OUTPUT(
      primitiveImporter_ =
          importerManager_.loadAndInstantiate("PrimitiveImporter"));
  // necessary for importer to be usable
  primitiveImporter_->openData("");
  // instantiate importer for file load
  CORRADE_INTERNAL_ASSERT_OUTPUT(
      fileImporter_ = importerManager_.loadAndInstantiate("AnySceneImporter"));

}  // buildImporters

void ResourceManager::initDefaultPrimAttributes() {
  // by this point, we should have a GL::Context so load the bb primitive.
  // TODO: replace this completely with standard mesh (i.e. treat the bb
  // wireframe cube no differently than other primivite-based rendered
  // objects)
  auto cubeMeshName =
      getAssetAttributesManager()
          ->getObjectCopyByHandle<CubePrimitiveAttributes>("cubeWireframe")
          ->getPrimObjClassName();

  auto wfCube = primitiveImporter_->mesh(cubeMeshName);
  primitive_meshes_[nextPrimitiveMeshId++] =
      std::make_unique<Magnum::GL::Mesh>(Magnum::MeshTools::compile(*wfCube));

}  // initDefaultPrimAttributes

void ResourceManager::initPhysicsManager(
    std::shared_ptr<physics::PhysicsManager>& physicsManager,
    bool isEnabled,
    scene::SceneNode* parent,
    const metadata::attributes::PhysicsManagerAttributes::ptr&
        physicsManagerAttributes) {
  //! PHYSICS INIT: Use the passed attributes to initialize physics engine
  bool defaultToNoneSimulator = true;
  if (isEnabled) {
    if (physicsManagerAttributes->getSimulator() == "bullet") {
#ifdef ESP_BUILD_WITH_BULLET
      physicsManager = std::make_shared<physics::BulletPhysicsManager>(
          *this, physicsManagerAttributes);
      defaultToNoneSimulator = false;
#else
      LOG(WARNING)
          << ":\n---\nPhysics was enabled and Bullet physics engine was "
             "specified, but the project is built without Bullet support. "
             "Objects added to the scene will be restricted to kinematic "
             "updates "
             "only. Reinstall with --bullet to enable Bullet dynamics.\n---";
#endif
    }
  }
  // reset to base PhysicsManager to override previous as default behavior
  // if the desired simulator is not supported reset to "none" in metaData
  if (defaultToNoneSimulator) {
    physicsManagerAttributes->setSimulator("none");
    physicsManager = std::make_shared<physics::PhysicsManager>(
        *this, physicsManagerAttributes);
  }

  // build default primitive asset templates, and default primitive object
  // templates
  initDefaultPrimAttributes();

  // initialize the physics simulator
  physicsManager->initPhysics(parent);
}  // ResourceManager::initPhysicsManager

bool ResourceManager::loadStage(
    StageAttributes::ptr& stageAttributes,
    const std::shared_ptr<physics::PhysicsManager>& _physicsManager,
    esp::scene::SceneManager* sceneManagerPtr,
    std::vector<int>& activeSceneIDs,
    bool createSemanticMesh,
    bool forceSeparateSemanticSceneGraph) {
  // create AssetInfos here for each potential mesh file for the scene, if they
  // are unique.
  bool buildCollisionMesh =
      ((_physicsManager != nullptr) &&
       (_physicsManager->getInitializationAttributes()->getSimulator() !=
        "none"));
  const std::string renderLightSetupKey(stageAttributes->getLightSetup());
  std::map<std::string, AssetInfo> assetInfoMap =
      createStageAssetInfosFromAttributes(stageAttributes, buildCollisionMesh,
                                          createSemanticMesh);

  // set equal to current Simulator::activeSemanticSceneID_ value
  int activeSemanticSceneID = activeSceneIDs[0];
  // if semantic scene load is requested and possible
  if (assetInfoMap.count("semantic") != 0u) {
    // check if file names exist
    AssetInfo semanticInfo = assetInfoMap.at("semantic");
    auto semanticStageFilename = semanticInfo.filepath;
    if (Cr::Utility::Directory::exists(semanticStageFilename)) {
      LOG(INFO) << "ResourceManager::loadStage : Loading Semantic Stage mesh : "
                << semanticStageFilename;
      activeSemanticSceneID = sceneManagerPtr->initSceneGraph();

      auto& semanticSceneGraph =
          sceneManagerPtr->getSceneGraph(activeSemanticSceneID);
      auto& semanticRootNode = semanticSceneGraph.getRootNode();
      auto& semanticDrawables = semanticSceneGraph.getDrawables();

      RenderAssetInstanceCreationInfo::Flags flags;
      flags |= RenderAssetInstanceCreationInfo::Flag::IsSemantic;
      if (stageAttributes->getFrustumCulling()) {
        // only treat as static if doing culling
        flags |= RenderAssetInstanceCreationInfo::Flag::IsStatic;
      }
      RenderAssetInstanceCreationInfo creation(
          semanticStageFilename, Cr::Containers::NullOpt, flags, NO_LIGHT_KEY);

      bool semanticStageSuccess =
          loadStageInternal(semanticInfo,  // AssetInfo
                            &creation,
                            &semanticRootNode,    // parent scene node
                            &semanticDrawables);  // drawable group

      // regardless of load failure, original code still changed
      // activeSemanticSceneID_
      if (!semanticStageSuccess) {
        LOG(ERROR) << " ResourceManager::loadStage : Semantic Stage mesh "
                      "load failed.";
        return false;
      } else {
        LOG(INFO) << "ResourceManager::loadStage : Semantic Stage mesh : "
                  << semanticStageFilename << " loaded.";
      }
    } else {  // semantic file name does not exist but house does
      LOG(WARNING)
          << "ResourceManager::loadStage : Not loading semantic mesh - "
             "File Name : "
          << semanticStageFilename << " does not exist.";
    }
  } else {  // not wanting to create semantic mesh
    LOG(INFO) << "ResourceManager::loadStage : Not loading semantic mesh";
  }

  if (forceSeparateSemanticSceneGraph &&
      activeSemanticSceneID == activeSceneIDs[0]) {
    // Create a separate semantic scene graph if it wasn't already created
    // above.
    activeSemanticSceneID = sceneManagerPtr->initSceneGraph();
  }

  // save active semantic scene ID so that simulator can consume
  activeSceneIDs[1] = activeSemanticSceneID;
  const bool isSeparateSemanticScene = activeSceneIDs[1] != activeSceneIDs[0];

  auto& sceneGraph = sceneManagerPtr->getSceneGraph(activeSceneIDs[0]);
  auto& rootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();

  AssetInfo renderInfo = assetInfoMap.at("render");

  RenderAssetInstanceCreationInfo::Flags flags;
  flags |= RenderAssetInstanceCreationInfo::Flag::IsStatic;
  flags |= RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  if (!isSeparateSemanticScene) {
    flags |= RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  }
  RenderAssetInstanceCreationInfo renderCreation(
      renderInfo.filepath, Cr::Containers::NullOpt, flags, renderLightSetupKey);
  LOG(INFO) << "ResourceManager::loadStage : start load render asset "
            << renderInfo.filepath << ".";

  bool renderMeshSuccess = loadStageInternal(renderInfo,  // AssetInfo
                                             &renderCreation,
                                             &rootNode,    // parent scene node
                                             &drawables);  //  drawable group
  if (!renderMeshSuccess) {
    LOG(ERROR)
        << " ResourceManager::loadStage : Stage render mesh load failed, "
           "Aborting scene initialization.";
    return false;
  }
  // declare mesh group variable
  std::vector<CollisionMeshData> meshGroup;
  AssetInfo& infoToUse = renderInfo;
  if (assetInfoMap.count("collision") != 0u) {
    AssetInfo colInfo = assetInfoMap.at("collision");
    if (resourceDict_.count(colInfo.filepath) == 0) {
      LOG(INFO) << "ResourceManager::loadStage : start load collision asset "
                << colInfo.filepath << ".";
      // will not reload if already present
      bool collisionMeshSuccess =
          loadStageInternal(colInfo,   // AssetInfo
                            nullptr,   // creation
                            nullptr,   // parent scene node
                            nullptr);  // drawable group

      if (!collisionMeshSuccess) {
        LOG(ERROR) << " ResourceManager::loadStage : Stage collision mesh "
                      "load failed.  Aborting scene initialization.";
        return false;
      }
    }
    // if we have a collision mesh, and it does not exist already as a
    // collision object, add it
    if (colInfo.filepath != EMPTY_SCENE) {
      infoToUse = colInfo;
    }  // if not colInfo.filepath.compare(EMPTY_SCENE)
  }    // if collision mesh desired
  // build the appropriate mesh groups, either for the collision mesh, or, if
  // the collision mesh is empty scene

  if ((_physicsManager != nullptr) && (infoToUse.filepath != EMPTY_SCENE)) {
    bool success = buildMeshGroups(infoToUse, meshGroup);
    if (!success) {
      return false;
    }
    //! Add to physics manager - will only be null for certain tests
    // Either add with pre-built meshGroup if collision assets are loaded
    // or empty vector for mesh group - this should only be the case if
    // we are using None-type physicsManager.
    bool sceneSuccess = _physicsManager->addStage(stageAttributes, meshGroup);
    if (!sceneSuccess) {
      LOG(ERROR) << "ResourceManager::loadStage : Adding Stage "
                 << stageAttributes->getHandle()
                 << " to PhysicsManager failed. Aborting scene initialization.";
      return false;
    }
  }

  return true;
}  // ResourceManager::loadScene
bool ResourceManager::buildMeshGroups(
    const AssetInfo& info,
    std::vector<CollisionMeshData>& meshGroup) {
  if (collisionMeshGroups_.count(info.filepath) == 0) {
    //! Collect collision mesh group
    bool colMeshGroupSuccess = false;
    if (info.type == AssetType::INSTANCE_MESH) {
      // PLY Instance mesh
      colMeshGroupSuccess =
          buildStageCollisionMeshGroup<GenericInstanceMeshData>(info.filepath,
                                                                meshGroup);
    } else if (info.type == AssetType::MP3D_MESH ||
               info.type == AssetType::UNKNOWN) {
      // GLB Mesh
      colMeshGroupSuccess = buildStageCollisionMeshGroup<GenericMeshData>(
          info.filepath, meshGroup);
    }
#ifdef ESP_BUILD_PTEX_SUPPORT
    else if (info.type == AssetType::FRL_PTEX_MESH) {
      colMeshGroupSuccess =
          buildStageCollisionMeshGroup<PTexMeshData>(info.filepath, meshGroup);
    }
#endif

    // failure during build of collision mesh group
    if (!colMeshGroupSuccess) {
      LOG(ERROR) << "ResourceManager::loadStage : Stage " << info.filepath
                 << " Collision mesh load failed. Aborting scene "
                    "initialization.";
      return false;
    }
    //! Add scene meshgroup to collision mesh groups
    collisionMeshGroups_.emplace(info.filepath, meshGroup);
  } else {
    // collision meshGroup already exists from prior load
    meshGroup = collisionMeshGroups_.at(info.filepath);
  }
  return true;
}  // ResourceManager::buildMeshGroups

std::map<std::string, AssetInfo>
ResourceManager::createStageAssetInfosFromAttributes(
    const StageAttributes::ptr& stageAttributes,
    bool createCollisionInfo,
    bool createSemanticInfo) {
  std::map<std::string, AssetInfo> resMap;
  auto frame =
      buildFrameFromAttributes(stageAttributes, stageAttributes->getOrigin());
  float virtualUnitToMeters = stageAttributes->getUnitsToMeters();
  // create render asset info
  auto renderType =
      static_cast<AssetType>(stageAttributes->getRenderAssetType());
  AssetInfo renderInfo{
      renderType,                               // type
      stageAttributes->getRenderAssetHandle(),  // file path
      frame,                                    // frame
      virtualUnitToMeters,                      // virtualUnitToMeters
      stageAttributes->getRequiresLighting()    // requiresLighting
  };
  resMap["render"] = renderInfo;
  if (createCollisionInfo) {
    // create collision asset info if requested
    auto colType =
        static_cast<AssetType>(stageAttributes->getCollisionAssetType());
    AssetInfo collisionInfo{
        colType,                                     // type
        stageAttributes->getCollisionAssetHandle(),  // file path
        frame,                                       // frame
        virtualUnitToMeters,                         // virtualUnitToMeters
        false                                        // requiresLighting
    };
    resMap["collision"] = collisionInfo;
  }
  if (createSemanticInfo) {
    // create semantic asset info if requested
    auto semanticType =
        static_cast<AssetType>(stageAttributes->getSemanticAssetType());
    AssetInfo semanticInfo{
        semanticType,                               // type
        stageAttributes->getSemanticAssetHandle(),  // file path
        frame,                                      // frame
        virtualUnitToMeters,                        // virtualUnitToMeters
        false,                                      // requiresLighting
        // only split instance mesh if doing frustum culling
        stageAttributes->getFrustumCulling()  // splitInstanceMesh
    };
    resMap["semantic"] = semanticInfo;
  }
  return resMap;
}  // ResourceManager::createStageAssetInfosFromAttributes

esp::geo::CoordinateFrame ResourceManager::buildFrameFromAttributes(
    const AbstractObjectAttributes::ptr& attribs,
    const Magnum::Vector3& origin) {
  const vec3f upEigen{
      Mn::EigenIntegration::cast<vec3f>(attribs->getOrientUp())};
  const vec3f frontEigen{
      Mn::EigenIntegration::cast<vec3f>(attribs->getOrientFront())};
  if (upEigen.isOrthogonal(frontEigen)) {
    const vec3f originEigen{Mn::EigenIntegration::cast<vec3f>(origin)};
    esp::geo::CoordinateFrame frame{upEigen, frontEigen, originEigen};
    return frame;
  } else {
    LOG(INFO) << "ResourceManager::buildFrameFromAttributes : Specified frame "
                 "in Attributes : "
              << attribs->getHandle()
              << " is not orthogonal, so returning default frame.";
    esp::geo::CoordinateFrame frame;
    return frame;
  }
}  // ResourceManager::buildCoordFrameFromAttribVals

std::string ResourceManager::createColorMaterial(
    const esp::assets::PhongMaterialColor& materialColor) {
  std::ostringstream matHandleStream;
  matHandleStream << "phong_amb_" << materialColor.ambientColor.toSrgbAlphaInt()
                  << "_dif_" << materialColor.diffuseColor.toSrgbAlphaInt()
                  << "_spec_" << materialColor.specularColor.toSrgbAlphaInt();
  std::string newMaterialID = matHandleStream.str();
  auto materialResource = shaderManager_.get<gfx::MaterialData>(newMaterialID);
  if (materialResource.state() == Mn::ResourceState::NotLoadedFallback) {
    gfx::PhongMaterialData::uptr phongMaterial =
        gfx::PhongMaterialData::create_unique();
    phongMaterial->ambientColor = materialColor.ambientColor;
    phongMaterial->diffuseColor = materialColor.diffuseColor;
    phongMaterial->specularColor = materialColor.specularColor;

    std::unique_ptr<gfx::MaterialData> finalMaterial(phongMaterial.release());
    shaderManager_.set(newMaterialID, finalMaterial.release());
  }
  return newMaterialID;
}  // ResourceManager::createColorMaterial

scene::SceneNode* ResourceManager::loadAndCreateRenderAssetInstance(
    const AssetInfo& assetInfo,
    const RenderAssetInstanceCreationInfo& creation,
    esp::scene::SceneManager* sceneManagerPtr,
    const std::vector<int>& activeSceneIDs) {
  // We map isStatic, isSemantic, and isRGBD to a scene graph.
  int sceneID = -1;
  if (!creation.isStatic()) {
    // Non-static instances must always get added to the RGBD scene graph, with
    // nodeType==OBJECT, and they will be drawn for both RGBD and Semantic
    // sensors.
    if (!(creation.isSemantic() && creation.isRGBD())) {
      LOG(WARNING) << "unsupported instance creation flags for asset ["
                   << assetInfo.filepath << "]";
      return nullptr;
    }
    sceneID = activeSceneIDs[0];
  } else {
    if (creation.isSemantic() && creation.isRGBD()) {
      if (activeSceneIDs[1] != activeSceneIDs[0]) {
        // Because we have a separate semantic scene graph, we can't support a
        // static instance with both isSemantic and isRGBD.
        LOG(WARNING)
            << "unsupported instance creation flags for asset ["
            << assetInfo.filepath
            << "] with "
               "SimulatorConfiguration::forceSeparateSemanticSceneGraph=true.";
        return nullptr;
      }
      sceneID = activeSceneIDs[0];
    } else {
      if (activeSceneIDs[1] == activeSceneIDs[0]) {
        // A separate semantic scene graph wasn't constructed, so we can't
        // support a Semantic-only (or RGBD-only) instance.
        LOG(WARNING)
            << "unsupported instance creation flags for asset ["
            << assetInfo.filepath
            << "] with "
               "SimulatorConfiguration::forceSeparateSemanticSceneGraph=false.";
        return nullptr;
      }
      sceneID = creation.isSemantic() ? activeSceneIDs[1] : activeSceneIDs[0];
    }
  }

  auto& sceneGraph = sceneManagerPtr->getSceneGraph(sceneID);
  auto& rootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();

  return loadAndCreateRenderAssetInstance(assetInfo, creation, &rootNode,
                                          &drawables);
}  // ResourceManager::loadAndCreateRenderAssetInstance

scene::SceneNode* ResourceManager::loadAndCreateRenderAssetInstance(
    const AssetInfo& assetInfo,
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* visNodeCache) {
  if (!loadRenderAsset(assetInfo)) {
    return nullptr;
  }
  ASSERT(assetInfo.filepath == creation.filepath);

  // copy the const creation info to modify the key if necessary
  RenderAssetInstanceCreationInfo finalCreation(creation);
  if (assetInfo.overridePhongMaterial != Cr::Containers::NullOpt) {
    // material override is requested so get the id
    finalCreation.filepath =
        assetInfo.filepath + "?" +
        createColorMaterial(*assetInfo.overridePhongMaterial);
  }

  return createRenderAssetInstance(finalCreation, parent, drawables,
                                   visNodeCache);
}

bool ResourceManager::loadRenderAsset(const AssetInfo& info) {
  bool registerMaterialOverride =
      info.overridePhongMaterial != Cr::Containers::NullOpt;
  bool fileAssetIsLoaded = resourceDict_.count(info.filepath) > 0;

  bool meshSuccess = fileAssetIsLoaded;
  // first load the file asset as-is if necessary
  if (!fileAssetIsLoaded) {
    // clone the AssetInfo and remove the custom material to load a default
    // AssetInfo first
    AssetInfo defaultInfo(info);
    defaultInfo.overridePhongMaterial = Cr::Containers::NullOpt;

    if (info.type == AssetType::PRIMITIVE) {
      buildPrimitiveAssetData(info.filepath);
      meshSuccess = true;
    } else if (info.type == AssetType::FRL_PTEX_MESH) {
      meshSuccess = loadRenderAssetPTex(defaultInfo);
    } else if (info.type == AssetType::INSTANCE_MESH) {
      meshSuccess = loadRenderAssetIMesh(defaultInfo);
    } else if (isRenderAssetGeneral(info.type)) {
      meshSuccess = loadRenderAssetGeneral(defaultInfo);
    } else {
      // loadRenderAsset doesn't yet support the requested asset type
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    }

    if (meshSuccess) {
      // create and register the collisionMeshGroups
      if (info.type != AssetType::PRIMITIVE) {
        std::vector<CollisionMeshData> meshGroup;
        ASSERT(buildMeshGroups(defaultInfo, meshGroup));
      }

      if (gfxReplayRecorder_) {
        gfxReplayRecorder_->onLoadRenderAsset(defaultInfo);
      }
    }
  }

  // now handle loading the material override AssetInfo if configured
  if (meshSuccess && registerMaterialOverride) {
    // register or get the override material id
    std::string materialId = createColorMaterial(*info.overridePhongMaterial);

    // construct the unique id for the material modified asset
    std::string modifiedAssetName = info.filepath + "?" + materialId;
    const bool matModAssetIsRegistered =
        resourceDict_.count(modifiedAssetName) > 0;
    if (!matModAssetIsRegistered) {
      // first register the copied metaData
      resourceDict_.emplace(modifiedAssetName,
                            LoadedAssetData(resourceDict_.at(info.filepath)));
      // Replace the AssetInfo
      resourceDict_.at(modifiedAssetName).assetInfo = info;
      // Modify the MeshMetaData local material ids for all components
      std::vector<MeshTransformNode*> nodeQueue;
      nodeQueue.push_back(
          &resourceDict_.at(modifiedAssetName).meshMetaData.root);
      while (!nodeQueue.empty()) {
        MeshTransformNode* node = nodeQueue.back();
        nodeQueue.pop_back();
        for (auto& child : node->children) {
          nodeQueue.push_back(&child);
        }
        if (node->meshIDLocal != ID_UNDEFINED) {
          node->materialID = materialId;
        }
      }
      if (info.type != AssetType::PRIMITIVE) {
        // clone the collision data
        collisionMeshGroups_.emplace(modifiedAssetName,
                                     collisionMeshGroups_.at(info.filepath));
      }

      if (gfxReplayRecorder_) {
        gfxReplayRecorder_->onLoadRenderAsset(info);
      }
    }
  }

  return meshSuccess;
}

scene::SceneNode* ResourceManager::createRenderAssetInstance(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* visNodeCache) {
  CORRADE_ASSERT(resourceDict_.count(creation.filepath), "asset is not loaded",
                 nullptr);

  const LoadedAssetData& loadedAssetData = resourceDict_.at(creation.filepath);
  if (!isLightSetupCompatible(loadedAssetData, creation.lightSetupKey)) {
    LOG(WARNING)
        << "Instantiating render asset " << creation.filepath
        << " with incompatible light setup, instance will not be correctly lit."
           "For objects, please ensure 'requires lighting' is enabled in "
           "object config file.";
  }

  const auto& info = loadedAssetData.assetInfo;
  scene::SceneNode* newNode = nullptr;
  if (info.type == AssetType::FRL_PTEX_MESH) {
    CORRADE_ASSERT(!visNodeCache,
                   "createRenderAssetInstancePTex doesn't support this",
                   nullptr);
    newNode = createRenderAssetInstancePTex(creation, parent, drawables);
  } else if (info.type == AssetType::INSTANCE_MESH) {
    CORRADE_ASSERT(!visNodeCache,
                   "createRenderAssetInstanceIMesh doesn't support this",
                   nullptr);
    newNode = createRenderAssetInstanceIMesh(creation, parent, drawables);
  } else if (isRenderAssetGeneral(info.type) ||
             info.type == AssetType::PRIMITIVE) {
    newNode = createRenderAssetInstanceGeneralPrimitive(
        creation, parent, drawables, visNodeCache);
  } else {
    // createRenderAssetInstance doesn't yet support the requested asset type
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  if (gfxReplayRecorder_ && newNode) {
    gfxReplayRecorder_->onCreateRenderAssetInstance(newNode, creation);
  }

  return newNode;
}  // ResourceManager::createRenderAssetInstance

bool ResourceManager::loadStageInternal(
    const AssetInfo& info,
    const RenderAssetInstanceCreationInfo* creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
  // scene mesh loading
  const std::string& filename = info.filepath;
  LOG(INFO) << "ResourceManager::loadStageInternal : Attempting to load stage "
            << filename << " ";
  bool meshSuccess = true;
  if (info.filepath != EMPTY_SCENE) {
    if (!Cr::Utility::Directory::exists(filename)) {
      LOG(ERROR)
          << "ResourceManager::loadStageInternal : Cannot find scene file "
          << filename;
      meshSuccess = false;
    } else {
      if (info.type == AssetType::SUNCG_SCENE) {
        meshSuccess = loadSUNCGHouseFile(info, parent, drawables);
      } else {
        // load render asset if necessary
        if (!loadRenderAsset(info)) {
          return false;
        } else {
          if (resourceDict_[filename].assetInfo != info) {
            // TODO: support color material modified assets by changing the
            // "creation" filepath to the modified key

            // Right now, we only allow for an asset to be loaded with one
            // configuration, since generated mesh data may be invalid for a new
            // configuration
            LOG(ERROR)
                << "Reloading asset " << filename
                << " with different configuration not currently supported. "
                << "Asset may not be rendered correctly.";
          }
        }
        // create render asset instance if requested
        if (parent) {
          CORRADE_INTERNAL_ASSERT(creation);
          createRenderAssetInstance(*creation, parent, drawables);
        }
        return true;
      }
    }
  } else {
    LOG(INFO) << "ResourceManager::loadStageInternal : Loading empty scene for "
              << filename;
    // EMPTY_SCENE (ie. "NONE") string indicates desire for an empty scene (no
    // scene mesh): welcome to the void
  }

  return meshSuccess;

}  // ResourceManager::loadStageInternal

template <class T>
bool ResourceManager::buildStageCollisionMeshGroup(
    const std::string& filename,
    std::vector<CollisionMeshData>& meshGroup) {
  // TODO : refactor to manage any mesh groups, not just scene

  //! Collect collision mesh group
  const MeshMetaData& metaData = getMeshMetaData(filename);
  auto indexPair = metaData.meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;
  for (int mesh_i = start; mesh_i <= end; ++mesh_i) {
    T* rawMeshData = dynamic_cast<T*>(meshes_.at(mesh_i).get());
    if (rawMeshData == nullptr) {
      // means dynamic cast failed
      Cr::Utility::Debug()
          << "ResourceManager::buildStageCollisionMeshGroup : "
             "AssetInfo::AssetType "
             "type error: unsupported mesh type, aborting. Try running "
             "without \"--enable-physics\" and consider logging an issue.";
      return false;
    }
    CollisionMeshData& colMeshData = rawMeshData->getCollisionMeshData();
    meshGroup.push_back(colMeshData);
  }  // for each mesh

  return true;
}  // ResourceManager::buildStageCollisionMeshGroup

bool ResourceManager::loadObjectMeshDataFromFile(
    const std::string& filename,
    const metadata::attributes::ObjectAttributes::ptr& objectAttributes,
    const std::string& meshType,
    const bool requiresLighting) {
  bool success = false;
  if (!filename.empty()) {
    AssetInfo meshInfo{AssetType::UNKNOWN, filename};
    meshInfo.requiresLighting = requiresLighting;
    success = loadRenderAsset(meshInfo);
    if (!success) {
      LOG(ERROR) << "Failed to load a physical object ("
                 << objectAttributes->getHandle() << ")'s " << meshType
                 << " mesh from file : " << filename;
    }
  }
  return success;
}  // loadObjectMeshDataFromFile

Magnum::Range3D ResourceManager::computeMeshBB(BaseMesh* meshDataGL) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  return Mn::Math::minmax(meshData.positions);
}

#ifdef ESP_BUILD_PTEX_SUPPORT
void ResourceManager::computePTexMeshAbsoluteAABBs(
    BaseMesh& baseMesh,
    const std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  std::vector<Mn::Matrix4> absTransforms =
      computeAbsoluteTransformations(staticDrawableInfo);

  CORRADE_ASSERT(
      absTransforms.size() == staticDrawableInfo.size(),
      "ResourceManager::computePTexMeshAbsoluteAABBs: number of "
      "transformations does not match number of drawables. Aborting.", );

  // obtain the sub-meshes within the ptex mesh
  PTexMeshData& ptexMeshData = dynamic_cast<PTexMeshData&>(baseMesh);
  const std::vector<PTexMeshData::MeshData>& submeshes = ptexMeshData.meshes();

  for (uint32_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    // convert std::vector<vec3f> to std::vector<Mn::Vector3>
    const PTexMeshData::MeshData& submesh =
        submeshes[staticDrawableInfo[iEntry].meshID];
    std::vector<Mn::Vector3> pos{submesh.vbo.begin(), submesh.vbo.end()};

    // transform the vertex positions to the world space
    Mn::MeshTools::transformPointsInPlace(absTransforms[iEntry], pos);

    scene::SceneNode& node = staticDrawableInfo[iEntry].node;
    node.setAbsoluteAABB(Mn::Math::minmax(pos));
  }
}  // ResourceManager::computePTexMeshAbsoluteAABBs
#endif

void ResourceManager::computeGeneralMeshAbsoluteAABBs(
    const std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  std::vector<Mn::Matrix4> absTransforms =
      computeAbsoluteTransformations(staticDrawableInfo);

  CORRADE_ASSERT(absTransforms.size() == staticDrawableInfo.size(),
                 "ResourceManager::computeGeneralMeshAbsoluteAABBs: number of "
                 "transforms does not match number of drawables.", );

  for (uint32_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    const int meshID = staticDrawableInfo[iEntry].meshID;

    Cr::Containers::Optional<Magnum::Trade::MeshData>& meshData =
        meshes_.at(meshID)->getMeshData();
    CORRADE_ASSERT(meshData,
                   "ResourceManager::computeGeneralMeshAbsoluteAABBs: The mesh "
                   "data specified at ID:"
                       << meshID << "is empty/undefined. Aborting", );

    // a vector to store the min, max pos for the aabb of every position array
    std::vector<Mn::Vector3> bbPos;

    // transform the vertex positions to the world space, compute the aabb for
    // each position array
    for (uint32_t jArray = 0;
         jArray < meshData->attributeCount(Mn::Trade::MeshAttribute::Position);
         ++jArray) {
      Cr::Containers::Array<Mn::Vector3> pos =
          meshData->positions3DAsArray(jArray);
      Mn::MeshTools::transformPointsInPlace(absTransforms[iEntry], pos);

      std::pair<Mn::Vector3, Mn::Vector3> bb = Mn::Math::minmax(pos);
      bbPos.push_back(bb.first);
      bbPos.push_back(bb.second);
    }

    // locate the scene node which contains the current drawable
    scene::SceneNode& node = staticDrawableInfo[iEntry].node;

    // set the absolute axis aligned bounding box
    node.setAbsoluteAABB(Mn::Math::minmax(bbPos));

  }  // iEntry
}  // ResourceManager::computeGeneralMeshAbsoluteAABBs

void ResourceManager::computeInstanceMeshAbsoluteAABBs(
    const std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  std::vector<Mn::Matrix4> absTransforms =
      computeAbsoluteTransformations(staticDrawableInfo);

  CORRADE_ASSERT(
      absTransforms.size() == staticDrawableInfo.size(),
      "ResourceManager::computeInstancelMeshAbsoluteAABBs: Number of "
      "transforms does not match number of drawables. Aborting.", );

  for (size_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    const int meshID = staticDrawableInfo[iEntry].meshID;

    // convert std::vector<vec3f> to std::vector<Mn::Vector3>
    const std::vector<vec3f>& vertexPositions =
        dynamic_cast<GenericInstanceMeshData&>(*meshes_.at(meshID))
            .getVertexBufferObjectCPU();
    std::vector<Mn::Vector3> transformedPositions{vertexPositions.begin(),
                                                  vertexPositions.end()};

    Mn::MeshTools::transformPointsInPlace(absTransforms[iEntry],
                                          transformedPositions);

    scene::SceneNode& node = staticDrawableInfo[iEntry].node;
    node.setAbsoluteAABB(Mn::Math::minmax(transformedPositions));
  }  // iEntry
}

std::vector<Mn::Matrix4> ResourceManager::computeAbsoluteTransformations(
    const std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  // sanity check
  if (staticDrawableInfo.size() == 0) {
    return {};
  }

  // basic assumption is that all the drawables are in the same scene;
  // so use the 1st element in the vector to obtain this scene
  auto* scene = dynamic_cast<MagnumScene*>(staticDrawableInfo[0].node.scene());

  CORRADE_ASSERT(scene != nullptr,
                 "ResourceManager::computeAbsoluteTransformations: The node is "
                 "not attached to any scene graph. Aborting.",
                 {});

  // collect all drawable objects
  std::vector<std::reference_wrapper<MagnumObject>> objects;
  objects.reserve(staticDrawableInfo.size());
  std::transform(staticDrawableInfo.begin(), staticDrawableInfo.end(),
                 std::back_inserter(objects),
                 [](const StaticDrawableInfo& info) -> MagnumObject& {
                   return info.node;
                 });

  // compute transformations of all objects in the group relative to the root,
  // which are the absolute transformations
  std::vector<Mn::Matrix4> absTransforms =
      scene->transformationMatrices(objects);

  return absTransforms;
}

void ResourceManager::translateMesh(BaseMesh* meshDataGL,
                                    Magnum::Vector3 translation) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();

  Magnum::Matrix4 transform = Magnum::Matrix4::translation(translation);
  Magnum::MeshTools::transformPointsInPlace(transform, meshData.positions);
  // save the mesh transformation for future query
  meshDataGL->meshTransform_ = transform * meshDataGL->meshTransform_;

  meshDataGL->BB = meshDataGL->BB.translated(translation);
}  // ResourceManager::translateMesh

void ResourceManager::buildPrimitiveAssetData(
    const std::string& primTemplateHandle) {
  // retrieves -actual- template, not a copy
  const esp::metadata::attributes::AbstractPrimitiveAttributes::ptr
      primTemplate =
          getAssetAttributesManager()->getObjectByHandle(primTemplateHandle);
  // check if unique name of attributes describing primitive asset is present
  // already - don't remake if so
  auto primAssetHandle = primTemplate->getHandle();
  if (resourceDict_.count(primAssetHandle) > 0) {
    LOG(INFO) << " Primitive Asset exists already : " << primAssetHandle;
    return;
  }

  // class of primitive object
  std::string primClassName = primTemplate->getPrimObjClassName();
  // make sure it is open before use
  primitiveImporter_->openData("");
  // configuration for PrimitiveImporter - replace appropriate group's data
  // before instancing prim object
  Cr::Utility::ConfigurationGroup& conf = primitiveImporter_->configuration();
  Cr::Utility::ConfigurationGroup* cfgGroup = conf.group(primClassName);
  if (cfgGroup != nullptr) {  // ignore prims with no configuration like cubes
    auto newCfgGroup = primTemplate->getConfigGroup();
    // replace current conf group with passed attributes
    *cfgGroup = newCfgGroup;
  }

  // make assetInfo
  AssetInfo info{AssetType::PRIMITIVE};
  info.requiresLighting = true;
  // set up primitive mesh
  // make  primitive mesh structure
  auto primMeshData = std::make_unique<GenericMeshData>(false);
  // build mesh data object
  primMeshData->importAndSetMeshData(*primitiveImporter_, primClassName);

  // compute the mesh bounding box
  primMeshData->BB = computeMeshBB(primMeshData.get());

  primMeshData->uploadBuffersToGPU(false);

  // make MeshMetaData
  int meshStart = nextMeshID_++;
  int meshEnd = meshStart;
  MeshMetaData meshMetaData{meshStart, meshEnd};

  meshes_.emplace(meshStart, std::move(primMeshData));

  // default material for now
  std::unique_ptr<gfx::MaterialData> phongMaterial =
      gfx::PhongMaterialData::create_unique();

  shaderManager_.set(std::to_string(nextMaterialID_), phongMaterial.release());

  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;
  meshMetaData.root.materialID = std::to_string(nextMaterialID_++);
  // store the rotation to world frame upon load - currently superfluous
  const quatf transform = info.frame.rotationFrameToWorld();
  Magnum::Matrix4 R = Magnum::Matrix4::from(
      Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
  meshMetaData.root.transformFromLocalToParent =
      R * meshMetaData.root.transformFromLocalToParent;

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};
  auto inserted =
      resourceDict_.emplace(primAssetHandle, std::move(loadedAssetData));

  LOG(INFO) << " Primitive Asset Added : ID : " << primTemplate->getID()
            << " : attr lib key : " << primTemplate->getHandle()
            << " | instance class : " << primClassName
            << " | Conf has group for this obj type : "
            << conf.hasGroup(primClassName);

}  // ResourceManager::buildPrimitiveAssetData

bool ResourceManager::loadRenderAssetPTex(const AssetInfo& info) {
  ASSERT(info.type == AssetType::FRL_PTEX_MESH);

#ifdef ESP_BUILD_PTEX_SUPPORT
  // if this is a new file, load it and add it to the dictionary
  const std::string& filename = info.filepath;
  ASSERT(resourceDict_.count(filename) == 0);

  const auto atlasDir = Cr::Utility::Directory::join(
      Cr::Utility::Directory::path(filename), "textures");

  int index = nextMeshID_++;
  meshes_.emplace(index, std::make_unique<PTexMeshData>());

  auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_.at(index).get());
  pTexMeshData->load(filename, atlasDir);

  // update the dictionary
  auto inserted =
      resourceDict_.emplace(filename, LoadedAssetData{info, {index, index}});
  MeshMetaData& meshMetaData = inserted.first->second.meshMetaData;
  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;
  // store the rotation to world frame upon load
  const quatf transform = info.frame.rotationFrameToWorld();
  Magnum::Matrix4 R = Magnum::Matrix4::from(
      Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
  meshMetaData.root.transformFromLocalToParent =
      R * meshMetaData.root.transformFromLocalToParent;

  CORRADE_ASSERT(
      meshMetaData.meshIndex.first == meshMetaData.meshIndex.second,
      "ResourceManager::loadRenderAssetPTex: ptex mesh is not loaded "
      "correctly. Aborting.",
      false);

  return true;
#else
  LOG(ERROR) << "PTex support not enabled. Enable the BUILD_PTEX_SUPPORT CMake "
                "option when building.";
  return false;
#endif
}  // ResourceManager::loadRenderAssetPTex

scene::SceneNode* ResourceManager::createRenderAssetInstancePTex(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
#ifdef ESP_BUILD_PTEX_SUPPORT
  ASSERT(!creation.scale);                         // PTex doesn't support scale
  ASSERT(creation.lightSetupKey == NO_LIGHT_KEY);  // PTex doesn't support
                                                   // lighting

  const std::string& filename = creation.filepath;
  const LoadedAssetData& loadedAssetData = resourceDict_.at(creation.filepath);
  const MeshMetaData& metaData = getMeshMetaData(filename);
  const auto& info = loadedAssetData.assetInfo;
  auto indexPair = metaData.meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;
  std::vector<StaticDrawableInfo> staticDrawableInfo;

  scene::SceneNode* instanceRoot = &parent->createChild();

  for (int iMesh = start; iMesh <= end; ++iMesh) {
    auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_.at(iMesh).get());

    pTexMeshData->uploadBuffersToGPU(false);

    for (int jSubmesh = 0; jSubmesh < pTexMeshData->getSize(); ++jSubmesh) {
      scene::SceneNode& node = instanceRoot->createChild();
      const quatf transform = info.frame.rotationFrameToWorld();
      node.setRotation(Magnum::Quaternion(transform));

      node.addFeature<gfx::PTexMeshDrawable>(*pTexMeshData, jSubmesh,
                                             shaderManager_, drawables);

      staticDrawableInfo.emplace_back(StaticDrawableInfo{node, jSubmesh});
    }
  }
  // we assume a ptex mesh is only used as static
  ASSERT(creation.isStatic());
  ASSERT(metaData.meshIndex.first == metaData.meshIndex.second);

  computePTexMeshAbsoluteAABBs(*meshes_.at(metaData.meshIndex.first),
                               staticDrawableInfo);
  return instanceRoot;
#else
  LOG(ERROR) << "PTex support not enabled. Enable the BUILD_PTEX_SUPPORT CMake "
                "option when building.";
  return nullptr;
#endif
}  // ResourceManager::createRenderAssetInstancePTex

bool ResourceManager::loadRenderAssetIMesh(const AssetInfo& info) {
  ASSERT(info.type == AssetType::INSTANCE_MESH);

  const std::string& filename = info.filepath;
  ASSERT(resourceDict_.count(filename) == 0);
  Cr::Containers::Pointer<Importer> importer;
  CORRADE_INTERNAL_ASSERT_OUTPUT(
      importer = importerManager_.loadAndInstantiate("StanfordImporter"));

  std::vector<GenericInstanceMeshData::uptr> instanceMeshes;
  if (info.splitInstanceMesh) {
    instanceMeshes =
        GenericInstanceMeshData::fromPlySplitByObjectId(*importer, filename);
  } else {
    GenericInstanceMeshData::uptr meshData =
        GenericInstanceMeshData::fromPLY(*importer, filename);
    if (meshData)
      instanceMeshes.emplace_back(std::move(meshData));
  }

  if (instanceMeshes.empty()) {
    LOG(ERROR) << "Error loading instance mesh data";
    return false;
  }

  int meshStart = nextMeshID_;
  int meshEnd = meshStart + instanceMeshes.size() - 1;
  nextMeshID_ = meshEnd + 1;
  MeshMetaData meshMetaData{meshStart, meshEnd};
  meshMetaData.root.children.resize(instanceMeshes.size());

  for (int meshIDLocal = 0; meshIDLocal < instanceMeshes.size();
       ++meshIDLocal) {
    instanceMeshes[meshIDLocal]->uploadBuffersToGPU(false);
    meshes_.emplace(meshStart + meshIDLocal,
                    std::move(instanceMeshes[meshIDLocal]));

    meshMetaData.root.children[meshIDLocal].meshIDLocal = meshIDLocal;
  }

  // update the dictionary
  resourceDict_.emplace(filename,
                        LoadedAssetData{info, std::move(meshMetaData)});

  return true;
}  // ResourceManager::loadRenderAssetIMesh

scene::SceneNode* ResourceManager::createRenderAssetInstanceIMesh(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
  ASSERT(!creation.scale);  // IMesh doesn't support scale
  ASSERT(creation.lightSetupKey == NO_LIGHT_KEY);  // IMesh doesn't support
                                                   // lighting

  const bool computeAbsoluteAABBs = creation.isStatic();

  std::vector<StaticDrawableInfo> staticDrawableInfo;
  auto indexPair = getMeshMetaData(creation.filepath).meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;

  scene::SceneNode* instanceRoot = &parent->createChild();

  for (int iMesh = start; iMesh <= end; ++iMesh) {
    scene::SceneNode& node = instanceRoot->createChild();

    // Instance mesh does NOT have normal texture, so do not bother to
    // query if the mesh data contain tangent or bitangent.
    gfx::Drawable::Flags meshAttributeFlags{
        gfx::Drawable::Flag::HasVertexColor};
    // WARNING:
    // This is to initiate drawables for instance mesh, and the instance mesh
    // data is NOT stored in the meshData_ in the BaseMesh.
    // That means One CANNOT query the data like e.g.,
    // meshes_.at(iMesh)->getMeshData()->hasAttribute(Mn::Trade::MeshAttribute::Tangent)
    // It will SEGFAULT!
    createDrawable(*(meshes_.at(iMesh)->getMagnumGLMesh()),  // render mesh
                   meshAttributeFlags,                 // mesh attribute flags
                   node,                               // scene node
                   creation.lightSetupKey,             // lightSetup key
                   PER_VERTEX_OBJECT_ID_MATERIAL_KEY,  // material key
                   drawables);                         // drawable group

    if (computeAbsoluteAABBs) {
      staticDrawableInfo.emplace_back(StaticDrawableInfo{node, iMesh});
    }
  }

  if (computeAbsoluteAABBs) {
    computeInstanceMeshAbsoluteAABBs(staticDrawableInfo);
  }

  return instanceRoot;
}  // ResourceManager::createRenderAssetInstanceIMesh

bool ResourceManager::loadRenderAssetGeneral(const AssetInfo& info) {
  ASSERT(isRenderAssetGeneral(info.type));

  const std::string& filename = info.filepath;
  const std::string dispFileName = Cr::Utility::Directory::filename(filename);
  CHECK(resourceDict_.count(filename) == 0);

  // Preferred plugins, Basis target GPU format
  importerManager_.setPreferredPlugins("GltfImporter", {"TinyGltfImporter"});
#ifdef ESP_BUILD_ASSIMP_SUPPORT
  importerManager_.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
  Cr::PluginManager::PluginMetadata* const assimpmetadata =
      importerManager_.metadata("AssimpImporter");
  assimpmetadata->configuration().setValue("ImportColladaIgnoreUpDirection",
                                           "true");
#endif
  {
    Cr::PluginManager::PluginMetadata* const metadata =
        importerManager_.metadata("BasisImporter");
    Mn::GL::Context& context = Mn::GL::Context::current();
#ifdef MAGNUM_TARGET_WEBGL
    if (context.isExtensionSupported<
            Mn::GL::Extensions::WEBGL::compressed_texture_astc>())
#else
    if (context.isExtensionSupported<
            Mn::GL::Extensions::KHR::texture_compression_astc_ldr>())
#endif
    {
      LOG(INFO) << "Importing Basis files as ASTC 4x4 for " << dispFileName;
      metadata->configuration().setValue("format", "Astc4x4RGBA");
    }
#ifdef MAGNUM_TARGET_GLES
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_bptc>())
#else
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::ARB::texture_compression_bptc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as BC7 for " << dispFileName;
      metadata->configuration().setValue("format", "Bc7RGBA");
    }
#ifdef MAGNUM_TARGET_WEBGL
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::WEBGL::compressed_texture_s3tc>())
#elif defined(MAGNUM_TARGET_GLES)
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_s3tc>() ||
             context.isExtensionSupported<
                 Mn::GL::Extensions::ANGLE::texture_compression_dxt5>())
#else
    else if (context.isExtensionSupported<
                 Mn::GL::Extensions::EXT::texture_compression_s3tc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as BC3 for " << dispFileName;
      metadata->configuration().setValue("format", "Bc3RGBA");
    }
#ifndef MAGNUM_TARGET_GLES2
    else
#ifndef MAGNUM_TARGET_GLES
        if (context.isExtensionSupported<
                Mn::GL::Extensions::ARB::ES3_compatibility>())
#endif
    {
      LOG(INFO) << "Importing Basis files as ETC2 for " << dispFileName;
      metadata->configuration().setValue("format", "Etc2RGBA");
    }
#else /* For ES2, fall back to PVRTC as ETC2 is not available */
    else
#ifdef MAGNUM_TARGET_WEBGL
        if (context.isExtensionSupported<Mn::WEBGL::compressed_texture_pvrtc>())
#else
        if (context.isExtensionSupported<Mn::IMG::texture_compression_pvrtc>())
#endif
    {
      LOG(INFO) << "Importing Basis files as PVRTC 4bpp for " << dispFileName;
      metadata->configuration().setValue("format", "PvrtcRGBA4bpp");
    }
#endif
#if defined(MAGNUM_TARGET_GLES2) || !defined(MAGNUM_TARGET_GLES)
    else /* ES3 has ETC2 always */
    {
      LOG(WARNING) << "No supported GPU compressed texture format detected, "
                      "Basis images will get imported as RGBA8 for "
                   << dispFileName;
      metadata->configuration().setValue("format", "RGBA8");
    }
#endif
  }

  if (!fileImporter_->openFile(filename)) {
    LOG(ERROR) << "Cannot open file " << filename;
    return false;
  }

  // load file and add it to the dictionary
  LoadedAssetData loadedAssetData{info};
  if (requiresTextures_) {
    loadTextures(*fileImporter_, loadedAssetData);
    loadMaterials(*fileImporter_, loadedAssetData);
  }
  loadMeshes(*fileImporter_, loadedAssetData);
  auto inserted = resourceDict_.emplace(filename, std::move(loadedAssetData));
  MeshMetaData& meshMetaData = inserted.first->second.meshMetaData;

  // Register magnum mesh
  if (fileImporter_->defaultScene() != -1) {
    Cr::Containers::Optional<Magnum::Trade::SceneData> sceneData =
        fileImporter_->scene(fileImporter_->defaultScene());
    if (!sceneData) {
      LOG(ERROR) << "Cannot load scene, exiting";
      return false;
    }
    for (unsigned int sceneDataID : sceneData->children3D()) {
      loadMeshHierarchy(*fileImporter_, meshMetaData.root, sceneDataID);
    }
  } else if ((fileImporter_->meshCount() != 0u) &&
             meshes_.at(meshMetaData.meshIndex.first)) {
    // no default scene --- standalone OBJ/PLY files, for example
    // take a wild guess and load the first mesh with the first material
    // addMeshToDrawables(metaData, *parent, drawables, 0, 0);
    loadMeshHierarchy(*fileImporter_, meshMetaData.root, 0);
  } else {
    LOG(ERROR) << "No default scene available and no meshes found, exiting";
    return false;
  }

  const quatf transform = info.frame.rotationFrameToWorld();
  Magnum::Matrix4 R = Magnum::Matrix4::from(
      Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
  meshMetaData.root.transformFromLocalToParent =
      R * meshMetaData.root.transformFromLocalToParent;

  return true;
}  // ResourceManager::loadRenderAssetGeneral

scene::SceneNode* ResourceManager::createRenderAssetInstanceGeneralPrimitive(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* userVisNodeCache) {
  ASSERT(parent);
  ASSERT(drawables);

  CHECK(resourceDict_.count(creation.filepath));
  const LoadedAssetData& loadedAssetData = resourceDict_.at(creation.filepath);

  std::vector<scene::SceneNode*> dummyVisNodeCache;
  auto& visNodeCache = userVisNodeCache ? *userVisNodeCache : dummyVisNodeCache;

  scene::SceneNode& newNode = parent->createChild();
  if (creation.scale) {
    // need a new node for scaling because motion state will override scale
    // set at the physical node
    // perf todo: avoid this if unit scale
    newNode.setScaling(*creation.scale);

    // legacy quirky behavior: only add this node to viscache if using scaling
    visNodeCache.push_back(&newNode);
  }

  std::vector<StaticDrawableInfo> staticDrawableInfo;

  auto nodeType = creation.isStatic() ? scene::SceneNodeType::EMPTY
                                      : scene::SceneNodeType::OBJECT;
  bool computeAbsoluteAABBs = creation.isStatic();

  addComponent(loadedAssetData.meshMetaData,       // mesh metadata
               newNode,                            // parent scene node
               creation.lightSetupKey,             // lightSetup key
               drawables,                          // drawable group
               loadedAssetData.meshMetaData.root,  // mesh transform node
               visNodeCache,  // a vector of scene nodes, the visNodeCache
               computeAbsoluteAABBs,  // compute absolute AABBs
               staticDrawableInfo);   // a vector of static drawable info

  if (computeAbsoluteAABBs) {
    // now compute aabbs by constructed staticDrawableInfo
    computeGeneralMeshAbsoluteAABBs(staticDrawableInfo);
  }

  // set the node type for all cached visual nodes
  if (nodeType != scene::SceneNodeType::EMPTY) {
    for (auto* node : visNodeCache) {
      node->setType(nodeType);
    }
  }

  return &newNode;
}  // ResourceManager::createRenderAssetInstanceGeneralPrimitive

bool ResourceManager::buildTrajectoryVisualization(
    const std::string& trajVisName,
    const std::vector<Mn::Vector3>& pts,
    int numSegments,
    float radius,
    const Magnum::Color4& color,
    bool smooth,
    int numInterp) {
  // enforce required minimum/reasonable values if illegal values specified
  if (numSegments < 3) {  // required by circle prim
    numSegments = 3;
  }
  // clip to 10 points between trajectory points, if illegal value
  if (smooth && (numInterp <= 0)) {
    numInterp = 10;
  }
  // 1 millimeter radius minimum
  if (radius <= 0) {
    radius = .001;
  }

  LOG(INFO) << "ResourceManager::loadTrajectoryVisualization : Calling "
               "trajectoryTubeSolid to build a tube named :"
            << trajVisName << " with " << pts.size()
            << " points, building a tube of radius :" << radius << " using "
            << numSegments << " circular segments and " << numInterp
            << " interpolated points between each trajectory point.";

  // create mesh tube
  Cr::Containers::Optional<Mn::Trade::MeshData> trajTubeMesh =
      geo::buildTrajectoryTubeSolid(pts, numSegments, radius, smooth,
                                    numInterp);
  LOG(INFO) << "ResourceManager::loadTrajectoryVisualization : Successfully "
               "returned from trajectoryTubeSolid ";

  // make assetInfo
  AssetInfo info{AssetType::PRIMITIVE};
  info.requiresLighting = true;
  // set up primitive mesh
  // make  primitive mesh structure
  auto visMeshData = std::make_unique<GenericMeshData>(false);
  visMeshData->setMeshData(*std::move(trajTubeMesh));
  // compute the mesh bounding box
  visMeshData->BB = computeMeshBB(visMeshData.get());

  visMeshData->uploadBuffersToGPU(false);

  // make MeshMetaData
  int meshStart = meshes_.size();
  int meshEnd = meshStart;
  MeshMetaData meshMetaData{meshStart, meshEnd};

  meshes_.emplace(meshStart, std::move(visMeshData));

  // default material for now
  auto phongMaterial = gfx::PhongMaterialData::create_unique();
  phongMaterial->specularColor = {1.0, 1.0, 1.0, 1.0};
  phongMaterial->ambientColor = color;
  phongMaterial->diffuseColor = color;

  shaderManager_.set(std::to_string(nextMaterialID_),
                     static_cast<gfx::MaterialData*>(phongMaterial.release()));

  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;
  meshMetaData.root.materialID = std::to_string(nextMaterialID_++);

  // store the rotation to world frame upon load - currently superfluous
  const quatf transform = info.frame.rotationFrameToWorld();
  Magnum::Matrix4 R = Magnum::Matrix4::from(
      Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
  meshMetaData.root.transformFromLocalToParent =
      R * meshMetaData.root.transformFromLocalToParent;

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};
  // TODO : need to free render assets associated with this object if collision
  // occurs, otherwise leak! (Currently unsupported).
  // if (resourceDict_.count(trajVisName) != 0) {
  //   resourceDict_.erase(trajVisName);
  // }
  auto inserted =
      resourceDict_.emplace(trajVisName, std::move(loadedAssetData));

  return true;
}  // ResourceManager::loadTrajectoryVisualization

int ResourceManager::loadNavMeshVisualization(esp::nav::PathFinder& pathFinder,
                                              scene::SceneNode* parent,
                                              DrawableGroup* drawables) {
  int navMeshPrimitiveID = ID_UNDEFINED;

  if (!pathFinder.isLoaded())
    return navMeshPrimitiveID;

  // create the mesh
  std::vector<Magnum::UnsignedInt> indices;
  std::vector<Magnum::Vector3> positions;

  const MeshData::ptr navMeshData = pathFinder.getNavMeshData();

  // add the vertices
  positions.resize(navMeshData->vbo.size());
  for (size_t vix = 0; vix < navMeshData->vbo.size(); vix++) {
    positions[vix] = Magnum::Vector3{navMeshData->vbo[vix]};
  }

  indices.resize(navMeshData->ibo.size() * 2);
  for (size_t ix = 0; ix < navMeshData->ibo.size();
       ix += 3) {  // for each triangle, create lines
    size_t nix = ix * 2;
    indices[nix] = navMeshData->ibo[ix];
    indices[nix + 1] = navMeshData->ibo[ix + 1];
    indices[nix + 2] = navMeshData->ibo[ix + 1];
    indices[nix + 3] = navMeshData->ibo[ix + 2];
    indices[nix + 4] = navMeshData->ibo[ix + 2];
    indices[nix + 5] = navMeshData->ibo[ix];
  }

  // create a temporary mesh object referencing the above data
  Mn::Trade::MeshData visualNavMesh{
      Mn::MeshPrimitive::Lines,
      {},
      indices,
      Mn::Trade::MeshIndexData{indices},
      {},
      positions,
      {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                    Cr::Containers::arrayView(positions)}}};

  // compile and add the new mesh to the structure
  navMeshPrimitiveID = nextPrimitiveMeshId;
  primitive_meshes_[nextPrimitiveMeshId++] = std::make_unique<Magnum::GL::Mesh>(
      Magnum::MeshTools::compile(visualNavMesh));

  if (parent != nullptr && drawables != nullptr &&
      navMeshPrimitiveID != ID_UNDEFINED) {
    // create the drawable
    addPrimitiveToDrawables(navMeshPrimitiveID, *parent, drawables);
  }

  return navMeshPrimitiveID;
}  // ResourceManager::loadNavMeshVisualization

void ResourceManager::loadMaterials(Importer& importer,
                                    LoadedAssetData& loadedAssetData) {
  for (int iMaterial = 0; iMaterial < importer.materialCount(); ++iMaterial) {
    int currentMaterialID = nextMaterialID_++;

    // TODO:
    // it seems we have a way to just load the material once in this case,
    // as long as the materialName includes the full path to the material
    Cr::Containers::Optional<Mn::Trade::MaterialData> materialData =
        importer.material(iMaterial);

    if (!materialData) {
      LOG(ERROR) << "Cannot load material, skipping";
      continue;
    }

    std::unique_ptr<gfx::MaterialData> finalMaterial;
    int textureBaseIndex = loadedAssetData.meshMetaData.textureIndex.first;

    if (loadedAssetData.assetInfo.requiresLighting &&
        materialData->types() &
            Magnum::Trade::MaterialType::PbrMetallicRoughness) {
      const auto& pbrMaterialData =
          materialData->as<Mn::Trade::PbrMetallicRoughnessMaterialData>();

      if (flags_ & Flag::BuildPhongFromPbr) {
        finalMaterial = gfx::buildPhongFromPbrMetallicRoughness(
            pbrMaterialData, textureBaseIndex, textures_);
      } else {
        finalMaterial =
            buildPbrShadedMaterialData(pbrMaterialData, textureBaseIndex);
      }
    } else {
      ASSERT(materialData);
      if (!(materialData->types() & Magnum::Trade::MaterialType::Phong)) {
        LOG(ERROR) << "Cannot load material, skipping";
        continue;
      }

      const auto& phongMaterialData =
          materialData->as<Mn::Trade::PhongMaterialData>();
      if (loadedAssetData.assetInfo.requiresLighting) {
        finalMaterial =
            buildPhongShadedMaterialData(phongMaterialData, textureBaseIndex);

      } else {
        finalMaterial =
            buildFlatShadedMaterialData(phongMaterialData, textureBaseIndex);
      }
    }
    // for now, just use unique ID for material key. This may change if we
    // expose materials to user for post-load modification
    shaderManager_.set(std::to_string(currentMaterialID),
                       finalMaterial.release());
  }
}  // ResourceManager::loadMaterials

gfx::PhongMaterialData::uptr ResourceManager::buildFlatShadedMaterialData(
    const Mn::Trade::PhongMaterialData& material,
    int textureBaseIndex) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  auto finalMaterial = gfx::PhongMaterialData::create_unique();
  finalMaterial->ambientColor = 0xffffffff_rgbaf;
  finalMaterial->diffuseColor = 0x00000000_rgbaf;
  finalMaterial->specularColor = 0x00000000_rgbaf;

  if (material.hasAttribute(Mn::Trade::MaterialAttribute::AmbientTexture)) {
    finalMaterial->ambientTexture =
        textures_.at(textureBaseIndex + material.ambientTexture()).get();
  } else if (material.hasAttribute(
                 Mn::Trade::MaterialAttribute::DiffuseTexture)) {
    // if we want to force flat shading, but we don't have ambient texture,
    // check for diffuse texture and use that instead
    finalMaterial->ambientTexture =
        textures_.at(textureBaseIndex + material.diffuseTexture()).get();
  }

  if (material.hasAttribute(Mn::Trade::MaterialAttribute::DiffuseColor)) {
    finalMaterial->ambientColor = material.diffuseColor();
  } else if (material.hasAttribute(
                 Mn::Trade::MaterialAttribute::AmbientColor)) {
    finalMaterial->ambientColor = material.ambientColor();
  }

  return finalMaterial;
}

gfx::PhongMaterialData::uptr ResourceManager::buildPhongShadedMaterialData(
    const Mn::Trade::PhongMaterialData& material,
    int textureBaseIndex) const {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  auto finalMaterial = gfx::PhongMaterialData::create_unique();
  finalMaterial->shininess = material.shininess();

  // texture transform, if there's none the matrix is an identity
  finalMaterial->textureMatrix = material.commonTextureMatrix();

  // ambient material properties
  finalMaterial->ambientColor = material.ambientColor();
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::AmbientTexture)) {
    finalMaterial->ambientTexture =
        textures_.at(textureBaseIndex + material.ambientTexture()).get();
  }

  // diffuse material properties
  finalMaterial->diffuseColor = material.diffuseColor();
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::DiffuseTexture)) {
    finalMaterial->diffuseTexture =
        textures_.at(textureBaseIndex + material.diffuseTexture()).get();
  }

  // specular material properties
  finalMaterial->specularColor = material.specularColor();
  if (material.hasSpecularTexture()) {
    finalMaterial->specularTexture =
        textures_.at(textureBaseIndex + material.specularTexture()).get();
  }

  // normal mapping
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::NormalTexture)) {
    finalMaterial->normalTexture =
        textures_.at(textureBaseIndex + material.normalTexture()).get();
  }
  return finalMaterial;
}

gfx::PbrMaterialData::uptr ResourceManager::buildPbrShadedMaterialData(
    const Mn::Trade::PbrMetallicRoughnessMaterialData& material,
    int textureBaseIndex) const {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  auto finalMaterial = gfx::PbrMaterialData::create_unique();

  // texture transform, if there's none the matrix is an identity
  finalMaterial->textureMatrix = material.commonTextureMatrix();

  // base color (albedo)
  finalMaterial->baseColor = material.baseColor();
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::BaseColorTexture)) {
    finalMaterial->baseColorTexture =
        textures_.at(textureBaseIndex + material.baseColorTexture()).get();
  }

  // normal map
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::NormalTexture)) {
    // must be inside the if clause otherwise assertion fails if no normal
    // texture is presented
    finalMaterial->normalTextureScale = material.normalTextureScale();

    finalMaterial->normalTexture =
        textures_.at(textureBaseIndex + material.normalTexture()).get();
  }

  // emission
  finalMaterial->emissiveColor = material.emissiveColor();
  if (material.hasAttribute(Mn::Trade::MaterialAttribute::EmissiveTexture)) {
    finalMaterial->emissiveTexture =
        textures_.at(textureBaseIndex + material.emissiveTexture()).get();
    if (!material.hasAttribute(Mn::Trade::MaterialAttribute::EmissiveColor)) {
      finalMaterial->emissiveColor = Mn::Vector3{1.0f};
    }
  }

  // roughness
  finalMaterial->roughness = material.roughness();
  if (material.hasRoughnessTexture()) {
    finalMaterial->roughnessTexture =
        textures_.at(textureBaseIndex + material.roughnessTexture()).get();
  }

  // metallic
  finalMaterial->metallic = material.metalness();
  if (material.hasMetalnessTexture()) {
    finalMaterial->metallicTexture =
        textures_.at(textureBaseIndex + material.metalnessTexture()).get();
  }

  // sanity check when both metallic and roughness materials are presented
  if (material.hasMetalnessTexture() && material.hasRoughnessTexture()) {
    /*
       sanity check using hasNoneRoughnessMetallicTexture() to ensure that:
        - both use the same texture coordinate attribute,
        - both have the same texture transformation, and
        - the metalness is in B and roughness is in G

       It checks for a subset of hasOcclusionRoughnessMetallicTexture(),
       so hasOcclusionRoughnessMetallicTexture() is not needed here.

       The normal/roughness/metallic is a totally different packing (in BA
       instead of GB), and it is NOT supported in the current version.
       so hasNormalRoughnessMetallicTexture() is not needed here.

    */
    CORRADE_ASSERT(
        material.hasNoneRoughnessMetallicTexture(),
        "ResourceManager::buildPbrShadedMaterialData(): if both the metallic "
        "and roughness texture exist, they must be packed in the same texture "
        "based on glTF 2.0 Spec.",
        finalMaterial);
  }

  // TODO:
  // Support NormalRoughnessMetallicTexture packing
  CORRADE_ASSERT(!material.hasNormalRoughnessMetallicTexture(),
                 "ResourceManager::buildPbrShadedMaterialData(): "
                 "Sorry. NormalRoughnessMetallicTexture is not supported in "
                 "the current version. We will work on it.",
                 finalMaterial);

  // double-sided
  finalMaterial->doubleSided = material.isDoubleSided();

  return finalMaterial;
}

void ResourceManager::loadMeshes(Importer& importer,
                                 LoadedAssetData& loadedAssetData) {
  int meshStart = nextMeshID_;
  int meshEnd = meshStart + importer.meshCount() - 1;
  nextMeshID_ = meshEnd + 1;
  loadedAssetData.meshMetaData.setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.meshCount(); ++iMesh) {
    // don't need normals if we aren't using lighting
    auto gltfMeshData = std::make_unique<GenericMeshData>(
        loadedAssetData.assetInfo.requiresLighting);
    gltfMeshData->importAndSetMeshData(importer, iMesh);

    // compute the mesh bounding box
    gltfMeshData->BB = computeMeshBB(gltfMeshData.get());

    gltfMeshData->uploadBuffersToGPU(false);
    meshes_.emplace(meshStart + iMesh, std::move(gltfMeshData));
  }
}  // ResourceManager::loadMeshes

//! Recursively load the transformation chain specified by the mesh file
void ResourceManager::loadMeshHierarchy(Importer& importer,
                                        MeshTransformNode& parent,
                                        int componentID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(componentID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(componentID)
               << ", skipping";
    return;
  }

  // Add the new node to the hierarchy and set its transformation
  parent.children.emplace_back();
  parent.children.back().transformFromLocalToParent =
      objectData->transformation();
  parent.children.back().componentID = componentID;

  const int meshIDLocal = objectData->instance();

  // Add a mesh index
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh &&
      meshIDLocal != ID_UNDEFINED) {
    parent.children.back().meshIDLocal = meshIDLocal;
    if (requiresTextures_) {
      auto* mod3D =
          static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get());
      if (mod3D->material() != ID_UNDEFINED) {
        // we've already loaded the materials, so we can get the global index
        // from the material count
        parent.children.back().materialID = std::to_string(
            mod3D->material() + nextMaterialID_ - importer.materialCount());
      }
    }
  }

  // Recursively add children
  for (auto childObjectID : objectData->children()) {
    loadMeshHierarchy(importer, parent.children.back(), childObjectID);
  }
}  // ResourceManager::loadMeshHierarchy

void ResourceManager::loadTextures(Importer& importer,
                                   LoadedAssetData& loadedAssetData) {
  int textureStart = nextTextureID_;
  int textureEnd = textureStart + importer.textureCount() - 1;
  nextTextureID_ = textureEnd + 1;
  loadedAssetData.meshMetaData.setTextureIndices(textureStart, textureEnd);

  for (int iTexture = 0; iTexture < importer.textureCount(); ++iTexture) {
    auto currentTextureID = textureStart + iTexture;
    textures_.emplace(currentTextureID,
                      std::make_shared<Magnum::GL::Texture2D>());
    auto& currentTexture = textures_.at(currentTextureID);

    auto textureData = importer.texture(iTexture);
    if (!textureData ||
        textureData->type() != Magnum::Trade::TextureData::Type::Texture2D) {
      LOG(ERROR) << "Cannot load texture " << iTexture << " skipping";
      currentTexture = nullptr;
      continue;
    }

    // Configure the texture
    Mn::GL::Texture2D& texture = *(textures_.at(textureStart + iTexture).get());
    texture.setMagnificationFilter(textureData->magnificationFilter())
        .setMinificationFilter(textureData->minificationFilter(),
                               textureData->mipmapFilter())
        .setWrapping(textureData->wrapping().xy());

    // Load all mip levels
    const std::uint32_t levelCount =
        importer.image2DLevelCount(textureData->image());
    bool generateMipmap = false;
    for (std::uint32_t level = 0; level != levelCount; ++level) {
      // TODO:
      // it seems we have a way to just load the image once in this case,
      // as long as the image2DName include the full path to the image
      Cr::Containers::Optional<Mn::Trade::ImageData2D> image =
          importer.image2D(textureData->image(), level);
      if (!image) {
        LOG(ERROR) << "Cannot load texture image, skipping";
        currentTexture = nullptr;
        break;
      }

      Mn::GL::TextureFormat format;
      if (image->isCompressed()) {
        format = Mn::GL::textureFormat(image->compressedFormat());
      } else {
        format = Mn::GL::textureFormat(image->format());
      }

      // For the very first level, allocate the texture
      if (level == 0) {
        // If there is just one level and the image is not compressed, we'll
        // generate mips ourselves
        if (levelCount == 1 && !image->isCompressed()) {
          texture.setStorage(Mn::Math::log2(image->size().max()) + 1, format,
                             image->size());
          generateMipmap = true;
        } else
          texture.setStorage(levelCount, format, image->size());
      }

      if (image->isCompressed())
        texture.setCompressedSubImage(level, {}, *image);
      else
        texture.setSubImage(level, {}, *image);
    }

    // Mip level loading failed, fail the whole texture
    if (currentTexture == nullptr)
      continue;

    // Generate a mipmap if requested
    if (generateMipmap)
      texture.generateMipmap();
  }
}  // ResourceManager::loadTextures

bool ResourceManager::instantiateAssetsOnDemand(
    const metadata::attributes::ObjectAttributes::ptr& objectAttributes) {
  if (!objectAttributes) {
    return false;
  }
  const std::string& objectTemplateHandle = objectAttributes->getHandle();

  // if attributes are "dirty" (important values have changed since last
  // registered) then re-register.  Should never return ID_UNDEFINED - this
  // would mean something has corrupted the library.
  // NOTE : this is called when an new object is being made, but before the
  // object has acquired a copy of its parent attributes.  No object should
  // ever have a copy of attributes with isDirty == true - any editing of
  // attributes for objects requires object rebuilding.
  if (objectAttributes->getIsDirty()) {
    CORRADE_ASSERT(
        (ID_UNDEFINED != getObjectAttributesManager()->registerObject(
                             objectAttributes, objectTemplateHandle)),
        "ResourceManager::instantiateAssetsOnDemand : Unknown failure "
        "attempting to register modified template :"
            << objectTemplateHandle
            << "before asset instantiation.  Aborting. ",
        false);
  }

  // get render asset handle
  std::string renderAssetHandle = objectAttributes->getRenderAssetHandle();
  // whether attributes requires lighting
  bool requiresLighting = objectAttributes->getRequiresLighting();
  bool renderMeshSuccess = false;
  // no resource dict entry exists for renderAssetHandle
  if (resourceDict_.count(renderAssetHandle) == 0) {
    if (objectAttributes->getRenderAssetIsPrimitive()) {
      // needs to have a primitive asset attributes with same name
      if (!getAssetAttributesManager()->getObjectLibHasHandle(
              renderAssetHandle)) {
        // this is bad, means no render primitive template exists with
        // expected name.  should never happen
        LOG(ERROR) << "No primitive asset attributes exists with name :"
                   << renderAssetHandle
                   << " so unable to instantiate primitive-based render "
                      "object.  Aborting.";
        return false;
      }
      // build primitive asset for this object based on defined primitive
      // attributes
      buildPrimitiveAssetData(renderAssetHandle);

    } else {
      // load/check_for render mesh metadata and load assets
      renderMeshSuccess = loadObjectMeshDataFromFile(
          renderAssetHandle, objectAttributes, "render", requiresLighting);
    }
  }  // if no render asset exists

  // check if uses collision mesh
  // TODO : handle visualization-only objects lacking collision assets
  //        Probably just need to check attr->isCollidable()
  if (!objectAttributes->getCollisionAssetIsPrimitive()) {
    const auto collisionAssetHandle =
        objectAttributes->getCollisionAssetHandle();
    if (resourceDict_.count(collisionAssetHandle) == 0) {
      bool collisionMeshSuccess = loadObjectMeshDataFromFile(
          collisionAssetHandle, objectAttributes, "collision",
          !renderMeshSuccess && requiresLighting);

      if (!collisionMeshSuccess) {
        return false;
      }
    }
    // check if collision handle exists in collision mesh groups yet.  if not
    // then instance
    if (collisionMeshGroups_.count(collisionAssetHandle) == 0) {
      // set collision mesh data
      const MeshMetaData& meshMetaData = getMeshMetaData(collisionAssetHandle);

      int start = meshMetaData.meshIndex.first;
      int end = meshMetaData.meshIndex.second;
      //! Gather mesh components for meshGroup data
      std::vector<CollisionMeshData> meshGroup;
      for (int mesh_i = start; mesh_i <= end; ++mesh_i) {
        GenericMeshData& gltfMeshData =
            dynamic_cast<GenericMeshData&>(*meshes_.at(mesh_i).get());
        CollisionMeshData& meshData = gltfMeshData.getCollisionMeshData();
        meshGroup.push_back(meshData);
      }
      collisionMeshGroups_.emplace(collisionAssetHandle, meshGroup);
    }
  }

  return true;
}  // ResourceManager::instantiateAssetsOnDemand

bool ResourceManager::importURDFAssets(io::URDF::Model& model) {
  bool importSuccess = true;
  for (size_t linkIx = 0; linkIx < model.m_links.size(); ++linkIx) {
    auto link = model.getLink(linkIx);
    // load collision shapes
    for (auto& collision : link->m_collisionArray) {
      if (collision.m_geometry.m_type == io::URDF::GEOM_MESH) {
        // pre-load the mesh asset for its collision shape
        assets::AssetInfo meshAsset{assets::AssetType::UNKNOWN,
                                    collision.m_geometry.m_meshFileName};
        importSuccess = loadRenderAsset(meshAsset);
      }
      if (!importSuccess) {
        break;
      }
    }
    if (!importSuccess) {
      break;
    }
    // pre-load visual meshes and primitive asset variations and cache the
    // handle
    for (auto& visual : link->m_visualArray) {
      assets::AssetInfo visualMeshInfo{assets::AssetType::UNKNOWN};
      visualMeshInfo.requiresLighting = true;

      std::shared_ptr<io::URDF::Material> material =
          visual.m_geometry.m_localMaterial;
      if (material) {
        visualMeshInfo.overridePhongMaterial = assets::PhongMaterialColor();
        visualMeshInfo.overridePhongMaterial->ambientColor =
            material->m_matColor.m_rgbaColor;
        visualMeshInfo.overridePhongMaterial->diffuseColor =
            material->m_matColor.m_rgbaColor;
        visualMeshInfo.overridePhongMaterial->specularColor =
            Mn::Color4(material->m_matColor.m_specularColor);
      }
      switch (visual.m_geometry.m_type) {
        case io::URDF::GEOM_CAPSULE: {
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          auto assetMgr = getAssetAttributesManager();
          auto capTemplate = assetMgr->getDefaultCapsuleTemplate(false);
          // proportions as suggested on magnum docs
          capTemplate->setHalfLength(0.5 * visual.m_geometry.m_capsuleHeight /
                                     visual.m_geometry.m_capsuleRadius);
          assetMgr->registerObject(capTemplate);
          // cache the new capsule asset handle for later instancing
          visual.m_geometry.m_meshFileName = capTemplate->getHandle();
        } break;
        case io::URDF::GEOM_CYLINDER:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath =
              "cylinderSolid_rings_1_segments_12_halfLen_1_useTexCoords_false_"
              "useTangents_false_capEnds_true";
          break;
        case io::URDF::GEOM_BOX:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath = "cubeSolid";
          break;
        case io::URDF::GEOM_SPHERE:
          visualMeshInfo.type = esp::assets::AssetType::PRIMITIVE;
          visualMeshInfo.filepath = "icosphereSolid_subdivs_1";
          break;
        case io::URDF::GEOM_MESH:
          visualMeshInfo.filepath = visual.m_geometry.m_meshFileName;
          break;
        default:
          Mn::Debug{} << "ResourceManager::importURDFAssets - unsupported "
                         "visual geometry type.";
          break;
      }
    }
  }
  return importSuccess;
}

void ResourceManager::addObjectToDrawables(
    const ObjectAttributes::ptr& ObjectAttributes,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>& visNodeCache,
    const std::string& lightSetupKey) {
  if (parent != nullptr and drawables != nullptr) {
    //! Add mesh to rendering stack

    const std::string& renderObjectName =
        ObjectAttributes->getRenderAssetHandle();

    RenderAssetInstanceCreationInfo::Flags flags;
    flags |= RenderAssetInstanceCreationInfo::Flag::IsRGBD;
    flags |= RenderAssetInstanceCreationInfo::Flag::IsSemantic;
    RenderAssetInstanceCreationInfo creation(
        renderObjectName, ObjectAttributes->getScale(), flags, lightSetupKey);

    createRenderAssetInstance(creation, parent, drawables, &visNodeCache);

  }  // should always be specified, otherwise won't do anything
}  // addObjectToDrawables

//! Add component to rendering stack, based on importer loading
void ResourceManager::addComponent(
    const MeshMetaData& metaData,
    scene::SceneNode& parent,
    const Mn::ResourceKey& lightSetupKey,
    DrawableGroup* drawables,
    const MeshTransformNode& meshTransformNode,
    std::vector<scene::SceneNode*>& visNodeCache,
    bool computeAbsoluteAABBs,
    std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  // Add the object to the scene and set its transformation
  scene::SceneNode& node = parent.createChild();
  visNodeCache.push_back(&node);
  node.MagnumObject::setTransformation(
      meshTransformNode.transformFromLocalToParent);

  const int meshIDLocal = meshTransformNode.meshIDLocal;

  // Add a drawable if the object has a mesh and the mesh is loaded
  if (meshIDLocal != ID_UNDEFINED) {
    const int meshID = metaData.meshIndex.first + meshIDLocal;
    Magnum::GL::Mesh& mesh = *meshes_.at(meshID)->getMagnumGLMesh();
    Mn::ResourceKey materialKey = meshTransformNode.materialID;

    gfx::Drawable::Flags meshAttributeFlags{};
    const auto& meshData = meshes_.at(meshID)->getMeshData();
    if (meshData != Cr::Containers::NullOpt) {
      if (meshData->hasAttribute(Mn::Trade::MeshAttribute::Tangent)) {
        meshAttributeFlags |= gfx::Drawable::Flag::HasTangent;

        // if it has tangent, then check if it has bitangent
        if (meshData->hasAttribute(Mn::Trade::MeshAttribute::Bitangent)) {
          meshAttributeFlags |= gfx::Drawable::Flag::HasSeparateBitangent;
        }
      }

      if (meshData->hasAttribute(Mn::Trade::MeshAttribute::Color)) {
        meshAttributeFlags |= gfx::Drawable::Flag::HasVertexColor;
      }
    }

    createDrawable(mesh,                // render mesh
                   meshAttributeFlags,  // mesh attribute flags
                   node,                // scene node
                   lightSetupKey,       // lightSetup Key
                   materialKey,         // material key
                   drawables);          // drawable group

    // compute the bounding box for the mesh we are adding
    if (computeAbsoluteAABBs) {
      staticDrawableInfo.emplace_back(StaticDrawableInfo{node, meshID});
    }
    BaseMesh* meshBB = meshes_.at(meshID).get();
    node.setMeshBB(computeMeshBB(meshBB));
  }

  // Recursively add children
  for (const auto& child : meshTransformNode.children) {
    addComponent(metaData,       // mesh metadata
                 node,           // parent scene node
                 lightSetupKey,  // lightSetup key
                 drawables,      // drawable group
                 child,          // mesh transform node
                 visNodeCache,   // a vector of scene nodes, the visNodeCache
                 computeAbsoluteAABBs,  // compute absolute aabbs
                 staticDrawableInfo);   // a vector of static drawable info
  }
}  // addComponent

void ResourceManager::addPrimitiveToDrawables(int primitiveID,
                                              scene::SceneNode& node,
                                              DrawableGroup* drawables) {
  CHECK(primitive_meshes_.count(primitiveID));
  // TODO:
  // currently we assume the primitives does not have normal texture
  // so do not need to worry about the tangent or bitangent.
  // it might be changed in the future.
  gfx::Drawable::Flags meshAttributeFlags{};
  createDrawable(*primitive_meshes_.at(primitiveID),  // render mesh
                 meshAttributeFlags,                  // meshAttributeFlags
                 node,                                // scene node
                 NO_LIGHT_KEY,                        // lightSetup key
                 WHITE_MATERIAL_KEY,                  // material key
                 drawables);                          // drawable group
}

void ResourceManager::removePrimitiveMesh(int primitiveID) {
  CHECK(primitive_meshes_.count(primitiveID));
  primitive_meshes_.erase(primitiveID);
}

void ResourceManager::createDrawable(Mn::GL::Mesh& mesh,
                                     gfx::Drawable::Flags& meshAttributeFlags,
                                     scene::SceneNode& node,
                                     const Mn::ResourceKey& lightSetupKey,
                                     const Mn::ResourceKey& materialKey,
                                     DrawableGroup* group /* = nullptr */) {
  const auto& materialDataType =
      shaderManager_.get<gfx::MaterialData>(materialKey)->type;
  switch (materialDataType) {
    case gfx::MaterialDataType::None:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
    case gfx::MaterialDataType::Phong:
      node.addFeature<gfx::GenericDrawable>(
          mesh,                // render mesh
          meshAttributeFlags,  // mesh attribute flags
          shaderManager_,      // shader manager
          lightSetupKey,       // lightSetup key
          materialKey,         // material key
          group);              // drawable group
      break;
    case gfx::MaterialDataType::Pbr:
      node.addFeature<gfx::PbrDrawable>(
          mesh,                // render mesh
          meshAttributeFlags,  // mesh attribute flags
          shaderManager_,      // shader manager
          lightSetupKey,       // lightSetup key
          materialKey,         // material key
          group);              // drawable group
      break;
  }
}  // ResourceManager::createDrawable

bool ResourceManager::loadSUNCGHouseFile(const AssetInfo& houseInfo,
                                         scene::SceneNode* parent,
                                         DrawableGroup* drawables) {
  ASSERT(parent != nullptr);

  LOG(WARNING) << "SUNCG support is deprecated. This codepath is untested.";

  std::string houseFile = Cr::Utility::Directory::join(
      Cr::Utility::Directory::current(), houseInfo.filepath);
  const auto& json = io::parseJsonFile(houseFile);
  const auto& levels = json["levels"].GetArray();
  std::vector<std::string> pathTokens = io::tokenize(houseFile, "/", 0, true);
  ASSERT(pathTokens.size() >= 3);
  pathTokens.pop_back();  // house.json
  const std::string houseId = pathTokens.back();
  pathTokens.pop_back();  // <houseId>
  pathTokens.pop_back();  // house
  const std::string basePath = Cr::Utility::String::join(pathTokens, '/');

  // store nodeIds to obtain linearized index for semantic masks
  std::vector<std::string> nodeIds;

  for (const auto& level : levels) {
    const auto& nodes = level["nodes"].GetArray();
    for (const auto& node : nodes) {
      const std::string nodeId = node["id"].GetString();
      const std::string nodeType = node["type"].GetString();
      const int valid = node["valid"].GetInt();
      if (valid == 0) {
        continue;
      }

      // helper for creating object nodes
      auto createObjectFunc = [&](const AssetInfo& info,
                                  const std::string& id) -> scene::SceneNode& {
        scene::SceneNode& objectNode = parent->createChild();
        const int nodeIndex = nodeIds.size();
        nodeIds.push_back(id);
        objectNode.setId(nodeIndex);
        if (info.type == AssetType::SUNCG_OBJECT) {
          CHECK(loadRenderAsset(info));
          RenderAssetInstanceCreationInfo::Flags flags;
          flags |= RenderAssetInstanceCreationInfo::Flag::IsRGBD;
          flags |= RenderAssetInstanceCreationInfo::Flag::IsSemantic;
          RenderAssetInstanceCreationInfo objectCreation(
              info.filepath, Cr::Containers::NullOpt, flags, NO_LIGHT_KEY);
          createRenderAssetInstance(objectCreation, &objectNode, drawables);
        }
        return objectNode;
      };

      const std::string roomPath =
          basePath + std::string("/room/").append(houseId).append("/");
      if (nodeType == "Room") {
        const std::string roomBase = roomPath + node["modelId"].GetString();
        const int hideCeiling = node["hideCeiling"].GetInt();
        const int hideFloor = node["hideFloor"].GetInt();
        const int hideWalls = node["hideWalls"].GetInt();
        if (hideCeiling != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "c.glb"},
                           nodeId + "c");
        }
        if (hideWalls != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "w.glb"},
                           nodeId + "w");
        }
        if (hideFloor != 1) {
          createObjectFunc({AssetType::SUNCG_OBJECT, roomBase + "f.glb"},
                           nodeId + "f");
        }
      } else if (nodeType == "Object") {
        const std::string modelId = node["modelId"].GetString();
        // Parse model-to-scene transformation matrix
        // NOTE: only "Object" nodes have transform, other nodes are directly
        // specified in scene coordinates
        std::vector<float> transformVec;
        io::toFloatVector(node["transform"], &transformVec);
        mat4f transform(transformVec.data());
        const AssetInfo info{AssetType::SUNCG_OBJECT,
                             basePath + std::string("/object/")
                                            .append(modelId)
                                            .append("/")
                                            .append(modelId)
                                            .append(".glb")};
        createObjectFunc(info, nodeId)
            .setTransformation(Magnum::Matrix4{transform});
      } else if (nodeType == "Box") {
        // TODO(MS): create Box geometry
        createObjectFunc({}, nodeId);
      } else if (nodeType == "Ground") {
        const std::string roomBase = roomPath + node["modelId"].GetString();
        const AssetInfo info{AssetType::SUNCG_OBJECT, roomBase + "f.glb"};
        createObjectFunc(info, nodeId);
      } else {
        LOG(ERROR) << "Unrecognized SUNCG house node type " << nodeType;
      }
    }
  }
  return true;
}
void ResourceManager::initDefaultLightSetups() {
  shaderManager_.set(NO_LIGHT_KEY, gfx::LightSetup{});
  shaderManager_.setFallback(gfx::LightSetup{});
}

void ResourceManager::initDefaultMaterials() {
  shaderManager_.set<gfx::MaterialData>(DEFAULT_MATERIAL_KEY,
                                        new gfx::PhongMaterialData{});
  auto* whiteMaterialData = new gfx::PhongMaterialData;
  whiteMaterialData->ambientColor = Magnum::Color4{1.0};
  shaderManager_.set<gfx::MaterialData>(WHITE_MATERIAL_KEY, whiteMaterialData);
  auto* perVertexObjectId = new gfx::PhongMaterialData{};
  perVertexObjectId->perVertexObjectId = true;
  perVertexObjectId->ambientColor = Mn::Color4{1.0};
  shaderManager_.set<gfx::MaterialData>(PER_VERTEX_OBJECT_ID_MATERIAL_KEY,
                                        perVertexObjectId);
  shaderManager_.setFallback<gfx::MaterialData>(new gfx::PhongMaterialData{});
}

bool ResourceManager::isLightSetupCompatible(
    const LoadedAssetData& loadedAssetData,
    const Magnum::ResourceKey& lightSetupKey) const {
  // if light setup has lights in it, but asset was loaded in as flat shaded,
  // there may be an error when rendering.
  return lightSetupKey == Mn::ResourceKey{NO_LIGHT_KEY} ||
         loadedAssetData.assetInfo.requiresLighting;
}

//! recursively join all sub-components of a mesh into a single unified
//! MeshData.
void ResourceManager::joinHeirarchy(
    MeshData& mesh,
    const MeshMetaData& metaData,
    const MeshTransformNode& node,
    const Magnum::Matrix4& transformFromParentToWorld) const {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  if (node.meshIDLocal != ID_UNDEFINED) {
    CollisionMeshData& meshData =
        meshes_.at(node.meshIDLocal + metaData.meshIndex.first)
            ->getCollisionMeshData();
    int lastIndex = mesh.vbo.size();
    for (auto& pos : meshData.positions) {
      mesh.vbo.push_back(Magnum::EigenIntegration::cast<vec3f>(
          transformFromLocalToWorld.transformPoint(pos)));
    }
    for (auto& index : meshData.indices) {
      mesh.ibo.push_back(index + lastIndex);
    }
  }

  for (const auto& child : node.children) {
    joinHeirarchy(mesh, metaData, child, transformFromLocalToWorld);
  }
}

std::unique_ptr<MeshData> ResourceManager::createJoinedCollisionMesh(
    const std::string& filename) const {
  std::unique_ptr<MeshData> mesh = std::make_unique<MeshData>();

  CHECK(resourceDict_.count(filename) > 0);

  const MeshMetaData& metaData = getMeshMetaData(filename);

  Magnum::Matrix4 identity;
  joinHeirarchy(*mesh, metaData, metaData.root, identity);

  return mesh;
}

#ifdef ESP_BUILD_WITH_VHACD
bool ResourceManager::outputMeshMetaDataToObj(
    const std::string& MeshMetaDataFile,
    const std::string& new_filename,
    const std::string& filepath) const {
  bool success = Cr::Utility::Directory::mkpath(filepath);

  const MeshMetaData& metaData = getMeshMetaData(MeshMetaDataFile);
  std::string out = "# chd Mesh group \n";

  // write vertex info to file
  int numVertices = 0;
  for (const MeshTransformNode& node : metaData.root.children) {
    CollisionMeshData& meshData =
        meshes_.at(node.meshIDLocal + metaData.meshIndex.first)
            ->getCollisionMeshData();
    for (auto& pos : meshData.positions) {
      Mn::Utility::formatInto(out, out.size(), "{0} {1} {2} {3}{4}", "v",
                              pos[0], pos[1], pos[2], "\n");
      numVertices++;
    }
  }

  Mn::Utility::formatInto(out, out.size(), "{0} {1} {2}",
                          "# Number of vertices", numVertices, "\n\n");

  // Now do second pass to write indices for each group (node)
  int globalVertexNum = 1;
  int numParts = 1;
  for (const MeshTransformNode& node : metaData.root.children) {
    CollisionMeshData& meshData =
        meshes_.at(node.meshIDLocal + metaData.meshIndex.first)
            ->getCollisionMeshData();
    Mn::Utility::formatInto(out, out.size(), "{0}{1} {2}", "g part_", numParts,
                            "mesh\n");
    for (int ix = 0; ix < meshData.indices.size(); ix += 3) {
      Mn::Utility::formatInto(out, out.size(), "{0} {1} {2} {3}{4}", "f",
                              meshData.indices[ix] + globalVertexNum,
                              meshData.indices[ix + 1] + globalVertexNum,
                              meshData.indices[ix + 2] + globalVertexNum, "\n");
    }
    numParts++;
    globalVertexNum += meshData.positions.size();
  }
  Cr::Utility::Directory::writeString(filepath + "/" + new_filename, out);

  return success;
}  // ResourceManager::outputMeshMetaDataToObj

bool ResourceManager::isAssetDataRegistered(
    const std::string& resourceName) const {
  return (resourceDict_.count(resourceName) > 0);
}

void ResourceManager::createConvexHullDecomposition(
    const std::string& filename,
    const std::string& chdFilename,
    const VHACDParameters& params,
    const bool saveChdToObj) {
  if (resourceDict_.count(filename) == 0) {
    // retrieve existing, or create new, object attributes corresponding to
    // passed filename
    auto objAttributes =
        getObjectAttributesManager()->getObjectCopyByHandle(filename);
    if (objAttributes == nullptr) {
      objAttributes =
          getObjectAttributesManager()->createObject(filename, false);
    }

    // load/check for render MeshMetaData and load assets
    loadObjectMeshDataFromFile(filename, objAttributes, "render", true);

  }  // if no render asset exists

  // get joined mesh data
  assets::MeshData::uptr joinedMesh = assets::MeshData::create_unique();
  joinedMesh = createJoinedCollisionMesh(filename);

  // use VHACD
  interfaceVHACD->Compute(&joinedMesh->vbo[0][0], joinedMesh->vbo.size(),
                          &joinedMesh->ibo[0], joinedMesh->ibo.size() / 3,
                          params);

  Cr::Utility::Debug() << "== VHACD ran ==";

  // convert convex hulls into MeshDatas, CollisionMeshDatas
  int meshStart = meshes_.size();
  std::vector<CollisionMeshData> collisionMeshGroup;
  int nConvexHulls = interfaceVHACD->GetNConvexHulls();
  Cr::Utility::Debug() << "Num Convex Hulls: " << nConvexHulls;
  Cr::Utility::Debug() << "Resolution: " << params.m_resolution;
  VHACD::IVHACD::ConvexHull ch{};
  std::unique_ptr<GenericMeshData> genCHMeshData;
  for (unsigned int p = 0; p < nConvexHulls; ++p) {
    // for each convex hull, transfer the data to a newly created  MeshData
    interfaceVHACD->GetConvexHull(p, ch);

    std::vector<Magnum::Vector3> positions;

    // add the vertices
    positions.resize(ch.m_nPoints);
    for (size_t vix = 0; vix < ch.m_nPoints; vix++) {
      positions[vix] =
          Magnum::Vector3(ch.m_points[vix * 3], ch.m_points[vix * 3 + 1],
                          ch.m_points[vix * 3 + 2]);
    }

    // add indices
    Cr::Containers::ArrayView<const Mn::UnsignedInt> indices{
        ch.m_triangles, ch.m_nTriangles * 3};

    // create an owned MeshData
    Cr::Containers::Optional<Mn::Trade::MeshData> CHMesh = Mn::MeshTools::owned(
        Mn::Trade::MeshData{Mn::MeshPrimitive::Triangles,
                            {},
                            indices,
                            Mn::Trade::MeshIndexData{indices},
                            {},
                            positions,
                            {Mn::Trade::MeshAttributeData{
                                Mn::Trade::MeshAttribute::Position,
                                Cr::Containers::arrayView(positions)}}});

    // Create a GenericMeshData (needsNormals_ = true and uploadBuffersToGPU in
    // order to render the collision asset)
    genCHMeshData = std::make_unique<GenericMeshData>(true);
    genCHMeshData->setMeshData(*std::move(CHMesh));
    genCHMeshData->BB = computeMeshBB(genCHMeshData.get());
    genCHMeshData->uploadBuffersToGPU(true);

    // Create CollisionMeshData and add to collisionMeshGroup vector
    CollisionMeshData CHCollisionMesh = genCHMeshData->getCollisionMeshData();
    collisionMeshGroup.push_back(CHCollisionMesh);

    // register GenericMeshData in meshes_ dict
    meshes_.emplace(meshes_.size(), std::move(genCHMeshData));
  }
  // make MeshMetaData
  int meshEnd = meshes_.size() - 1;
  MeshMetaData meshMetaData{meshStart, meshEnd};

  // get original componentID (REVISIT)
  int componentID = getMeshMetaData(filename).root.componentID;

  // populate MeshMetaData root children
  for (unsigned int p = 0; p < nConvexHulls; ++p) {
    MeshTransformNode transformNode;
    transformNode.meshIDLocal = p;
    transformNode.componentID = componentID;
    meshMetaData.root.children.push_back(transformNode);
  }

  // make assetInfo
  AssetInfo info{AssetType::PRIMITIVE};
  info.requiresLighting = true;

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};

  // Register collision mesh group
  auto insertedCollisionMeshGroup =
      collisionMeshGroups_.emplace(chdFilename, std::move(collisionMeshGroup));
  // insert MeshMetaData into resourceDict_
  auto insertedResourceDict =
      resourceDict_.emplace(chdFilename, std::move(loadedAssetData));
  if (saveChdToObj) {
    std::string objDirectory = Cr::Utility::Directory::join(
        Corrade::Utility::Directory::current(), "data/VHACD_outputs");
    std::string new_filename =
        Cr::Utility::Directory::filename(
            Cr::Utility::Directory::splitExtension(chdFilename).first) +
        ".obj";
    outputMeshMetaDataToObj(chdFilename, new_filename, objDirectory);
  }
}
#endif
}  // namespace assets
}  // namespace esp
