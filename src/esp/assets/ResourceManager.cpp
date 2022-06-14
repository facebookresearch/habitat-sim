// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ResourceManager.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Containers/PointerStl.h>
#include <Corrade/Containers/Triple.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/PluginManager/PluginMetadata.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Path.h>
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
#include <Magnum/MeshTools/Concatenate.h>
#include <Magnum/MeshTools/FilterAttributes.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/Reference.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneTools/FlattenMeshHierarchy.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/FlatMaterialData.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/VertexFormat.h>

#include <memory>

#include "esp/geo/Geo.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/MaterialUtil.h"
#include "esp/gfx/PbrDrawable.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/io/Json.h"
#include "esp/io/URDFParser.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SemanticScene.h"

#include "esp/nav/PathFinder.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

#include "CollisionMeshData.h"
#include "GenericMeshData.h"
#include "GenericSemanticMeshData.h"
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
using metadata::attributes::ObjectInstanceShaderType;
using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::SceneObjectInstanceAttributes;
using metadata::attributes::StageAttributes;
using metadata::managers::AssetAttributesManager;
using metadata::managers::ObjectAttributesManager;
using metadata::managers::PhysicsAttributesManager;
using metadata::managers::StageAttributesManager;
using Mn::Trade::MaterialAttribute;

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
  // appropriately configure importerManager_ based on compilation flags
  buildImporters();

  if (flags_ & Flag::PbrImageBasedLighting) {
    // TODO: HDRi image name should be config based
    initPbrImageBasedLighting("lythwood_room_4k.jpg");
  }
}

ResourceManager::~ResourceManager() {
#ifdef ESP_BUILD_WITH_VHACD
  interfaceVHACD->Clean();
  interfaceVHACD->Release();
#endif
}

void ResourceManager::buildImporters() {
  // Preferred plugins, Basis target GPU format
#ifdef ESP_BUILD_ASSIMP_SUPPORT
  importerManager_.setPreferredPlugins("ObjImporter", {"AssimpImporter"});
  Cr::PluginManager::PluginMetadata* const assimpmetadata =
      importerManager_.metadata("AssimpImporter");
  assimpmetadata->configuration().setValue("ImportColladaIgnoreUpDirection",
                                           "true");
#endif

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

bool ResourceManager::getCreateRenderer() const {
  return metadataMediator_->getCreateRenderer();
}

void ResourceManager::initDefaultPrimAttributes() {
  if (!getCreateRenderer()) {
    return;
  }

  ConfigureImporterManagerGLExtensions();
  // by this point, we should have a GL::Context so load the bb primitive.
  // TODO: replace this completely with standard mesh (i.e. treat the bb
  // wireframe cube no differently than other primitive-based rendered
  // objects)
  auto cubeMeshName =
      getAssetAttributesManager()
          ->getObjectCopyByHandle<CubePrimitiveAttributes>("cubeWireframe")
          ->getPrimObjClassName();

  auto wfCube = primitiveImporter_->mesh(cubeMeshName);
  primitive_meshes_[nextPrimitiveMeshId++] =
      std::make_unique<Mn::GL::Mesh>(Mn::MeshTools::compile(*wfCube));

}  // initDefaultPrimAttributes

void ResourceManager::initPhysicsManager(
    std::shared_ptr<physics::PhysicsManager>& physicsManager,
    scene::SceneNode* parent,
    const metadata::attributes::PhysicsManagerAttributes::ptr&
        physicsManagerAttributes) {
  const bool isEnabled =
      metadataMediator_->getSimulatorConfiguration().enablePhysics;
  //! PHYSICS INIT: Use the passed attributes to initialize physics engine
  bool defaultToNoneSimulator = true;
  if (isEnabled) {
    if (physicsManagerAttributes->getSimulator() == "bullet") {
#ifdef ESP_BUILD_WITH_BULLET
      physicsManager = std::make_shared<physics::BulletPhysicsManager>(
          *this, physicsManagerAttributes);
      defaultToNoneSimulator = false;
#else
      ESP_WARNING()
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

std::unordered_map<uint32_t, std::vector<scene::CCSemanticObject::ptr>>
ResourceManager::buildSemanticCCObjects(
    const StageAttributes::ptr& stageAttributes) {
  if (!metadataMediator_->getSimulatorConfiguration().loadSemanticMesh) {
    ESP_WARNING() << "Unable to create semantic CC Objects due to no semantic "
                     "scene being loaded/existing.";
    return {};
  }

  std::map<std::string, AssetInfo> assetInfoMap =
      createStageAssetInfosFromAttributes(stageAttributes, false, true);

  AssetInfo semanticInfo = assetInfoMap.at("semantic");

  const std::string& filename = semanticInfo.filepath;
  if (!infoSemanticMeshData_) {
    /* Open the file. On error the importer already prints a diagnostic message,
   so no need to do that here. The importer implicitly converts per-face
   attributes to per-vertex, so nothing extra needs to be done. */
    ESP_CHECK((fileImporter_->openFile(filename) &&
               (fileImporter_->meshCount() > 0u)),
              Cr::Utility::formatString(
                  "Error loading semantic mesh data from file {}", filename));

    // flatten source meshes, preserving transforms, build semanticMeshData and
    // construct vertex-based semantic bboxes, if requested for dataset.
    infoSemanticMeshData_ =
        flattenImportedMeshAndBuildSemantic(*fileImporter_, semanticInfo);
  }

  // return connectivity query results - per color map of vectors of CC-based
  // Semantic objects.
  return infoSemanticMeshData_->buildCCBasedSemanticObjs(semanticScene_);
}  // ResourceManager::buildSemanticCCObjects

std::vector<std::string> ResourceManager::buildVertexColorMapReport(
    const metadata::attributes::StageAttributes::ptr& stageAttributes) {
  if (!semanticScene_) {
    // must have a semantic scene for this report to make sense.
    return {
        "Unable to evaluate vertex-to-semantic object color mapping due to "
        "semantic scene being null."};
  }
  std::map<std::string, AssetInfo> assetInfoMap =
      createStageAssetInfosFromAttributes(stageAttributes, false, true);

  AssetInfo semanticInfo = assetInfoMap.at("semantic");

  const std::string& filename = semanticInfo.filepath;
  if (!infoSemanticMeshData_) {
    /* Open the file. On error the importer already prints a diagnostic message,
       so no need to do that here. The importer implicitly converts per-face
       attributes to per-vertex, so nothing extra needs to be done. */
    ESP_CHECK((fileImporter_->openFile(filename) &&
               (fileImporter_->meshCount() > 0u)),
              Cr::Utility::formatString(
                  "Error loading semantic mesh data from file {}", filename));

    // flatten source meshes, preserving transforms, build semanticMeshData and
    // construct vertex-based semantic bboxes, if requested for dataset.
    infoSemanticMeshData_ =
        flattenImportedMeshAndBuildSemantic(*fileImporter_, semanticInfo);
  }

  return infoSemanticMeshData_->getVertColorSSDReport(
      Cr::Utility::Path::split(filename).second(), semanticColorMapBeingUsed_,
      semanticScene_);
}  // ResourceManager::buildVertexColorMapReport

bool ResourceManager::loadSemanticSceneDescriptor(
    const std::string& ssdFilename,
    const std::string& activeSceneName) {
  namespace FileUtil = Cr::Utility::Path;
  semanticScene_ = nullptr;
  if (ssdFilename != "") {
    bool success = false;
    // semantic scene descriptor might not exist
    semanticScene_ = scene::SemanticScene::create();
    ESP_DEBUG() << "SceneInstance :" << activeSceneName
                << "proposed Semantic Scene Descriptor filename :"
                << ssdFilename;

    bool fileExists = FileUtil::exists(ssdFilename);
    if (fileExists) {
      // Attempt to load semantic scene descriptor specified in scene instance
      // file, agnostic to file type inferred by name, if file exists.
      success = scene::SemanticScene::loadSemanticSceneDescriptor(
          ssdFilename, *semanticScene_);
      if (success) {
        ESP_DEBUG() << "SSD with SceneInstanceAttributes-provided name "
                    << ssdFilename << "successfully found and loaded";
      } else {
        // here if provided file exists but does not correspond to appropriate
        // SSD
        ESP_ERROR() << "SSD Load Failure! File with "
                       "SceneInstanceAttributes-provided name "
                    << ssdFilename << "exists but was unable to be loaded.";
      }
      return success;
      // if not success then try to construct a name
    } else {
      // attempt to look for specified file failed, attempt to build new file
      // name by searching in path specified of specified file for
      // info_semantic.json file for replica dataset
      const std::string constructedFilename = FileUtil::join(
          FileUtil::split(ssdFilename).first(), "info_semantic.json");
      fileExists = FileUtil::exists(constructedFilename);
      if (fileExists) {
        success = scene::SemanticScene::loadReplicaHouse(constructedFilename,
                                                         *semanticScene_);
        if (success) {
          ESP_DEBUG() << "SSD for Replica using constructed file :"
                      << constructedFilename << "in directory with"
                      << ssdFilename << "loaded successfully";
        } else {
          // here if constructed file exists but does not correspond to
          // appropriate SSD or some loading error occurred.
          ESP_ERROR() << "SSD Load Failure! Replica file with constructed name "
                      << ssdFilename << "exists but was unable to be loaded.";
        }
        return success;
      } else {
        // neither provided non-empty filename nor constructed filename
        // exists. This is probably due to an incorrect naming in the
        // SceneInstanceAttributes
        ESP_WARNING() << "SSD File Naming Issue! Neither "
                         "SceneInstanceAttributes-provided name :"
                      << ssdFilename
                      << " nor constructed filename :" << constructedFilename
                      << "exist on disk.";
        return false;
      }
    }  // if given SSD file name specified exists
  }    // if semantic scene descriptor specified in scene instance

  return false;
}  // ResourceManager::loadSemanticSceneDescriptor

void ResourceManager::buildSemanticColorMap() {
  CORRADE_ASSERT(semanticScene_,
                 "Unable to build Semantic Color map due to no semanticScene "
                 "being loaded.", );

  semanticColorMapBeingUsed_.clear();
  semanticColorAsInt_.clear();
  const auto& ssdClrMap = semanticScene_->getSemanticColorMap();
  if (ssdClrMap.empty()) {
    return;
  }

  // The color map was built with first maxSemanticID elements in proper order
  // to match provided semantic IDs (so that ID is IDX of semantic color in
  // map).  Any overflow colors will be uniquely mapped 1-to-1 to unmapped
  // semantic IDs as their index.
  semanticColorMapBeingUsed_.assign(ssdClrMap.begin(), ssdClrMap.end());
  buildSemanticColorAsIntMap();

}  // ResourceManager::buildSemanticColorMap

void ResourceManager::buildSemanticColorAsIntMap() {
  semanticColorAsInt_.clear();
  if (semanticColorMapBeingUsed_.empty()) {
    return;
  }
  semanticColorAsInt_.reserve(semanticColorMapBeingUsed_.size());

  // build listing of colors as ints, with idx being semantic ID
  std::transform(semanticColorMapBeingUsed_.cbegin(),
                 semanticColorMapBeingUsed_.cend(),
                 std::back_inserter(semanticColorAsInt_),
                 [](const Mn::Color3ub& color) -> uint32_t {
                   return geo::getValueAsUInt(color);
                 });
}

bool ResourceManager::loadStage(
    const StageAttributes::ptr& stageAttributes,
    const SceneObjectInstanceAttributes::cptr& stageInstanceAttributes,
    const std::shared_ptr<physics::PhysicsManager>& _physicsManager,
    esp::scene::SceneManager* sceneManagerPtr,
    std::vector<int>& activeSceneIDs) {
  // If the semantic mesh should be created, based on SimulatorConfiguration
  const bool createSemanticMesh =
      metadataMediator_->getSimulatorConfiguration().loadSemanticMesh;

  // Force creation of a separate semantic scene graph, even when no semantic
  // mesh is loaded for the stage.  This is required to support playback of any
  // replay that includes a semantic-only render asset instance.
  const bool forceSeparateSemanticSceneGraph =
      metadataMediator_->getSimulatorConfiguration()
          .forceSeparateSemanticSceneGraph;

  // create AssetInfos here for each potential mesh file for the scene, if they
  // are unique.
  bool buildCollisionMesh =
      ((_physicsManager != nullptr) &&
       (_physicsManager->getInitializationAttributes()->getSimulator() !=
        "none"));
  const std::string renderLightSetupKey(stageAttributes->getLightSetupKey());
  std::map<std::string, AssetInfo> assetInfoMap =
      createStageAssetInfosFromAttributes(stageAttributes, buildCollisionMesh,
                                          createSemanticMesh);

  // set equal to current Simulator::activeSemanticSceneID_ value
  int activeSemanticSceneID = activeSceneIDs[0];
  // if semantic scene load is requested and possible
  auto semanticInfoIter = assetInfoMap.find("semantic");
  if (semanticInfoIter != assetInfoMap.end()) {
    // check if file names exist
    AssetInfo semanticInfo = semanticInfoIter->second;
    auto semanticStageFilename = semanticInfo.filepath;
    if (Cr::Utility::Path::exists(semanticStageFilename)) {
      ESP_DEBUG() << "Loading Semantic Stage mesh :" << semanticStageFilename;
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
      // if texture-based semantic mesh specify in creation
      if (semanticInfo.hasSemanticTextures) {
        flags |= RenderAssetInstanceCreationInfo::Flag::IsTextureBasedSemantic;
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
        ESP_ERROR() << "Semantic Stage mesh load failed.";
        return false;
      }
      ESP_DEBUG() << "Semantic Stage mesh :" << semanticStageFilename
                  << "loaded.";

    } else if (semanticStageFilename !=
               "") {  // semantic file name does not exist but house does
      ESP_ERROR() << "Not loading semantic mesh with File Name :"
                  << semanticStageFilename << "does not exist.";
    }
  } else {  // not wanting to create semantic mesh
    ESP_DEBUG() << "Not loading semantic mesh";
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
  // if texture-based semantic mesh specify in creation
  // TODO: Currently do not support single mesh texture-based semantic and
  // render stages
  // if (renderInfo.hasSemanticTextures) {
  //   flags |= RenderAssetInstanceCreationInfo::Flag::IsTextureBasedSemantic;
  // }
  RenderAssetInstanceCreationInfo renderCreation(
      renderInfo.filepath, Cr::Containers::NullOpt, flags, renderLightSetupKey);
  ESP_DEBUG() << "Start load render asset" << renderInfo.filepath << ".";

  bool renderMeshSuccess = loadStageInternal(renderInfo,  // AssetInfo
                                             &renderCreation,
                                             &rootNode,    // parent scene node
                                             &drawables);  //  drawable group
  if (!renderMeshSuccess) {
    ESP_ERROR()
        << "Stage render mesh load failed, Aborting stage initialization.";
    return false;
  }
  // declare mesh group variable
  std::vector<CollisionMeshData> meshGroup;
  AssetInfo& infoToUse = renderInfo;
  auto colInfoIter = assetInfoMap.find("collision");
  if (colInfoIter != assetInfoMap.end()) {
    AssetInfo colInfo = colInfoIter->second;
    if (resourceDict_.count(colInfo.filepath) == 0) {
      ESP_DEBUG() << "Start load collision asset" << colInfo.filepath << ".";
      // will not reload if already present
      bool collisionMeshSuccess =
          loadStageInternal(colInfo,   // AssetInfo
                            nullptr,   // creation
                            nullptr,   // parent scene node
                            nullptr);  // drawable group

      if (!collisionMeshSuccess) {
        ESP_ERROR() << "Stage collision mesh load failed.  Aborting stage "
                       "initialization.";
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
    bool sceneSuccess = _physicsManager->addStage(
        stageAttributes, stageInstanceAttributes, meshGroup);
    if (!sceneSuccess) {
      ESP_ERROR() << "Adding Stage" << stageAttributes->getHandle()
                  << "to PhysicsManager failed. Aborting stage initialization.";
      return false;
    }
  }

  return true;
}  // ResourceManager::loadScene

bool ResourceManager::buildMeshGroups(
    const AssetInfo& info,
    std::vector<CollisionMeshData>& meshGroup) {
  auto colMeshGroupIter = collisionMeshGroups_.find(info.filepath);
  if (colMeshGroupIter == collisionMeshGroups_.end()) {
    //! Collect collision mesh group
    bool colMeshGroupSuccess = false;
    if ((info.type == AssetType::INSTANCE_MESH) && !info.hasSemanticTextures) {
      // PLY Semantic mesh
      colMeshGroupSuccess =
          buildStageCollisionMeshGroup<GenericSemanticMeshData>(info.filepath,
                                                                meshGroup);
    } else if ((info.type == AssetType::MP3D_MESH ||
                info.type == AssetType::UNKNOWN) ||
               ((info.type == AssetType::INSTANCE_MESH) &&
                info.hasSemanticTextures)) {
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
      ESP_ERROR() << "Stage" << info.filepath
                  << "Collision mesh load failed. Aborting scene "
                     "initialization.";
      return false;
    }
    //! Add scene meshgroup to collision mesh groups
    collisionMeshGroups_.emplace(info.filepath, meshGroup);
  } else {
    // collision meshGroup already exists from prior load
    meshGroup = colMeshGroupIter->second;
  }
  return true;
}  // ResourceManager::buildMeshGroups

std::map<std::string, AssetInfo>
ResourceManager::createStageAssetInfosFromAttributes(
    const StageAttributes::ptr& stageAttributes,
    bool createCollisionInfo,
    bool createSemanticInfo) {
  std::map<std::string, AssetInfo> resMap;
  auto frame = buildFrameFromAttributes(
      stageAttributes->getHandle(), stageAttributes->getOrientUp(),
      stageAttributes->getOrientFront(), stageAttributes->getOrigin());
  float virtualUnitToMeters = stageAttributes->getUnitsToMeters();
  // create render asset info
  auto renderType =
      static_cast<AssetType>(stageAttributes->getRenderAssetType());
  AssetInfo renderInfo{
      renderType,                               // type
      stageAttributes->getRenderAssetHandle(),  // file path
      frame,                                    // frame
      virtualUnitToMeters,                      // virtualUnitToMeters
      stageAttributes->getForceFlatShading()    // forceFlatShading
  };
  renderInfo.shaderTypeToUse = stageAttributes->getShaderType();
  std::string debugStr = "Frame :";
  Cr::Utility::formatInto(debugStr, debugStr.size(),
                          "{} for render mesh named : {}",
                          renderInfo.frame.toString(), renderInfo.filepath);
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
        true                                         // forceFlatShading
    };
    resMap["collision"] = collisionInfo;
  }
  if (createSemanticInfo) {
    // create semantic asset info if requested
    auto semanticType =
        static_cast<AssetType>(stageAttributes->getSemanticAssetType());
    // This check being false means a specific orientation for semantic meshes
    // was specified in config file, so they should use -this- orientation
    // instead of the base render asset orientation.
    if (!stageAttributes->getUseFrameForAllOrientation()) {
      frame = buildFrameFromAttributes(
          stageAttributes->getHandle(), stageAttributes->getSemanticOrientUp(),
          stageAttributes->getSemanticOrientFront(),
          stageAttributes->getOrigin());
    }
    AssetInfo semanticInfo{
        semanticType,                               // type
        stageAttributes->getSemanticAssetHandle(),  // file path
        frame,                                      // frame
        virtualUnitToMeters,                        // virtualUnitToMeters
        true,                                       // forceFlatShading
        // only split semantic mesh if doing frustum culling
        stageAttributes->getFrustumCulling()  // splitInstanceMesh
    };
    // specify whether the semantic asset has semantically annotated textures.
    // this is only true if the assets are available (as specified in the
    // dataset config) and if the  user has requested them (via
    // SimulatorConfiguration::useSemanticTexturesIfFound)
    semanticInfo.hasSemanticTextures = stageAttributes->useSemanticTextures();

    Cr::Utility::formatInto(
        debugStr, debugStr.size(),
        "|{} for semantic mesh named : {} with type specified as {}|Semantic "
        "Txtrs : {}",
        frame.toString(), semanticInfo.filepath,
        esp::metadata::attributes::getMeshTypeName(semanticInfo.type),
        (semanticInfo.hasSemanticTextures ? "True" : "False"));
    resMap["semantic"] = semanticInfo;
  } else {
    Cr::Utility::formatInto(debugStr, debugStr.size(),
                            "|No Semantic asset info specified.");
  }
  ESP_DEBUG() << debugStr;
  return resMap;
}  // ResourceManager::createStageAssetInfosFromAttributes

esp::geo::CoordinateFrame ResourceManager::buildFrameFromAttributes(
    const std::string& attribName,
    const Mn::Vector3& up,
    const Mn::Vector3& front,
    const Mn::Vector3& origin) {
  const vec3f upEigen{Mn::EigenIntegration::cast<vec3f>(up)};
  const vec3f frontEigen{Mn::EigenIntegration::cast<vec3f>(front)};
  if (upEigen.isOrthogonal(frontEigen)) {
    const vec3f originEigen{Mn::EigenIntegration::cast<vec3f>(origin)};
    esp::geo::CoordinateFrame frame{upEigen, frontEigen, originEigen};
    return frame;
  } else {
    ESP_DEBUG() << "Specified frame in Attributes :" << attribName
                << "is not orthogonal, so returning default frame.";
    esp::geo::CoordinateFrame frame;
    return frame;
  }
}  // ResourceManager::buildFrameFromAttributes

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
    // NOTE: This multiplication is a hack to roughly balance the Phong and PBR
    // light intensity reactions.
    phongMaterial->diffuseColor = materialColor.diffuseColor * 0.175;
    phongMaterial->specularColor = materialColor.specularColor * 0.175;

    shaderManager_.set(newMaterialID, static_cast<gfx::MaterialData*>(
                                          phongMaterial.release()));
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
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Unsupported instance creation flags for asset ["
          << assetInfo.filepath << "]";
      return nullptr;
    }
    sceneID = activeSceneIDs[0];
  } else {
    if (creation.isSemantic() && creation.isRGBD()) {
      if (activeSceneIDs[1] != activeSceneIDs[0]) {
        // Because we have a separate semantic scene graph, we can't support a
        // static instance with both isSemantic and isRGBD.
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Unsupported instance creation flags for asset ["
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
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Unsupported instance creation flags for asset ["
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

std::string ResourceManager::createModifiedAssetName(const AssetInfo& info,
                                                     std::string& materialId) {
  std::string modifiedAssetName = info.filepath;

  // check materialId
  if (info.overridePhongMaterial != Cr::Containers::NullOpt) {
    if (materialId.empty()) {
      // if passed value is empty, synthesize new color and new materialId
      // based on this color and values specified in info
      materialId = createColorMaterial(*info.overridePhongMaterial);
    }
    modifiedAssetName += '?' + materialId;
  }
  // construct name with materialId specification and desired shader type
  return modifiedAssetName;
}  // ResourceManager::createModifiedAssetName

scene::SceneNode* ResourceManager::loadAndCreateRenderAssetInstance(
    const AssetInfo& assetInfo,
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* visNodeCache) {
  if (!loadRenderAsset(assetInfo)) {
    return nullptr;
  }
  CORRADE_INTERNAL_ASSERT(assetInfo.filepath == creation.filepath);

  // copy the const creation info to modify the key if necessary
  RenderAssetInstanceCreationInfo finalCreation(creation);
  if (assetInfo.overridePhongMaterial != Cr::Containers::NullOpt) {
    std::string materiaId = "";
    // material override is requested so get the id
    finalCreation.filepath = createModifiedAssetName(assetInfo, materiaId);
  }

  return createRenderAssetInstance(finalCreation, parent, drawables,
                                   visNodeCache);
}

bool ResourceManager::loadRenderAsset(const AssetInfo& info) {
  bool registerMaterialOverride =
      (info.overridePhongMaterial != Cr::Containers::NullOpt);
  bool fileAssetIsLoaded = resourceDict_.count(info.filepath) > 0;

  bool meshSuccess = fileAssetIsLoaded;
  // first load the file asset as-is if necessary
  if (!fileAssetIsLoaded) {
    // clone the AssetInfo and remove the custom material to load a default
    // AssetInfo first
    AssetInfo defaultInfo(info);
    defaultInfo.overridePhongMaterial = Cr::Containers::NullOpt;

    if (info.type == AssetType::PRIMITIVE) {
      ESP_DEBUG() << "Building Prim named:" << info.filepath;
      buildPrimitiveAssetData(info.filepath);
      meshSuccess = true;
    } else if (info.type == AssetType::FRL_PTEX_MESH) {
      ESP_DEBUG() << "Loading PTEX asset named:" << info.filepath;
      meshSuccess = loadRenderAssetPTex(defaultInfo);
    } else if (info.type == AssetType::INSTANCE_MESH) {
      ESP_DEBUG() << "Loading Semantic Mesh asset named:" << info.filepath;
      meshSuccess = loadSemanticRenderAsset(defaultInfo);
    } else if (isRenderAssetGeneral(info.type)) {
      ESP_DEBUG() << "Loading general asset named:" << info.filepath;
      meshSuccess = loadRenderAssetGeneral(defaultInfo);
    } else {
      // loadRenderAsset doesn't yet support the requested asset type
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
    }

    if (meshSuccess) {
      // create and register the collisionMeshGroups
      if (info.type != AssetType::PRIMITIVE) {
        std::vector<CollisionMeshData> meshGroup;
        CORRADE_ASSERT(buildMeshGroups(defaultInfo, meshGroup),
                       "Failed to construct collisionMeshGroups for asset"
                           << info.filepath,
                       false);
      }

      if (gfxReplayRecorder_) {
        gfxReplayRecorder_->onLoadRenderAsset(defaultInfo);
      }
    }
  }

  // now handle loading the material override AssetInfo if configured
  if (meshSuccess && registerMaterialOverride) {
    // register or get the override material id
    std::string materialId = "";

    // construct the unique id for the material modified asset
    std::string modifiedAssetName = createModifiedAssetName(info, materialId);
    const bool matModAssetIsRegistered =
        resourceDict_.count(modifiedAssetName) > 0;
    if (!matModAssetIsRegistered) {
      // first register the copied metaData
      auto res = resourceDict_.emplace(
          modifiedAssetName, LoadedAssetData(resourceDict_.at(info.filepath)));
      // Replace the AssetInfo
      res.first->second.assetInfo = info;
      // Modify the MeshMetaData local material ids for all components
      std::vector<MeshTransformNode*> nodeQueue;
      nodeQueue.push_back(&res.first->second.meshMetaData.root);
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
}  // ResourceManager::loadRenderAsset

scene::SceneNode* ResourceManager::createRenderAssetInstance(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* visNodeCache) {
  auto resourceDictIter = resourceDict_.find(creation.filepath);
  CORRADE_ASSERT(resourceDictIter != resourceDict_.end(), "asset is not loaded",
                 nullptr);

  const LoadedAssetData& loadedAssetData = resourceDictIter->second;
  if (!isLightSetupCompatible(loadedAssetData, creation.lightSetupKey)) {
    ESP_WARNING()
        << "Instantiating render asset" << creation.filepath
        << "with incompatible light setup, instance will not be correctly lit."
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
                   "createRenderAssetInstanceVertSemantic doesn't support this",
                   nullptr);
    newNode = createSemanticRenderAssetInstance(creation, parent, drawables);
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
  ESP_DEBUG() << "Attempting to load stage" << filename << "";
  bool meshSuccess = true;
  if (filename != EMPTY_SCENE) {
    if (!Cr::Utility::Path::exists(filename)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Attempting to load stage but cannot find specified asset file : '"
          << filename << "'. Aborting.";
      meshSuccess = false;
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
          ESP_ERROR() << "Reloading asset" << filename
                      << "with different configuration not currently supported."
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
  } else {
    ESP_DEBUG() << "Loading empty scene since" << filename
                << "specified as filename.";
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
      ESP_DEBUG()
          << "AssetInfo::AssetType "
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
    const bool forceFlatShading) {
  bool success = false;
  if (!filename.empty()) {
    AssetInfo meshInfo{AssetType::UNKNOWN, filename};
    meshInfo.forceFlatShading = forceFlatShading;
    meshInfo.shaderTypeToUse = objectAttributes->getShaderType();
    meshInfo.frame = buildFrameFromAttributes(
        objectAttributes->getHandle(), objectAttributes->getOrientUp(),
        objectAttributes->getOrientFront(), {0, 0, 0});
    success = loadRenderAsset(meshInfo);
    if (!success) {
      ESP_ERROR() << "Failed to load a physical object ("
                  << objectAttributes->getHandle() << ")'s" << meshType
                  << "mesh from file :" << filename;
    }
  }
  return success;
}  // loadObjectMeshDataFromFile

Mn::Range3D ResourceManager::computeMeshBB(BaseMesh* meshDataGL) {
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
      "::computePTexMeshAbsoluteAABBs: number of "
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
                 "::computeGeneralMeshAbsoluteAABBs: number of "
                 "transforms does not match number of drawables.", );

  for (uint32_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    const int meshID = staticDrawableInfo[iEntry].meshID;

    Cr::Containers::Optional<Mn::Trade::MeshData>& meshData =
        meshes_.at(meshID)->getMeshData();
    CORRADE_ASSERT(meshData,
                   "::computeGeneralMeshAbsoluteAABBs: The mesh "
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

  CORRADE_ASSERT(absTransforms.size() == staticDrawableInfo.size(),
                 "::computeInstancelMeshAbsoluteAABBs: Number of "
                 "transforms does not match number of drawables. Aborting.", );

  for (size_t iEntry = 0; iEntry < absTransforms.size(); ++iEntry) {
    const int meshID = staticDrawableInfo[iEntry].meshID;

    std::vector<Mn::Vector3> transformedPositions =
        dynamic_cast<GenericSemanticMeshData&>(*meshes_.at(meshID))
            .getVertexBufferObjectCPU();

    Mn::MeshTools::transformPointsInPlace(absTransforms[iEntry],
                                          transformedPositions);

    scene::SceneNode& node = staticDrawableInfo[iEntry].node;
    node.setAbsoluteAABB(Mn::Math::minmax(transformedPositions));
  }  // iEntry
}

std::vector<Mn::Matrix4> ResourceManager::computeAbsoluteTransformations(
    const std::vector<StaticDrawableInfo>& staticDrawableInfo) {
  // sanity check
  if (staticDrawableInfo.empty()) {
    return {};
  }

  // basic assumption is that all the drawables are in the same scene;
  // so use the 1st element in the vector to obtain this scene
  auto* scene = dynamic_cast<MagnumScene*>(staticDrawableInfo[0].node.scene());

  CORRADE_ASSERT(scene != nullptr,
                 "::computeAbsoluteTransformations: The node is "
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

void ResourceManager::buildPrimitiveAssetData(
    const std::string& primTemplateHandle) {
  // retrieves -actual- template, not a copy
  esp::metadata::attributes::AbstractPrimitiveAttributes::ptr primTemplate =
      getAssetAttributesManager()->getObjectByHandle(primTemplateHandle);

  if (primTemplate == nullptr) {
    // Template does not yet exist, create it using its name - primitive
    // template names encode all the pertinent template settings and cannot be
    // changed by the user, so the template name can be used to recreate the
    // template itself.
    auto newTemplate = getAssetAttributesManager()->createTemplateFromHandle(
        primTemplateHandle);
    // if still null, fail.
    if (newTemplate == nullptr) {
      ESP_ERROR() << "Attempting to reference or build a "
                     "primitive template from an unknown/malformed handle :"
                  << primTemplateHandle << ".  Aborting";
      return;
    }
    // we do not want a copy of the newly created template, but the actual
    // template
    primTemplate = getAssetAttributesManager()->getObjectByHandle(
        newTemplate->getHandle());
  }
  // check if unique name of attributes describing primitive asset is present
  // already - don't remake if so
  auto primAssetHandle = primTemplate->getHandle();
  if (resourceDict_.count(primAssetHandle) > 0) {
    ESP_DEBUG() << "Primitive Asset exists already :" << primAssetHandle;
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
  info.forceFlatShading = false;
  // set up primitive mesh
  // make  primitive mesh structure
  auto primMeshData = std::make_unique<GenericMeshData>(false);
  // build mesh data object
  primMeshData->importAndSetMeshData(*primitiveImporter_, primClassName);

  // compute the mesh bounding box
  primMeshData->BB = computeMeshBB(primMeshData.get());

  if (getCreateRenderer()) {
    primMeshData->uploadBuffersToGPU(false);
  }

  // make MeshMetaData
  int meshStart = nextMeshID_++;
  int meshEnd = meshStart;
  MeshMetaData meshMetaData{meshStart, meshEnd};

  meshes_.emplace(meshStart, std::move(primMeshData));

  // default material for now
  std::unique_ptr<gfx::PhongMaterialData> phongMaterial =
      gfx::PhongMaterialData::create_unique();

  meshMetaData.root.materialID = std::to_string(nextMaterialID_++);
  shaderManager_.set(meshMetaData.root.materialID,
                     static_cast<gfx::MaterialData*>(phongMaterial.release()));
  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;

  // set the root rotation to world frame upon load
  meshMetaData.setRootFrameOrientation(info.frame);
  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};
  auto inserted =
      resourceDict_.emplace(primAssetHandle, std::move(loadedAssetData));

  ESP_DEBUG() << "Primitive Asset Added : ID :" << primTemplate->getID()
              << ": attr lib key :" << primTemplate->getHandle()
              << "| instance class :" << primClassName
              << "| Conf has group for this obj type :"
              << conf.hasGroup(primClassName);

}  // ResourceManager::buildPrimitiveAssetData

bool ResourceManager::loadRenderAssetPTex(const AssetInfo& info) {
  CORRADE_INTERNAL_ASSERT(info.type == AssetType::FRL_PTEX_MESH);

#ifdef ESP_BUILD_PTEX_SUPPORT
  // if this is a new file, load it and add it to the dictionary
  const std::string& filename = info.filepath;
  CORRADE_INTERNAL_ASSERT(resourceDict_.count(filename) == 0);

  const auto atlasDir = Cr::Utility::Path::join(
      Cr::Utility::Path::split(filename).first(), "textures");

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

  // set the root rotation to world frame upon load
  meshMetaData.setRootFrameOrientation(info.frame);

  CORRADE_ASSERT(meshMetaData.meshIndex.first == meshMetaData.meshIndex.second,
                 "::loadRenderAssetPTex: ptex mesh is not loaded "
                 "correctly. Aborting.",
                 false);

  return true;
#else
  ESP_ERROR()
      << "PTex support not enabled. Enable the BUILD_PTEX_SUPPORT CMake "
         "option when building.";
  return false;
#endif
}  // ResourceManager::loadRenderAssetPTex

scene::SceneNode* ResourceManager::createRenderAssetInstancePTex(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
#ifdef ESP_BUILD_PTEX_SUPPORT
  CORRADE_INTERNAL_ASSERT(!creation.scale);  // PTex doesn't support scale
  CORRADE_INTERNAL_ASSERT(creation.lightSetupKey ==
                          NO_LIGHT_KEY);  // PTex doesn't support
                                          // lighting

  const std::string& filename = creation.filepath;
  auto resDictIter = resourceDict_.find(filename);
  CORRADE_INTERNAL_ASSERT(resDictIter != resourceDict_.end());
  const LoadedAssetData& loadedAssetData = resDictIter->second;
  const MeshMetaData& metaData = getMeshMetaData(filename);
  const auto& info = loadedAssetData.assetInfo;
  auto indexPair = metaData.meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;
  std::vector<StaticDrawableInfo> staticDrawableInfo;

  scene::SceneNode* instanceRoot = &parent->createChild();
  if (getCreateRenderer()) {
    for (int iMesh = start; iMesh <= end; ++iMesh) {
      auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_.at(iMesh).get());
      pTexMeshData->uploadBuffersToGPU(false);
      for (int jSubmesh = 0; jSubmesh < pTexMeshData->getSize(); ++jSubmesh) {
        scene::SceneNode& node = instanceRoot->createChild();
        const quatf transform = info.frame.rotationFrameToWorld();
        node.setRotation(Mn::Quaternion(transform));
        node.addFeature<gfx::PTexMeshDrawable>(*pTexMeshData, jSubmesh,
                                               shaderManager_, drawables);
        staticDrawableInfo.emplace_back(StaticDrawableInfo{node, jSubmesh});
      }
    }
  } else {
    // don't push to gpu if not creating renderer
    for (int iMesh = start; iMesh <= end; ++iMesh) {
      auto* pTexMeshData = dynamic_cast<PTexMeshData*>(meshes_.at(iMesh).get());
      for (int jSubmesh = 0; jSubmesh < pTexMeshData->getSize(); ++jSubmesh) {
        scene::SceneNode& node = instanceRoot->createChild();
        const quatf transform = info.frame.rotationFrameToWorld();
        node.setRotation(Mn::Quaternion(transform));
        node.addFeature<gfx::PTexMeshDrawable>(*pTexMeshData, jSubmesh,
                                               shaderManager_, drawables);
        staticDrawableInfo.emplace_back(StaticDrawableInfo{node, jSubmesh});
      }
    }
  }
  // we assume a ptex mesh is only used as static
  CORRADE_INTERNAL_ASSERT(creation.isStatic());
  CORRADE_INTERNAL_ASSERT(metaData.meshIndex.first ==
                          metaData.meshIndex.second);

  computePTexMeshAbsoluteAABBs(*meshes_.at(metaData.meshIndex.first),
                               staticDrawableInfo);
  return instanceRoot;
#else
  ESP_ERROR()
      << "PTex support not enabled. Enable the BUILD_PTEX_SUPPORT CMake "
         "option when building.";
  return nullptr;
#endif
}  // ResourceManager::createRenderAssetInstancePTex

bool ResourceManager::loadSemanticRenderAsset(const AssetInfo& info) {
  if (info.hasSemanticTextures) {
    // use loadRenderAssetGeneral for texture-based semantics
    return loadRenderAssetGeneral(info);
  }
  // special handling for vertex-based semantics
  return loadRenderAssetSemantic(info);
}  // ResourceManager::loadSemanticRenderAsset

scene::SceneNode* ResourceManager::createSemanticRenderAssetInstance(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
  if (creation.isTextureBasedSemantic()) {
    // Treat texture-based semantic meshes as General/Primitves.
    return createRenderAssetInstanceGeneralPrimitive(creation, parent,
                                                     drawables, nullptr);
  }
  // Special handling for vertex-based semantics
  return createRenderAssetInstanceVertSemantic(creation, parent, drawables);

}  // ResourceManager::createSemanticRenderAssetInstance

GenericSemanticMeshData::uptr
ResourceManager::flattenImportedMeshAndBuildSemantic(Importer& fileImporter,
                                                     const AssetInfo& info) {
  const std::string& filename = info.filepath;

  // Transform meshData by reframing frame rotation.  Doing this here so that
  // transformation is caught in OBB calc.
  const Mn::Matrix4 reframeTransform = Mn::Matrix4::from(
      Mn::Quaternion(info.frame.rotationFrameToWorld()).toMatrix(),
      Mn::Vector3());

  auto sceneID = fileImporter.defaultScene();
  // The meshData to build
  Cr::Containers::Optional<Mn::Trade::MeshData> meshData;

  if (sceneID == -1) {
    // no default scene --- standalone OBJ/PLY files, for example
    // already verified at least one mesh exists, this means only one mesh,
    // so no need to merge/flatten anything
    meshData =
        Mn::MeshTools::transform3D(*fileImporter.mesh(0), reframeTransform);
  } else {
    // flatten multi-submesh source meshes, since GenericSemanticMeshData
    // re-partitions based on ID.
    Cr::Containers::Optional<Mn::Trade::SceneData> scene =
        fileImporter.scene(sceneID);

    Cr::Containers::Array<Mn::Trade::MeshData> flattenedMeshes;
    for (const Cr::Containers::Triple<Mn::UnsignedInt, Mn::Int, Mn::Matrix4>&
             meshTransformation :
         Mn::SceneTools::flattenMeshHierarchy3D(*scene)) {
      int iMesh = meshTransformation.first();
      if (Cr::Containers::Optional<Mn::Trade::MeshData> mesh =
              fileImporter.mesh(iMesh)) {
        const auto transform = reframeTransform * meshTransformation.third();
        arrayAppend(flattenedMeshes,
                    Mn::MeshTools::transform3D(*mesh, transform));
      }
    }

    // build view
    Cr::Containers::Array<Cr::Containers::Reference<const Mn::Trade::MeshData>>
        meshView;
    Cr::Containers::arrayReserve(meshView, flattenedMeshes.size());
    for (const auto& mesh : flattenedMeshes) {
      arrayAppend(meshView, mesh);
    }
    // build concatenated meshData from container of meshes.
    meshData = Mn::MeshTools::concatenate(meshView);
    // filter out all unnecessary attributes (i.e. texture coords) in order to
    // remove duplicate verts
    meshData =
        Mn::MeshTools::removeDuplicates(Mn::MeshTools::filterOnlyAttributes(
            *meshData, {
                           Mn::Trade::MeshAttribute::Position,
                           Mn::Trade::MeshAttribute::Color,
                           Mn::Trade::MeshAttribute::ObjectId
                           // filtering out UV/texture coords
                       }));

  }  // flatten/reframe src meshes

  // build semanticColorMapBeingUsed_ if semanticScene_ is not nullptr
  if (semanticScene_) {
    buildSemanticColorMap();
  }

  GenericSemanticMeshData::uptr semanticMeshData =
      GenericSemanticMeshData::buildSemanticMeshData(
          *meshData, Cr::Utility::Path::split(filename).second(),
          semanticColorMapBeingUsed_,
          (filename.find(".ply") == std::string::npos), semanticScene_);

  // augment colors_as_int array to handle if un-expected colors have been found
  // in mesh verts.
  if (semanticScene_) {
    buildSemanticColorAsIntMap();
  }
  return semanticMeshData;
}  // ResourceManager::loadAndFlattenImportedMeshData

bool ResourceManager::loadRenderAssetSemantic(const AssetInfo& info) {
  CORRADE_INTERNAL_ASSERT(info.type == AssetType::INSTANCE_MESH);

  const std::string& filename = info.filepath;

  CORRADE_INTERNAL_ASSERT(resourceDict_.count(filename) == 0);
  ConfigureImporterManagerGLExtensions();

  /* Open the file. On error the importer already prints a diagnostic message,
     so no need to do that here. The importer implicitly converts per-face
     attributes to per-vertex, so nothing extra needs to be done. */
  ESP_CHECK(
      (fileImporter_->openFile(filename) && (fileImporter_->meshCount() > 0u)),
      Cr::Utility::formatString("Error loading semantic mesh data from file {}",
                                filename));

  // flatten source meshes, preserving transforms, build semanticMeshData and
  // construct vertex-based semantic bboxes, if requested for dataset.
  GenericSemanticMeshData::uptr semanticMeshData =
      flattenImportedMeshAndBuildSemantic(*fileImporter_, info);

  // partition semantic mesh for culling
  std::vector<GenericSemanticMeshData::uptr> instanceMeshes;
  if (info.splitInstanceMesh && semanticMeshData->meshCanBePartitioned()) {
    instanceMeshes =
        GenericSemanticMeshData::partitionSemanticMeshData(semanticMeshData);
  } else {
    instanceMeshes.emplace_back(std::move(semanticMeshData));
  }

  ESP_CHECK(!instanceMeshes.empty(),
            Cr::Utility::formatString(
                "Error loading semantic mesh data from file {}", filename));

  int meshStart = nextMeshID_;
  int meshEnd = meshStart + instanceMeshes.size() - 1;
  nextMeshID_ = meshEnd + 1;
  MeshMetaData meshMetaData{meshStart, meshEnd};
  meshMetaData.root.children.resize(instanceMeshes.size());

  // specify colormap to use to build TextureVisualizerShader
  // If this is true, we want to build a colormap from the vertex colors.
  for (int meshIDLocal = 0; meshIDLocal < instanceMeshes.size();
       ++meshIDLocal) {
    if (getCreateRenderer()) {
      instanceMeshes[meshIDLocal]->uploadBuffersToGPU(false);
    }
    meshes_.emplace(meshStart + meshIDLocal,
                    std::move(instanceMeshes[meshIDLocal]));
    meshMetaData.root.children[meshIDLocal].meshIDLocal = meshIDLocal;
  }
  // reframe transform happened already - do not reapply
  // meshMetaData.setRootFrameOrientation(info.frame);

  // update the dictionary
  resourceDict_.emplace(filename,
                        LoadedAssetData{info, std::move(meshMetaData)});

  return true;
}  // ResourceManager::loadRenderAssetSemantic

scene::SceneNode* ResourceManager::createRenderAssetInstanceVertSemantic(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables) {
  CORRADE_INTERNAL_ASSERT(!creation.scale);  // IMesh doesn't support scale
  CORRADE_INTERNAL_ASSERT(creation.lightSetupKey ==
                          NO_LIGHT_KEY);  // IMesh doesn't support
                                          // lighting

  const bool computeAbsoluteAABBs = creation.isStatic();

  std::vector<StaticDrawableInfo> staticDrawableInfo;
  auto meshMetaData = getMeshMetaData(creation.filepath);
  auto indexPair = meshMetaData.meshIndex;
  int start = indexPair.first;
  int end = indexPair.second;

  scene::SceneNode* instanceRoot = &parent->createChild();
  // transform based on transformNode setting
  instanceRoot->MagnumObject::setTransformation(
      meshMetaData.root.transformFromLocalToParent);

  for (int iMesh = start; iMesh <= end; ++iMesh) {
    scene::SceneNode& node = instanceRoot->createChild();

    // Semantic mesh does NOT have normal texture, so do not bother to
    // query if the mesh data contain tangent or bitangent.
    gfx::Drawable::Flags meshAttributeFlags{
        gfx::Drawable::Flag::HasVertexColor};
    // WARNING:
    // This is to initiate drawables for semantic mesh, and the semantic mesh
    // data is NOT stored in the meshData_ in the BaseMesh.
    // That means One CANNOT query the data like e.g.,
    // meshes_.at(iMesh)->getMeshData()->hasAttribute(Mn::Trade::MeshAttribute::Tangent)
    // It will SEGFAULT!
    createDrawable(meshes_.at(iMesh)->getMagnumGLMesh(),  // render mesh
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
}  // ResourceManager::createRenderAssetInstanceVertSemantic

void ResourceManager::ConfigureImporterManagerGLExtensions() {
  if (!getCreateRenderer()) {
    return;
  }

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
    ESP_DEBUG() << "Importing Basis files as ASTC 4x4.";
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
    ESP_DEBUG() << "Importing Basis files as BC7.";
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
    ESP_DEBUG() << "Importing Basis files as BC3.";
    metadata->configuration().setValue("format", "Bc3RGBA");
  }
#ifndef MAGNUM_TARGET_GLES2
  else
#ifndef MAGNUM_TARGET_GLES
      if (context.isExtensionSupported<
              Mn::GL::Extensions::ARB::ES3_compatibility>())
#endif
  {
    ESP_DEBUG() << "Importing Basis files as ETC2.";
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
    ESP_DEBUG() << "Importing Basis files as PVRTC 4bpp.";
    metadata->configuration().setValue("format", "PvrtcRGBA4bpp");
  }
#endif
#if defined(MAGNUM_TARGET_GLES2) || !defined(MAGNUM_TARGET_GLES)
  else /* ES3 has ETC2 always */
  {
    ESP_WARNING() << "No supported GPU compressed texture format detected, "
                     "Basis images will get imported as RGBA8.";
    metadata->configuration().setValue("format", "RGBA8");
  }
#endif

}  // ResourceManager::ConfigureImporterManagerGLExtensions

namespace {

void setMeshTransformNodeChildren(
    const Mn::Trade::SceneData& scene,
    Cr::Containers::Array<
        Cr::Containers::Optional<esp::assets::MeshTransformNode>>& nodes,
    MeshTransformNode& parent,
    int parentID) {
  for (unsigned childObjectID : scene.childrenFor(parentID)) {
    CORRADE_INTERNAL_ASSERT(nodes[childObjectID]);
    parent.children.push_back(std::move(*nodes[childObjectID]));
    setMeshTransformNodeChildren(scene, nodes, parent.children.back(),
                                 childObjectID);
  }
}

}  // namespace

bool ResourceManager::loadRenderAssetGeneral(const AssetInfo& info) {
  // verify either is general render asset, or else is semantic/instance asset
  // w/texture annotations
  CORRADE_INTERNAL_ASSERT(
      isRenderAssetGeneral(info.type) ||
      ((info.type == AssetType::INSTANCE_MESH) && info.hasSemanticTextures));

  const std::string& filename = info.filepath;
  CORRADE_INTERNAL_ASSERT(resourceDict_.count(filename) == 0);
  ConfigureImporterManagerGLExtensions();

  ESP_CHECK(
      (fileImporter_->openFile(filename) && (fileImporter_->meshCount() > 0u)),
      Cr::Utility::formatString("Error loading general mesh data from file {}",
                                filename));

  // load file and add it to the dictionary
  LoadedAssetData loadedAssetData{info};
  if (requiresTextures_) {
    loadTextures(*fileImporter_, loadedAssetData);
    loadMaterials(*fileImporter_, loadedAssetData);
  }
  loadMeshes(*fileImporter_, loadedAssetData);
  auto inserted = resourceDict_.emplace(filename, std::move(loadedAssetData));
  MeshMetaData& meshMetaData = inserted.first->second.meshMetaData;

  // no default scene --- standalone OBJ/PLY files, for example
  // take a wild guess and load the first mesh with the first material
  if (fileImporter_->defaultScene() == -1) {
    if ((fileImporter_->meshCount() != 0u) &&
        meshes_.at(meshMetaData.meshIndex.first)) {
      meshMetaData.root.children.emplace_back();
      meshMetaData.root.children.back().meshIDLocal = 0;
      return true;
    } else {
      ESP_ERROR() << "No default scene available and no meshes found, exiting";
      return false;
    }
  }

  /* Load the scene */
  Cr::Containers::Optional<Mn::Trade::SceneData> scene;
  if (!(scene = fileImporter_->scene(fileImporter_->defaultScene())) ||
      !scene->is3D() || !scene->hasField(Mn::Trade::SceneField::Parent)) {
    ESP_ERROR() << "Cannot load scene, exiting";
    return false;
  }

  // Allocate objects that are part of the hierarchy. Parent / child
  // relationship handled at the very last because MeshTransformNode stores its
  // children by-value in a vector inside, which would mean we'd have to move
  // them out of here
  Cr::Containers::Array<
      Cr::Containers::Optional<esp::assets::MeshTransformNode>>
      nodes{std::size_t(scene->mappingBound())};
  for (const Cr::Containers::Pair<unsigned, int>& parent :
       scene->parentsAsArray()) {
    nodes[parent.first()].emplace();
    nodes[parent.first()]->componentID = parent.first();
  }

  // Set transformations. Objects that are not part of the hierarchy are
  // ignored, nodes that have no transformation entry retain an identity
  // transformation.
  for (const Cr::Containers::Pair<unsigned, Mn::Matrix4>& transformation :
       scene->transformations3DAsArray()) {
    if (Cr::Containers::Optional<esp::assets::MeshTransformNode>& node =
            nodes[transformation.first()]) {
      node->transformFromLocalToParent = transformation.second();
    }
  }

  // Add mesh indices for objects that have a mesh, again ignoring nodes that
  // are not part of the hierarchy.
  for (const Cr::Containers::Pair<
           unsigned, Cr::Containers::Pair<unsigned, int>>& meshMaterial :
       scene->meshesMaterialsAsArray()) {
    Cr::Containers::Optional<esp::assets::MeshTransformNode>& node =
        nodes[meshMaterial.first()];
    if (!node) {
      continue;
    }

    // If meshIDLocal != -1 then we have multiple meshes assigned to the same
    // MeshTransformNode.  We make subsequent meshes children of the first mesh
    // we've seen, and give them identity trasnforms.
    // TODO: either drop MeshTransformNode in favor of SceneData or use
    // Mn::SceneTools::convertToSingleFunctionObjects() when it's exposed.
    esp::assets::MeshTransformNode* tmpNode = &*node;
    if (node->meshIDLocal != -1) {
      node->children.emplace_back();
      tmpNode = &node->children.back();
      tmpNode->componentID = meshMaterial.first();
    }

    tmpNode->meshIDLocal = meshMaterial.second().first();
    if (meshMaterial.second().second() != -1) {
      tmpNode->materialID =
          std::to_string(meshMaterial.second().second() + nextMaterialID_ -
                         fileImporter_->materialCount());
    }
  }

  // Recursively populate the hierarchy, moving the MeshTransformNode instances
  // out of the nodes array
  setMeshTransformNodeChildren(*scene, nodes, meshMetaData.root, -1);

  meshMetaData.setRootFrameOrientation(info.frame);

  return true;
}  // ResourceManager::loadRenderAssetGeneral

scene::SceneNode* ResourceManager::createRenderAssetInstanceGeneralPrimitive(
    const RenderAssetInstanceCreationInfo& creation,
    scene::SceneNode* parent,
    DrawableGroup* drawables,
    std::vector<scene::SceneNode*>* userVisNodeCache) {
  CORRADE_INTERNAL_ASSERT(parent);
  CORRADE_INTERNAL_ASSERT(drawables);

  auto resourceDictIter = resourceDict_.find(creation.filepath);
  CORRADE_INTERNAL_ASSERT(resourceDictIter != resourceDict_.end());
  const LoadedAssetData& loadedAssetData = resourceDictIter->second;

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
    const std::vector<Mn::Color3>& colorVec,
    int numSegments,
    float radius,
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
  if (pts.size() < 2) {
    ESP_ERROR()
        << "Cannot build a trajectory from fewer than 2 points. Aborting.";
    return false;
  }

  ESP_DEBUG() << "Calling trajectoryTubeSolid to build a tube named :"
              << trajVisName << "with" << pts.size()
              << "points, building a tube of radius :" << radius << "using"
              << numSegments << "circular segments and" << numInterp
              << "interpolated points between each trajectory point.";

  // create mesh tube
  Cr::Containers::Optional<Mn::Trade::MeshData> trajTubeMesh =
      geo::buildTrajectoryTubeSolid(pts, colorVec, numSegments, radius, smooth,
                                    numInterp);
  ESP_DEBUG() << "Successfully returned from trajectoryTubeSolid";

  // make assetInfo
  AssetInfo info{AssetType::PRIMITIVE};
  info.forceFlatShading = false;
  // set up primitive mesh
  // make  primitive mesh structure
  auto visMeshData = std::make_unique<GenericMeshData>(false);
  visMeshData->setMeshData(*std::move(trajTubeMesh));
  // compute the mesh bounding box
  visMeshData->BB = computeMeshBB(visMeshData.get());

  ESP_CHECK(getCreateRenderer(),
            "buildTrajectoryVisualization requires a renderer");
  visMeshData->uploadBuffersToGPU(false);

  // make MeshMetaData
  int meshStart = nextMeshID_++;
  int meshEnd = meshStart;
  MeshMetaData meshMetaData{meshStart, meshEnd};

  meshes_.emplace(meshStart, std::move(visMeshData));

  // default material for now
  auto phongMaterial = gfx::PhongMaterialData::create_unique();
  phongMaterial->specularColor = {1.0, 1.0, 1.0, 1.0};
  phongMaterial->shininess = 160.f;
  phongMaterial->ambientColor = {1.0, 1.0, 1.0, 1.0};
  phongMaterial->perVertexObjectId = true;

  meshMetaData.root.materialID = std::to_string(nextMaterialID_++);
  shaderManager_.set(meshMetaData.root.materialID,
                     static_cast<gfx::MaterialData*>(phongMaterial.release()));

  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;

  // store the rotation to world frame upon load
  meshMetaData.setRootFrameOrientation(info.frame);

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};
  // TODO : need to free render assets associated with this object if
  // collision occurs, otherwise leak! (Currently unsupported).
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

  if (!pathFinder.isLoaded()) {
    return navMeshPrimitiveID;
  }

  if (!getCreateRenderer()) {
    return navMeshPrimitiveID;
  }

  // create the mesh
  std::vector<Mn::UnsignedInt> indices;
  std::vector<Mn::Vector3> positions;

  const MeshData::ptr navMeshData = pathFinder.getNavMeshData();

  // add the vertices
  positions.resize(navMeshData->vbo.size());
  for (size_t vix = 0; vix < navMeshData->vbo.size(); ++vix) {
    positions[vix] = Mn::Vector3{navMeshData->vbo[vix]};
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
  primitive_meshes_[nextPrimitiveMeshId++] =
      std::make_unique<Mn::GL::Mesh>(Mn::MeshTools::compile(visualNavMesh));

  if (parent != nullptr && drawables != nullptr &&
      navMeshPrimitiveID != ID_UNDEFINED) {
    // create the drawable
    addPrimitiveToDrawables(navMeshPrimitiveID, *parent, drawables);
    parent->setMeshBB(Mn::Math::minmax(positions));
    parent->computeCumulativeBB();
  }

  return navMeshPrimitiveID;
}  // ResourceManager::loadNavMeshVisualization

namespace {
/**
 * @brief given passed @ref metadata::attributes::ObjectInstanceShaderType @p
 * typeToCheck and given @ref Mn::Trade::MaterialData, verify that the
 * material's intrinsic type is the same as inferred by @p typeToCheck. Ignore
 * flat, since all shaders already support flat.
 * @param typeToCheck The type of shader being specified.
 * @param materialData The imported material to check for type
 * @return Whether the imported material's supported types include one
 * congruient with the specified shader type.
 */
bool compareShaderTypeToMnMatType(const ObjectInstanceShaderType typeToCheck,
                                  const Mn::Trade::MaterialData& materialData) {
  switch (typeToCheck) {
    case ObjectInstanceShaderType::Phong: {
      bool compRes =
          bool(materialData.types() & Mn::Trade::MaterialType::Phong);
      ESP_DEBUG() << "Forcing to Phong | Material currently"
                  << (compRes ? "supports" : "does not support") << "Phong";
      return compRes;
    }
    case ObjectInstanceShaderType::PBR: {
      bool compRes = bool(materialData.types() &
                          Mn::Trade::MaterialType::PbrMetallicRoughness);
      ESP_DEBUG() << "Forcing to PBR | Material currently"
                  << (compRes ? "supports" : "does not support") << "PBR";
      return compRes;
    }
    default: {
      return false;
    }
  }
}  // compareShaderTypeToMnMatType

}  // namespace

void ResourceManager::loadMaterials(Importer& importer,
                                    LoadedAssetData& loadedAssetData) {
  // Specify the shaderType to use to render the materials being imported
  ObjectInstanceShaderType shaderTypeToUse =
      getMaterialShaderType(loadedAssetData.assetInfo);

  // name of asset, for debugging purposes
  const std::string assetName =
      Cr::Utility::Path::split(loadedAssetData.assetInfo.filepath).second();
  int numMaterials = importer.materialCount();
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Building " << numMaterials << " materials for asset named '"
      << assetName << "' : ";

  if (loadedAssetData.assetInfo.hasSemanticTextures) {
    int textureBaseIndex = loadedAssetData.meshMetaData.textureIndex.first;
    // TODO: Verify this is correct process for building individual materials
    // for each semantic int texture.
    for (int iMaterial = 0; iMaterial < numMaterials; ++iMaterial) {
      Cr::Containers::Optional<Mn::Trade::MaterialData> materialData =
          importer.material(iMaterial);

      if (!materialData) {
        ESP_ERROR() << "Material load failed for index" << iMaterial
                    << "so skipping that material for asset" << assetName
                    << ".";
        continue;
      }
      // Semantic texture-based
      std::unique_ptr<gfx::PhongMaterialData> finalMaterial =
          gfx::PhongMaterialData::create_unique();

      // const auto& material = materialData->as<Mn::Trade::FlatMaterialData>();

      finalMaterial->ambientColor = Mn::Color4{1.0};
      finalMaterial->diffuseColor = Mn::Color4{};
      finalMaterial->specularColor = Mn::Color4{};
      finalMaterial->shaderTypeSpec =
          static_cast<int>(ObjectInstanceShaderType::Flat);
      // has texture-based semantic annotations
      finalMaterial->textureObjectId = true;
      // get semantic int texture
      finalMaterial->objectIdTexture =
          textures_.at(textureBaseIndex + iMaterial).get();

      shaderManager_.set<gfx::MaterialData>(std::to_string(nextMaterialID_++),
                                            finalMaterial.release());
    }
  } else {
    for (int iMaterial = 0; iMaterial < numMaterials; ++iMaterial) {
      // TODO:
      // it seems we have a way to just load the material once in this case,
      // as long as the materialName includes the full path to the material
      Cr::Containers::Optional<Mn::Trade::MaterialData> materialData =
          importer.material(iMaterial);

      if (!materialData) {
        ESP_ERROR() << "Material load failed for index" << iMaterial
                    << "so skipping that material for asset" << assetName
                    << ".";
        continue;
      }

      std::unique_ptr<gfx::MaterialData> finalMaterial;
      std::string debugStr =
          Cr::Utility::formatString("Idx {:.02d}:", iMaterial);

      int textureBaseIndex = loadedAssetData.meshMetaData.textureIndex.first;
      // If we are not using the material's native shadertype, or flat (Which
      // all materials already support), expand the Mn::Trade::MaterialData with
      // appropriate data for all possible shadertypes
      if ((shaderTypeToUse != ObjectInstanceShaderType::Material) &&
          (shaderTypeToUse != ObjectInstanceShaderType::Flat) &&
          !(compareShaderTypeToMnMatType(shaderTypeToUse, *materialData))) {
        Cr::Utility::formatInto(
            debugStr, debugStr.size(),
            "(Expanding existing materialData to support requested shaderType `"
            "{}`) ",
            metadata::attributes::getShaderTypeName(shaderTypeToUse));
        materialData = esp::gfx::createUniversalMaterial(*materialData);
      }

      // pbr shader spec, of material-specified and material specifies pbr
      if (checkForPassedShaderType(
              shaderTypeToUse, *materialData, ObjectInstanceShaderType::PBR,
              Mn::Trade::MaterialType::PbrMetallicRoughness)) {
        Cr::Utility::formatInto(debugStr, debugStr.size(), "PBR.");
        finalMaterial =
            buildPbrShadedMaterialData(*materialData, textureBaseIndex);

        // phong shader spec, of material-specified and material specifies phong
      } else if (checkForPassedShaderType(shaderTypeToUse, *materialData,
                                          ObjectInstanceShaderType::Phong,
                                          Mn::Trade::MaterialType::Phong)) {
        Cr::Utility::formatInto(debugStr, debugStr.size(), "Phong.");
        finalMaterial =
            buildPhongShadedMaterialData(*materialData, textureBaseIndex);

        // flat shader spec or material-specified and material specifies flat
      } else if (checkForPassedShaderType(shaderTypeToUse, *materialData,
                                          ObjectInstanceShaderType::Flat,
                                          Mn::Trade::MaterialType::Flat)) {
        Cr::Utility::formatInto(debugStr, debugStr.size(), "Flat.");
        finalMaterial =
            buildFlatShadedMaterialData(*materialData, textureBaseIndex);

      } else {
        ESP_CHECK(
            false,
            Cr::Utility::formatString(
                "Unhandled ShaderType specification : {} and/or unmanaged "
                "type specified in material @ idx: {} for asset {}.",
                metadata::attributes::getShaderTypeName(shaderTypeToUse),
                iMaterial, assetName));
      }
      ESP_DEBUG() << debugStr;
      // for now, just use unique ID for material key. This may change if we
      // expose materials to user for post-load modification
      shaderManager_.set(std::to_string(nextMaterialID_++),
                         finalMaterial.release());
    }
  }
}  // ResourceManager::loadMaterials

ObjectInstanceShaderType ResourceManager::getMaterialShaderType(
    const AssetInfo& info) const {
  // if specified to be force-flat, then should be flat shaded, regardless of
  // material or other settings.
  if (info.forceFlatShading) {
    return ObjectInstanceShaderType::Flat;
  }
  ObjectInstanceShaderType infoSpecShaderType = info.shaderTypeToUse;

  if (infoSpecShaderType == ObjectInstanceShaderType::Unspecified) {
    // use the material's inherent shadertype
    infoSpecShaderType = ObjectInstanceShaderType::Material;
  }
  ESP_DEBUG() << "Shadertype being used for file :"
              << Cr::Utility::Path::split(info.filepath).second()
              << "| shadertype name :"
              << metadata::attributes::getShaderTypeName(infoSpecShaderType);
  return infoSpecShaderType;
}  // ResourceManager::getMaterialShaderType

gfx::PhongMaterialData::uptr ResourceManager::buildFlatShadedMaterialData(
    const Mn::Trade::MaterialData& materialData,
    int textureBaseIndex) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  const auto& material = materialData.as<Mn::Trade::FlatMaterialData>();

  // To save on shader switching, a Phong shader with zero lights is used for
  // flat materials
  auto finalMaterial = gfx::PhongMaterialData::create_unique();

  // texture transform, if the material has a texture; if there's none the
  // matrix is an identity
  // TODO this check shouldn't be needed, remove when this no longer asserts
  // in magnum for untextured materials
  if (material.hasTexture()) {
    finalMaterial->textureMatrix = material.textureMatrix();
  }

  finalMaterial->ambientColor = material.color();
  if (material.hasTexture()) {
    finalMaterial->ambientTexture =
        textures_.at(textureBaseIndex + material.texture()).get();
  }
  finalMaterial->diffuseColor = 0x00000000_rgbaf;
  finalMaterial->specularColor = 0x00000000_rgbaf;

  finalMaterial->shaderTypeSpec =
      static_cast<int>(ObjectInstanceShaderType::Flat);

  return finalMaterial;
}  // ResourceManager::buildFlatShadedMaterialData

gfx::PhongMaterialData::uptr ResourceManager::buildPhongShadedMaterialData(
    const Mn::Trade::MaterialData& materialData,
    int textureBaseIndex) const {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;
  const auto& material = materialData.as<Mn::Trade::PhongMaterialData>();

  auto finalMaterial = gfx::PhongMaterialData::create_unique();
  finalMaterial->shininess = material.shininess();

  // texture transform, if there's none the matrix is an identity
  finalMaterial->textureMatrix = material.commonTextureMatrix();

  // ambient material properties
  finalMaterial->ambientColor = material.ambientColor();
  if (material.hasAttribute(MaterialAttribute::AmbientTexture)) {
    finalMaterial->ambientTexture =
        textures_.at(textureBaseIndex + material.ambientTexture()).get();
  }

  // diffuse material properties
  finalMaterial->diffuseColor = material.diffuseColor();
  if (material.hasAttribute(MaterialAttribute::DiffuseTexture)) {
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
  if (material.hasAttribute(MaterialAttribute::NormalTexture)) {
    finalMaterial->normalTexture =
        textures_.at(textureBaseIndex + material.normalTexture()).get();
  }

  finalMaterial->shaderTypeSpec =
      static_cast<int>(ObjectInstanceShaderType::Phong);

  return finalMaterial;
}  // ResourceManager::buildPhongShadedMaterialData

gfx::PbrMaterialData::uptr ResourceManager::buildPbrShadedMaterialData(
    const Mn::Trade::MaterialData& materialData,
    int textureBaseIndex) const {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;
  const auto& material =
      materialData.as<Mn::Trade::PbrMetallicRoughnessMaterialData>();

  auto finalMaterial = gfx::PbrMaterialData::create_unique();

  // texture transform, if there's none the matrix is an identity
  finalMaterial->textureMatrix = material.commonTextureMatrix();

  // base color (albedo)
  finalMaterial->baseColor = material.baseColor();
  if (material.hasAttribute(MaterialAttribute::BaseColorTexture)) {
    finalMaterial->baseColorTexture =
        textures_.at(textureBaseIndex + material.baseColorTexture()).get();
  }

  // normal map
  if (material.hasAttribute(MaterialAttribute::NormalTexture)) {
    // must be inside the if clause otherwise assertion fails if no normal
    // texture is presented
    finalMaterial->normalTextureScale = material.normalTextureScale();

    finalMaterial->normalTexture =
        textures_.at(textureBaseIndex + material.normalTexture()).get();
  }

  // emission
  finalMaterial->emissiveColor = material.emissiveColor();
  if (material.hasAttribute(MaterialAttribute::EmissiveTexture)) {
    finalMaterial->emissiveTexture =
        textures_.at(textureBaseIndex + material.emissiveTexture()).get();
    if (!material.hasAttribute(MaterialAttribute::EmissiveColor)) {
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
    CORRADE_ASSERT(material.hasNoneRoughnessMetallicTexture(),
                   "::buildPbrShadedMaterialData(): if both the metallic "
                   "and roughness texture exist, they must be packed in the "
                   "same texture "
                   "based on glTF 2.0 Spec.",
                   finalMaterial);
  }

  // TODO:
  // Support NormalRoughnessMetallicTexture packing
  CORRADE_ASSERT(!material.hasNormalRoughnessMetallicTexture(),
                 "::buildPbrShadedMaterialData(): "
                 "Sorry. NormalRoughnessMetallicTexture is not supported in "
                 "the current version. We will work on it.",
                 finalMaterial);

  // double-sided
  finalMaterial->doubleSided = material.isDoubleSided();

  finalMaterial->shaderTypeSpec =
      static_cast<int>(ObjectInstanceShaderType::PBR);

  return finalMaterial;
}  // ResourceManager::buildPbrShadedMaterialData

void ResourceManager::loadMeshes(Importer& importer,
                                 LoadedAssetData& loadedAssetData) {
  int meshStart = nextMeshID_;
  int meshEnd = meshStart + importer.meshCount() - 1;
  nextMeshID_ = meshEnd + 1;
  loadedAssetData.meshMetaData.setMeshIndices(meshStart, meshEnd);

  for (int iMesh = 0; iMesh < importer.meshCount(); ++iMesh) {
    // don't need normals if we aren't using lighting
    auto gltfMeshData = std::make_unique<GenericMeshData>(
        !loadedAssetData.assetInfo.forceFlatShading);
    gltfMeshData->importAndSetMeshData(importer, iMesh);

    // compute the mesh bounding box
    gltfMeshData->BB = computeMeshBB(gltfMeshData.get());
    if (getCreateRenderer()) {
      gltfMeshData->uploadBuffersToGPU(false);
    }
    meshes_.emplace(meshStart + iMesh, std::move(gltfMeshData));
  }
}  // ResourceManager::loadMeshes

Mn::Image2D ResourceManager::convertRGBToSemanticId(
    const Mn::ImageView2D& srcImage,
    Cr::Containers::Array<Mn::UnsignedShort>& clrToSemanticId) {
  // convert image to semantic image here

  const Mn::Vector2i size = srcImage.size();
  // construct empty integer image
  Mn::Image2D resImage{
      Mn::PixelFormat::R16UI, size,
      Cr::Containers::Array<char>{
          Mn::NoInit,
          std::size_t(size.product() * pixelSize(Mn::PixelFormat::R16UI))}};

  Cr::Containers::StridedArrayView2D<const Mn::Color3ub> input =
      srcImage.pixels<Mn::Color3ub>();
  Cr::Containers::StridedArrayView2D<Mn::UnsignedShort> output =
      resImage.pixels<Mn::UnsignedShort>();
  for (std::size_t y = 0; y != size.y(); ++y) {
    Cr::Containers::StridedArrayView1D<const Mn::Color3ub> inputRow = input[y];
    Cr::Containers::StridedArrayView1D<Mn::UnsignedShort> outputRow = output[y];
    for (std::size_t x = 0; x != size.x(); ++x) {
      const Mn::Color3ub color = inputRow[x];
      /* Fugly. Sorry. Needs better API on Magnum side. */
      const Mn::UnsignedInt colorInt = geo::getValueAsUInt(color);
      outputRow[x] = clrToSemanticId[colorInt];
    }
  }
  return resImage;
}  // ResourceManager::convertRGBToSemanticId
void ResourceManager::loadTextures(Importer& importer,
                                   LoadedAssetData& loadedAssetData) {
  int textureStart = nextTextureID_;
  int textureEnd = textureStart + importer.textureCount() - 1;
  nextTextureID_ = textureEnd + 1;
  loadedAssetData.meshMetaData.setTextureIndices(textureStart, textureEnd);
  if (loadedAssetData.assetInfo.hasSemanticTextures) {
    // build semantic BBoxes and semanticColorMapBeingUsed_ if semanticScene_
    // and save results to informational SemanticMeshData, to facilitate future
    // reporting
    infoSemanticMeshData_ = flattenImportedMeshAndBuildSemantic(
        importer, loadedAssetData.assetInfo);

    // We are assuming that the only textures that exist are the semantic
    // textures. We build table of all possible colors holding ushorts
    // representing semantic IDs for those colors. We then assign known semantic
    // IDs to table entries corresponding the ID's specified color. Unknown
    // entries have semantic id 0x0 (corresponding to Unknown object in semantic
    // scene).
    //
    Cr::Containers::Array<Mn::UnsignedShort> clrToSemanticId{
        Mn::DirectInit, 256 * 256 * 256, Mn::UnsignedShort(0x0)};

    for (std::size_t i = 0; i < semanticColorAsInt_.size(); ++i) {
      // skip '0x0' (black) color - already mapped 0
      if (semanticColorAsInt_[i] == 0) {
        continue;
      }
      // assign semantic ID to list at colorAsInt idx
      clrToSemanticId[semanticColorAsInt_[i]] =
          static_cast<Mn::UnsignedShort>(i);
    }

    for (int iTexture = 0; iTexture < importer.textureCount(); ++iTexture) {
      auto currentTextureID = textureStart + iTexture;
      auto txtrIter = textures_.emplace(currentTextureID,
                                        std::make_shared<Mn::GL::Texture2D>());
      auto& currentTexture = txtrIter.first->second;

      auto textureData = importer.texture(iTexture);
      if (!textureData ||
          textureData->type() != Mn::Trade::TextureType::Texture2D) {
        ESP_ERROR() << "Cannot load texture" << iTexture << "skipping";
        currentTexture = nullptr;
        continue;
      }
      // texture will end up being semantic IDs
      currentTexture->setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
          .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
          .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge);

      // load only first level of textures for semantic annotations
      Cr::Containers::Optional<Mn::Trade::ImageData2D> image =
          importer.image2D(textureData->image(), 0);
      if (!image) {
        ESP_ERROR() << "Cannot load texture image, skipping";
        currentTexture = nullptr;
        continue;
      }
      // Convert color-based image to semantic image here
      auto newImage = convertRGBToSemanticId(*image, clrToSemanticId);

      currentTexture
          ->setStorage(1, Mn::GL::TextureFormat::R16UI, newImage.size())
          .setSubImage(0, {}, newImage);

      // Whether semantic RGB or not
    }
  } else {
    for (int iTexture = 0; iTexture < importer.textureCount(); ++iTexture) {
      auto currentTextureID = textureStart + iTexture;
      auto txtrIter = textures_.emplace(currentTextureID,
                                        std::make_shared<Mn::GL::Texture2D>());
      auto& currentTexture = txtrIter.first->second;

      auto textureData = importer.texture(iTexture);
      if (!textureData ||
          textureData->type() != Mn::Trade::TextureType::Texture2D) {
        ESP_ERROR() << "Cannot load texture" << iTexture << "skipping";
        currentTexture = nullptr;
        continue;
      }

      // Configure the texture
      // Mn::GL::Texture2D& texture = *(textures_.at(textureStart +
      // iTexture).get());
      currentTexture->setMagnificationFilter(textureData->magnificationFilter())
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
          ESP_ERROR() << "Cannot load texture image, skipping";
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
            currentTexture->setStorage(Mn::Math::log2(image->size().max()) + 1,
                                       format, image->size());
            generateMipmap = true;
          } else {
            currentTexture->setStorage(levelCount, format, image->size());
          }
        }

        if (image->isCompressed()) {
          currentTexture->setCompressedSubImage(level, {}, *image);
        } else {
          currentTexture->setSubImage(level, {}, *image);
        }
      }

      // Mip level loading failed, fail the whole texture
      if (currentTexture == nullptr) {
        continue;
      }

      // Generate a mipmap if requested
      if (generateMipmap) {
        currentTexture->generateMipmap();
      }
    }
  }  // Whether semantic RGB or not
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
  // NOTE : this is called when a new object is being made, but before the
  // object has acquired a copy of its parent attributes.  No object should
  // ever have a copy of attributes with isDirty == true - any editing of
  // attributes for objects requires object rebuilding.
  if (objectAttributes->getIsDirty()) {
    CORRADE_ASSERT(
        (ID_UNDEFINED != getObjectAttributesManager()->registerObject(
                             objectAttributes, objectTemplateHandle)),
        "::instantiateAssetsOnDemand : Unknown failure "
        "attempting to register modified template :"
            << objectTemplateHandle
            << "before asset instantiation.  Aborting. ",
        false);
  }

  // get render asset handle
  std::string renderAssetHandle = objectAttributes->getRenderAssetHandle();
  // whether attributes requires lighting
  bool forceFlatShading = objectAttributes->getForceFlatShading();
  bool renderMeshSuccess = false;
  // no resource dict entry exists for renderAssetHandle
  if (resourceDict_.count(renderAssetHandle) == 0) {
    if (objectAttributes->getRenderAssetIsPrimitive()) {
      // needs to have a primitive asset attributes with same name
      if (!getAssetAttributesManager()->getObjectLibHasHandle(
              renderAssetHandle)) {
        // this is bad, means no render primitive template exists with
        // expected name.  should never happen
        ESP_ERROR() << "No primitive asset attributes exists with name :"
                    << renderAssetHandle
                    << "so unable to instantiate primitive-based render "
                       "object.  Aborting.";
        return false;
      }
      // build primitive asset for this object based on defined primitive
      // attributes
      buildPrimitiveAssetData(renderAssetHandle);

    } else {
      // load/check_for render mesh metadata and load assets
      renderMeshSuccess = loadObjectMeshDataFromFile(
          renderAssetHandle, objectAttributes, "render", forceFlatShading);
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
          !renderMeshSuccess && !forceFlatShading);

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
    Mn::GL::Mesh* mesh = meshes_.at(meshID)->getMagnumGLMesh();
    if (getCreateRenderer()) {
      CORRADE_ASSERT(mesh,
                     "::addComponent() : GL mesh expected but not found", );
    } else {
      CORRADE_ASSERT(!mesh,
                     "addComponent() : encountered unexpected GL mesh with "
                     "createRenderer==false", );
    }
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
  auto primMeshIter = primitive_meshes_.find(primitiveID);
  CORRADE_INTERNAL_ASSERT(primMeshIter != primitive_meshes_.end());
  // TODO:
  // currently we assume the primitives does not have normal texture
  // so do not need to worry about the tangent or bitangent.
  // it might be changed in the future.
  gfx::Drawable::Flags meshAttributeFlags{};
  createDrawable(primMeshIter->second.get(),  // render mesh
                 meshAttributeFlags,          // meshAttributeFlags
                 node,                        // scene node
                 NO_LIGHT_KEY,                // lightSetup key
                 WHITE_MATERIAL_KEY,          // material key
                 drawables);                  // drawable group
}

void ResourceManager::removePrimitiveMesh(int primitiveID) {
  auto primMeshIter = primitive_meshes_.find(primitiveID);
  CORRADE_INTERNAL_ASSERT(primMeshIter != primitive_meshes_.end());
  primitive_meshes_.erase(primMeshIter);
}

void ResourceManager::createDrawable(Mn::GL::Mesh* mesh,
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
          group,               // drawable group
          activePbrIbl_ >= 0 ? pbrImageBasedLightings_[activePbrIbl_].get()
                             : nullptr);  // pbr image based lighting
      break;
  }
}  // ResourceManager::createDrawable

void ResourceManager::initDefaultLightSetups() {
  shaderManager_.set(NO_LIGHT_KEY, gfx::LightSetup{});
  shaderManager_.setFallback(gfx::LightSetup{});
}

void ResourceManager::initPbrImageBasedLighting(
    const std::string& hdriImageFilename) {
  // TODO:
  // should work with the scene instance config, initialize
  // different PBR IBLs at different positions in the scene.

  // TODO: HDR Image!
  pbrImageBasedLightings_.emplace_back(
      std::make_unique<gfx::PbrImageBasedLighting>(
          gfx::PbrImageBasedLighting::Flag::IndirectDiffuse |
              gfx::PbrImageBasedLighting::Flag::IndirectSpecular |
              gfx::PbrImageBasedLighting::Flag::UseLDRImages,
          shaderManager_, hdriImageFilename));
  activePbrIbl_ = 0;
}

void ResourceManager::initDefaultMaterials() {
  shaderManager_.set<gfx::MaterialData>(DEFAULT_MATERIAL_KEY,
                                        new gfx::PhongMaterialData{});
  auto* whiteMaterialData = new gfx::PhongMaterialData;
  whiteMaterialData->ambientColor = Mn::Color4{1.0};
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
    const Mn::ResourceKey& lightSetupKey) const {
  // if light setup has lights in it, but asset was loaded in as flat shaded,
  // there may be an error when rendering.
  return lightSetupKey == Mn::ResourceKey{NO_LIGHT_KEY} ||
         !loadedAssetData.assetInfo.forceFlatShading;
}

//! recursively join all sub-components of a mesh into a single unified
//! MeshData.
void ResourceManager::joinHierarchy(
    MeshData& mesh,
    const MeshMetaData& metaData,
    const MeshTransformNode& node,
    const Mn::Matrix4& transformFromParentToWorld) const {
  Mn::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  if (node.meshIDLocal != ID_UNDEFINED) {
    CollisionMeshData& meshData =
        meshes_.at(node.meshIDLocal + metaData.meshIndex.first)
            ->getCollisionMeshData();
    int lastIndex = mesh.vbo.size();
    for (const auto& pos : meshData.positions) {
      mesh.vbo.push_back(Mn::EigenIntegration::cast<vec3f>(
          transformFromLocalToWorld.transformPoint(pos)));
    }
    for (const auto& index : meshData.indices) {
      mesh.ibo.push_back(index + lastIndex);
    }
  }

  for (const auto& child : node.children) {
    joinHierarchy(mesh, metaData, child, transformFromLocalToWorld);
  }
}

//! recursively join all sub-components of the semantic mesh into a single
//! unified MeshData.
void ResourceManager::joinSemanticHierarchy(
    MeshData& mesh,
    std::vector<uint16_t>& meshObjectIds,
    const MeshMetaData& metaData,
    const MeshTransformNode& node,
    const Mn::Matrix4& transformFromParentToWorld) const {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  // If the mesh local id is not -1, populate the mesh data.
  if (node.meshIDLocal != ID_UNDEFINED) {
    std::shared_ptr<BaseMesh> baseMeshData =
        meshes_.at(node.meshIDLocal + metaData.meshIndex.first);
    std::shared_ptr<GenericSemanticMeshData> meshData =
        std::dynamic_pointer_cast<GenericSemanticMeshData>(baseMeshData);

    if (!meshData) {
      ESP_ERROR() << "Could not get the GenericSemanticMeshData";
      return;
    }

    const std::vector<Mn::Vector3>& vertices =
        meshData->getVertexBufferObjectCPU();
    const std::vector<uint32_t>& indices = meshData->getIndexBufferObjectCPU();
    const std::vector<uint16_t>& objectIds =
        meshData->getObjectIdsBufferObjectCPU();
    // Note : The color is not being used currently

    int lastIndex = mesh.vbo.size();

    // Save the vertices
    for (const auto& pos : vertices) {
      mesh.vbo.push_back(Magnum::EigenIntegration::cast<vec3f>(
          transformFromLocalToWorld.transformPoint(pos)));
    }

    // Save the indices
    for (const auto& index : indices) {
      mesh.ibo.push_back(index + lastIndex);
    }

    // Save the object ids
    for (const auto ids : objectIds) {
      meshObjectIds.push_back(ids);
    }
  }

  // for all the children of the node, recurse
  for (const auto& child : node.children) {
    joinSemanticHierarchy(mesh, meshObjectIds, metaData, child,
                          transformFromLocalToWorld);
  }
}

std::unique_ptr<MeshData> ResourceManager::createJoinedCollisionMesh(
    const std::string& filename) const {
  std::unique_ptr<MeshData> mesh = std::make_unique<MeshData>();

  const MeshMetaData& metaData = getMeshMetaData(filename);

  Mn::Matrix4 identity;
  joinHierarchy(*mesh, metaData, metaData.root, identity);

  return mesh;
}

std::unique_ptr<MeshData> ResourceManager::createJoinedSemanticCollisionMesh(
    std::vector<std::uint16_t>& objectIds,
    const std::string& filename) const {
  std::unique_ptr<MeshData> mesh = std::make_unique<MeshData>();

  CORRADE_INTERNAL_ASSERT(resourceDict_.count(filename) > 0);

  const MeshMetaData& metaData = getMeshMetaData(filename);

  Magnum::Matrix4 identity;
  joinSemanticHierarchy(*mesh, objectIds, metaData, metaData.root, identity);

  return mesh;
}

#ifdef ESP_BUILD_WITH_VHACD
bool ResourceManager::outputMeshMetaDataToObj(
    const std::string& MeshMetaDataFile,
    const std::string& new_filename,
    const std::string& filepath) const {
  bool success = Cr::Utility::Path::make(filepath);

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
  Cr::Utility::Path::write(Cr::Utility::Path::join(filepath, new_filename),
                           Cr::Containers::StringView{out});

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

  ESP_DEBUG() << "== VHACD ran ==";

  // convert convex hulls into MeshDatas, CollisionMeshDatas
  int meshStart = meshes_.size();
  std::vector<CollisionMeshData> collisionMeshGroup;
  int nConvexHulls = interfaceVHACD->GetNConvexHulls();
  ESP_DEBUG() << "Num Convex Hulls:" << nConvexHulls;
  ESP_DEBUG() << "Resolution:" << params.m_resolution;
  VHACD::IVHACD::ConvexHull ch{};
  std::unique_ptr<GenericMeshData> genCHMeshData;
  for (unsigned int p = 0; p < nConvexHulls; ++p) {
    // for each convex hull, transfer the data to a newly created  MeshData
    interfaceVHACD->GetConvexHull(p, ch);

    std::vector<Mn::Vector3> positions;

    // add the vertices
    positions.resize(ch.m_nPoints);
    for (size_t vix = 0; vix < ch.m_nPoints; ++vix) {
      positions[vix] =
          Mn::Vector3(ch.m_points[vix * 3], ch.m_points[vix * 3 + 1],
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

    // Create a GenericMeshData (needsNormals_ = true and uploadBuffersToGPU
    // in order to render the collision asset)
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
  info.forceFlatShading = false;

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{info, meshMetaData};

  // Register collision mesh group
  auto insertedCollisionMeshGroup =
      collisionMeshGroups_.emplace(chdFilename, std::move(collisionMeshGroup));
  // insert MeshMetaData into resourceDict_
  auto insertedResourceDict =
      resourceDict_.emplace(chdFilename, std::move(loadedAssetData));
  if (saveChdToObj) {
    std::string objDirectory = Cr::Utility::Path::join(
        *Cr::Utility::Path::currentDirectory(), "data/VHACD_outputs");
    std::string new_filename =
        Cr::Utility::Path::split(
            Cr::Utility::Path::splitExtension(chdFilename).first())
            .second() +
        ".obj";
    outputMeshMetaDataToObj(chdFilename, new_filename, objDirectory);
  }
}
#endif

}  // namespace assets
}  // namespace esp
