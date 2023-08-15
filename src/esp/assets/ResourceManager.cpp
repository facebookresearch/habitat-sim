// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ResourceManager.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/BitArray.h>
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
#include <Corrade/Utility/Resource.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/MaterialTools/Merge.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/Tags.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Concatenate.h>
#include <Magnum/MeshTools/Copy.h>
#include <Magnum/MeshTools/Filter.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneTools/Hierarchy.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/FlatMaterialData.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/SkinData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/VertexFormat.h>

#include <memory>
#include <utility>

#include "esp/assets/BaseMesh.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/geo/Geo.h"
#include "esp/gfx/DrawableConfiguration.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/PbrDrawable.h"
#include "esp/gfx/SkinData.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/io/Json.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/URDFParser.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SemanticScene.h"

#include "esp/nav/PathFinder.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

#include "CollisionMeshData.h"
#include "GenericMeshData.h"
#include "GenericSemanticMeshData.h"
#include "MeshData.h"

// This is to import the "resources" at runtime. When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called *outside* of any namespace.
static void importPbrImageResources() {
  CORRADE_RESOURCE_INITIALIZE(PbrIBlImageResources)
}

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {

using metadata::attributes::AbstractObjectAttributes;
using metadata::attributes::ArticulatedObjectAttributes;
using metadata::attributes::CubePrimitiveAttributes;
using metadata::attributes::ObjectAttributes;
using metadata::attributes::ObjectInstanceShaderType;
using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::SceneObjectInstanceAttributes;
using metadata::attributes::StageAttributes;
using metadata::managers::AOAttributesManager;
using metadata::managers::AssetAttributesManager;
using metadata::managers::ObjectAttributesManager;
using metadata::managers::PhysicsAttributesManager;
using metadata::managers::StageAttributesManager;
using Mn::Trade::MaterialAttribute;

namespace assets {

ResourceManager::ResourceManager(
    metadata::MetadataMediator::ptr _metadataMediator)
    : metadataMediator_(std::move(_metadataMediator))
#ifdef MAGNUM_BUILD_STATIC
      ,
      // avoid using plugins that might depend on different library versions
      importerManager_("nonexistent")
#endif
{
  initDefaultLightSetups();
  initDefaultMaterials();
  // appropriately configure importerManager_ based on compilation flags
  buildImporters();
}

ResourceManager::~ResourceManager() = default;

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

  // instantiate importer for file load
  CORRADE_INTERNAL_ASSERT_OUTPUT(
      fileImporter_ = importerManager_.loadAndInstantiate("AnySceneImporter"));

  // instantiate importer for image load
  CORRADE_INTERNAL_ASSERT_OUTPUT(
      imageImporter_ = importerManager_.loadAndInstantiate("AnyImageImporter"));

  // set quiet importer flags if asset logging is quieted
  if (!isLevelEnabled(logging::Subsystem::assets,
                      logging::LoggingLevel::Warning)) {
    fileImporter_->addFlags(Mn::Trade::ImporterFlag::Quiet);
    primitiveImporter_->addFlags(Mn::Trade::ImporterFlag::Quiet);
    imageImporter_->addFlags(Mn::Trade::ImporterFlag::Quiet);
  } else if (isLevelEnabled(logging::Subsystem::assets,
                            logging::LoggingLevel::VeryVerbose)) {
    // set verbose flags if necessary
    fileImporter_->addFlags(Mn::Trade::ImporterFlag::Verbose);
    primitiveImporter_->addFlags(Mn::Trade::ImporterFlag::Verbose);
    imageImporter_->addFlags(Mn::Trade::ImporterFlag::Verbose);
  }

  // necessary for importer to be usable
  primitiveImporter_->openData("");
}  // buildImporters

bool ResourceManager::getCreateRenderer() const {
  return metadataMediator_->getCreateRenderer();
}

void ResourceManager::initDefaultPrimAttributes() {
  if (!getCreateRenderer()) {
    return;
  }

  configureImporterManagerGLExtensions();
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
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "SceneInstance : `" << activeSceneName
        << "` proposed Semantic Scene Descriptor filename : `" << ssdFilename
        << "`.";

    bool fileExists = FileUtil::exists(ssdFilename);
    if (fileExists) {
      // Attempt to load semantic scene descriptor specified in scene instance
      // file, agnostic to file type inferred by name, if file exists.
      success = scene::SemanticScene::loadSemanticSceneDescriptor(
          ssdFilename, *semanticScene_);
      if (success) {
        ESP_DEBUG(Mn::Debug::Flag::NoSpace)
            << "SSD with SceneInstanceAttributes-provided name `" << ssdFilename
            << "` successfully found and loaded.";
      } else {
        // here if provided file exists but does not correspond to appropriate
        // SSD
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << "SSD Load Failure! File with "
               "SceneInstanceAttributes-provided name `"
            << ssdFilename << "` exists but failed to load.";
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
          ESP_DEBUG(Mn::Debug::Flag::NoSpace)
              << "SSD for Replica using constructed file : `"
              << constructedFilename << "` in directory with `" << ssdFilename
              << "` loaded successfully";
        } else {
          // here if constructed file exists but does not correspond to
          // appropriate SSD or some loading error occurred.
          ESP_ERROR(Mn::Debug::Flag::NoSpace)
              << "SSD Load Failure! Replica file with constructed name `"
              << ssdFilename << "` exists but failed to load.";
        }
        return success;
      } else {
        // neither provided non-empty filename nor constructed filename
        // exists. This is probably due to an incorrect naming in the
        // SceneInstanceAttributes
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "SSD File Naming Issue! Neither "
               "SceneInstanceAttributes-provided name : `"
            << ssdFilename << "` nor constructed filename : `"
            << constructedFilename << "` exist on disk.";
        return false;
      }
    }  // if given SSD file name specified exists
  }    // if semantic scene descriptor specified in scene instance

  return false;
}  // ResourceManager::loadSemanticSceneDescriptor

void ResourceManager::buildSemanticColorMap() {
  CORRADE_ASSERT(semanticScene_,
                 "Unable to build Semantic Color map due to no semanticScene "
                 "existing/having been loaded.", );

  semanticColorMapBeingUsed_.clear();
  semanticColorAsInt_.clear();
  const auto& ssdClrMap = semanticScene_->getSemanticColorMap();
  if (ssdClrMap.empty()) {
    return;
  }

  // The color map was built with first maxSemanticID elements in proper order
  // to match provided semantic IDs (so that ID is IDX of semantic color in
  // map). Any overflow colors will be uniquely mapped 1-to-1 to unmapped
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
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Loading Semantic Stage mesh : `" << semanticStageFilename << "`.";
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
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << "Load of Semantic Stage mesh : `" << semanticStageFilename
            << "` failed.";
        return false;
      }
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Semantic Stage mesh : `" << semanticStageFilename << "` loaded.";

    } else if (semanticStageFilename != "") {
      // semantic file name is not found on disk
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Unable to load requested Semantic Stage mesh : `"
          << semanticStageFilename << "` : File not found.";
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
        ESP_ERROR() << "Stage collision mesh load failed. Aborting stage "
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
    bool sceneSuccess =
        _physicsManager->addStage(stageAttributes, stageInstanceAttributes);
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
  std::string debugStr{};
  // Only construct debug string if debug logging level is enabled
  if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
    Cr::Utility::formatInto(debugStr, debugStr.size(),
                            "Frame : {} for Render mesh : `{}`",
                            renderInfo.frame.toString(), renderInfo.filepath);
  }
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
    // Only construct debug string if debug logging level is enabled
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
      Cr::Utility::formatInto(debugStr, debugStr.size(),
                              " and Collision mesh : `{}`",
                              collisionInfo.filepath);
    }

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
    // Only construct debug string if debug logging level is enabled
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
      Cr::Utility::formatInto(
          debugStr, debugStr.size(),
          "|{} for semantic mesh : `{}` of type `{}`|Semantic Txtrs : {}",
          frame.toString(), semanticInfo.filepath,
          esp::metadata::attributes::getMeshTypeName(semanticInfo.type),
          (semanticInfo.hasSemanticTextures ? "True" : "False"));
    }
    resMap["semantic"] = semanticInfo;
  } else {
    // Only construct debug string if debug logging level is enabled
    if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
      Cr::Utility::formatInto(debugStr, debugStr.size(),
                              "|No Semantic asset info specified.");
    }
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
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Specified frame in Attributes `" << attribName
      << "` is not orthogonal, so returning default frame.";
  esp::geo::CoordinateFrame frame;
  return frame;
}  // ResourceManager::buildFrameFromAttributes

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
          << "Unsupported instance creation flags for asset `"
          << assetInfo.filepath << "`.";
      return nullptr;
    }
    sceneID = activeSceneIDs[0];
  } else {
    if (creation.isSemantic() && creation.isRGBD()) {
      if (activeSceneIDs[1] != activeSceneIDs[0]) {
        // Because we have a separate semantic scene graph, we can't support a
        // static instance with both isSemantic and isRGBD.
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Unsupported instance creation flags for asset `"
            << assetInfo.filepath
            << "` with "
               "SimulatorConfiguration::forceSeparateSemanticSceneGraph=true.";
        return nullptr;
      }
      sceneID = activeSceneIDs[0];
    } else {
      if (activeSceneIDs[1] == activeSceneIDs[0]) {
        // A separate semantic scene graph wasn't constructed, so we can't
        // support a Semantic-only (or RGBD-only) instance.
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "Unsupported instance creation flags for asset `"
            << assetInfo.filepath
            << "` with "
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
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Building Prim named `" << info.filepath << "`.";
      buildPrimitiveAssetData(info.filepath);
      meshSuccess = true;
    } else if (info.type == AssetType::INSTANCE_MESH) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Loading Semantic Mesh asset named `" << info.filepath << "`.";
      meshSuccess = loadSemanticRenderAsset(defaultInfo);
    } else if (isRenderAssetGeneral(info.type)) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Loading general asset named `" << info.filepath << "`.";
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
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "Instantiating render asset `" << creation.filepath
        << "` with incompatible light setup, instance will not be correctly "
           "lit. For objects, please ensure 'requires lighting' is enabled in "
           "object config file.";
  }

  const auto& info = loadedAssetData.assetInfo;
  scene::SceneNode* newNode = nullptr;
  if (info.type == AssetType::INSTANCE_MESH) {
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
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Attempting to load stage" << filename << "";
  bool meshSuccess = true;
  if (filename != EMPTY_SCENE) {
    if (!Cr::Utility::Path::exists(filename)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Attempting to load stage but cannot find specified asset file : `"
          << filename << "`. Aborting load.";
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
          ESP_ERROR(Mn::Debug::Flag::NoSpace)
              << "Reloading asset `" << filename
              << "` with different configuration not currently supported."
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
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Loading empty scene since `" << filename
        << "` specified as filename.";
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
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Failed to load a physical object `"
          << objectAttributes->getHandle() << "`'s " << meshType
          << " mesh from file : `" << filename << "`.";
    }
  }
  return success;
}  // loadObjectMeshDataFromFile

Mn::Range3D ResourceManager::computeMeshBB(BaseMesh* meshDataGL) {
  CollisionMeshData& meshData = meshDataGL->getCollisionMeshData();
  return Mn::Math::minmax(meshData.positions);
}

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
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "Attempting to reference or build a "
             "primitive template from an unknown/malformed handle : `"
          << primTemplateHandle << "`, so aborting build.";
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
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Primitive Asset exists already : `" << primAssetHandle << "`.";
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
  meshMetaData.root.materialID = std::to_string(nextMaterialID_++);

  // default material for now
  // Populate with defaults from sim's gfx::PhongMaterialData
  Mn::Trade::MaterialData materialData = buildDefaultPhongMaterial();

  // Set expected user-defined attributes
  materialData = setMaterialDefaultUserAttributes(
      materialData, ObjectInstanceShaderType::Phong);

  shaderManager_.set<Mn::Trade::MaterialData>(meshMetaData.root.materialID,
                                              std::move(materialData));

  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;

  // set the root rotation to world frame upon load
  meshMetaData.setRootFrameOrientation(info.frame);
  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{std::move(info), std::move(meshMetaData)};
  auto inserted =
      resourceDict_.emplace(primAssetHandle, std::move(loadedAssetData));

  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "Primitive Asset Added : ID : " << primTemplate->getID()
      << ": Attributes key : `" << primTemplate->getHandle() << "`| Class : `"
      << primClassName << "` | Importer Conf has group for this obj type : "
      << conf.hasGroup(primClassName);

}  // ResourceManager::buildPrimitiveAssetData

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

    // To access the mesh id
    Cr::Containers::Array<Cr::Containers::Pair<
        Mn::UnsignedInt, Cr::Containers::Pair<Mn::UnsignedInt, Mn::Int>>>
        meshesMaterials = scene->meshesMaterialsAsArray();
    // All the transformations, flattened and indexed by mesh id, with
    // reframeTransform applied to each
    Cr::Containers::Array<Mn::Matrix4> transformations =
        Mn::SceneTools::absoluteFieldTransformations3D(
            *scene, Mn::Trade::SceneField::Mesh, reframeTransform);

    Cr::Containers::Array<Mn::Trade::MeshData> flattenedMeshes;
    Cr::Containers::arrayReserve(flattenedMeshes, meshesMaterials.size());

    for (std::size_t i = 0; i != meshesMaterials.size(); ++i) {
      Mn::UnsignedInt iMesh = meshesMaterials[i].second().first();
      if (Cr::Containers::Optional<Mn::Trade::MeshData> mesh =
              fileImporter.mesh(iMesh)) {
        arrayAppend(flattenedMeshes,
                    Mn::MeshTools::transform3D(*mesh, transformations[i]));
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
  configureImporterManagerGLExtensions();

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
  gfx::DrawableConfiguration drawableConfig{
      creation.lightSetupKey,             // lightSetup Key
      PER_VERTEX_OBJECT_ID_MATERIAL_KEY,  // material key
      ObjectInstanceShaderType::Phong,    // shader type to use
      drawables,                          // drawable group
      nullptr,                            // no skinning data
      nullptr,                            // Not PBR so no IBL map
      nullptr};                           // Not PBR so no PbrShaderAttributes

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
                   meshAttributeFlags,  // mesh attribute flags
                   node,                // scene node
                   drawableConfig);

    if (computeAbsoluteAABBs) {
      staticDrawableInfo.emplace_back(StaticDrawableInfo{node, iMesh});
    }
  }

  if (computeAbsoluteAABBs) {
    computeInstanceMeshAbsoluteAABBs(staticDrawableInfo);
  }

  return instanceRoot;
}  // ResourceManager::createRenderAssetInstanceVertSemantic

void ResourceManager::configureImporterManagerGLExtensions() {
  if (!getCreateRenderer()) {
    return;
  }

  Cr::PluginManager::PluginMetadata* const metadata =
      importerManager_.metadata("BasisImporter");
  if (!metadata)
    return;

  Mn::GL::Context& context = Mn::GL::Context::current();
  /* This is reduced to formats that Magnum currently can Y-flip. More formats
     will get added back with new additions to Magnum/Math/ColorBatch.h. */
#ifdef MAGNUM_TARGET_WEBGL
  if (context.isExtensionSupported<
          Mn::GL::Extensions::WEBGL::compressed_texture_s3tc>())
#elif defined(MAGNUM_TARGET_GLES)
  if (context.isExtensionSupported<
          Mn::GL::Extensions::EXT::texture_compression_s3tc>() ||
      context.isExtensionSupported<
          Mn::GL::Extensions::ANGLE::texture_compression_dxt5>())
#else
  if (context.isExtensionSupported<
          Mn::GL::Extensions::EXT::texture_compression_s3tc>())
#endif
  {
    ESP_DEBUG() << "Importing Basis files as BC3.";
    metadata->configuration().setValue("format", "Bc3RGBA");
  } else {
    ESP_WARNING()
        << "No GPU compressed texture format with Y-flip support detected, "
           "Basis images will get imported as RGBA8.";
    metadata->configuration().setValue("format", "RGBA8");
  }

}  // ResourceManager::configureImporterManagerGLExtensions

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
  configureImporterManagerGLExtensions();

  ESP_CHECK(
      (fileImporter_->openFile(filename) && (fileImporter_->meshCount() > 0u)),
      Cr::Utility::formatString(
          "Error loading general mesh data from file '{}'", filename));

  // load file and add it to the dictionary
  LoadedAssetData loadedAssetData{info};
  if (requiresTextures_) {
    loadTextures(*fileImporter_, loadedAssetData);
    loadMaterials(*fileImporter_, loadedAssetData);
  }
  loadMeshes(*fileImporter_, loadedAssetData);
  loadSkins(*fileImporter_, loadedAssetData);

  auto inserted = resourceDict_.emplace(filename, std::move(loadedAssetData));
  MeshMetaData& meshMetaData = inserted.first->second.meshMetaData;

  // no scenes --- standalone OBJ/PLY files, for example
  // take a wild guess and load the first mesh with the first material
  if (!fileImporter_->sceneCount()) {
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

  /* Load the scene. If no default scene is specified, use the first one. */
  Cr::Containers::Optional<Mn::Trade::SceneData> scene;
  if (!(scene = fileImporter_->scene(fileImporter_->defaultScene() == -1
                                         ? 0
                                         : fileImporter_->defaultScene())) ||
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
    nodes[parent.first()]->name = fileImporter_->objectName(parent.first());
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
    // we've seen, and give them identity transforms.
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

  // If the object has a skin and a rig (articulated object), link them together
  // such as the model bones are driven by the articulated object links.
  std::shared_ptr<gfx::InstanceSkinData> instanceSkinData = nullptr;
  const auto& meshMetaData = loadedAssetData.meshMetaData;
  if (creation.rig && meshMetaData.skinIndex.first != ID_UNDEFINED) {
    ESP_CHECK(
        !skins_.empty(),
        "Cannot instantiate skinned model because no skin data is imported.");
    const auto& skinData = skins_[meshMetaData.skinIndex.first];
    instanceSkinData = std::make_shared<gfx::InstanceSkinData>(skinData);
    mapSkinnedModelToArticulatedObject(meshMetaData.root, creation.rig,
                                       instanceSkinData);
    ESP_CHECK(instanceSkinData->rootArticulatedObjectNode,
              "Could not map skinned model to articulated object.");
  }

  addComponent(meshMetaData,            // mesh metadata
               newNode,                 // parent scene node
               creation.lightSetupKey,  // lightSetup key
               drawables,               // drawable group
               meshMetaData.root,       // mesh transform node
               visNodeCache,  // a vector of scene nodes, the visNodeCache
               computeAbsoluteAABBs,  // compute absolute AABBs
               staticDrawableInfo,    // a vector of static drawable info
               instanceSkinData);     // instance skinning data

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
    ESP_ERROR() << "Cannot build a trajectory from fewer than 2 points, so "
                   "trajectory build failed.";
    return false;
  }

  ESP_VERY_VERBOSE() << "Calling trajectoryTubeSolid to build a tube named"
                     << trajVisName << "with tube radius" << radius << "using"
                     << pts.size() << "points," << numSegments
                     << "circular segments and" << numInterp
                     << "interpolated points between each trajectory point.";

  // create mesh tube
  Cr::Containers::Optional<Mn::Trade::MeshData> trajTubeMesh =
      geo::buildTrajectoryTubeSolid(pts, colorVec, numSegments, radius, smooth,
                                    numInterp);
  ESP_VERY_VERBOSE() << "Successfully returned from trajectoryTubeSolid";

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
  // Populate with defaults from sim's gfx::PhongMaterialData
  Mn::Trade::MaterialData materialData = buildDefaultPhongMaterial();
  // Override default values
  materialData.mutableAttribute<Mn::Color4>(
      Mn::Trade::MaterialAttribute::AmbientColor) = Mn::Color4{1.0};
  materialData.mutableAttribute<Mn::Color4>(
      Mn::Trade::MaterialAttribute::SpecularColor) = Mn::Color4{1.0};
  materialData.mutableAttribute<Mn::Float>(
      Mn::Trade::MaterialAttribute::Shininess) = 160.0f;
  // Set expected user-defined attributes
  materialData = setMaterialDefaultUserAttributes(
      materialData, ObjectInstanceShaderType::Phong, true);

  shaderManager_.set<Mn::Trade::MaterialData>(meshMetaData.root.materialID,
                                              std::move(materialData));

  meshMetaData.root.meshIDLocal = 0;
  meshMetaData.root.componentID = 0;

  // store the rotation to world frame upon load
  meshMetaData.setRootFrameOrientation(info.frame);

  // make LoadedAssetData corresponding to this asset
  LoadedAssetData loadedAssetData{std::move(info), std::move(meshMetaData)};
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
 * congruent with the specified shader type.
 */
bool compareShaderTypeToMnMatType(const ObjectInstanceShaderType typeToCheck,
                                  const Mn::Trade::MaterialData& materialData) {
  switch (typeToCheck) {
    case ObjectInstanceShaderType::Phong: {
      return bool(materialData.types() & Mn::Trade::MaterialType::Phong);
    }
    case ObjectInstanceShaderType::PBR: {
      return bool(materialData.types() &
                  Mn::Trade::MaterialType::PbrMetallicRoughness);
    }
    default: {
      return false;
    }
  }
}  // compareShaderTypeToMnMatType

/**
 * @brief This function will take an existing @ref Mn::Trade::MaterialData and add
 * the missing attributes for the types it does not support, so that it will
 * have attributes for all habitat-supported shader types. This should only be
 * called if the user has specified a desired shader type that the material does
 * not natively support.
 * @param origMaterialData The original material from the importer
 * @return The new material with attribute support for all supported shader
 * types.
 */
Mn::Trade::MaterialData createUniversalMaterial(
    const Mn::Trade::MaterialData& origMaterialData) {
  // create a material, based on the passed material, that will have reasonable
  // attributes to support any possible shader type. should only be called if
  // we are not using the Material's natively specified shaderType

  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;

  // get source attributes from original material
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> newAttributes;

  const auto origMatTypes = origMaterialData.types();
  // add appropriate attributes based on what is missing
  // flat material already recognizes Phong and pbr, so don't have to do
  // anything

  // whether the MaterialAttribute::TextureMatrix has been set already
  bool setTexMatrix = false;
  if (!(origMatTypes & Mn::Trade::MaterialType::Phong)) {
    // add appropriate values for expected attributes to support Phong from
    // PbrMetallicRoughnessMaterialData

    const auto& pbrMaterial =
        origMaterialData.as<Mn::Trade::PbrMetallicRoughnessMaterialData>();

    /////////////////
    // calculate Phong values from PBR material values
    ////////////////

    // derive ambient color from pbr baseColor
    const Mn::Color4 ambientColor = pbrMaterial.baseColor();

    // If there's a roughness texture, we have no way to use it here. The safest
    // fallback is to assume roughness == 1, thus producing no spec highlights.
    // If pbrMaterial does not have a roughness value, it returns a 1 by
    // default.
    const float roughness =
        pbrMaterial.hasRoughnessTexture() ? 1.0f : pbrMaterial.roughness();

    // If there's a metalness texture, we have no way to use it here.
    // The safest fallback is to assume non-metal.
    const float metalness =
        pbrMaterial.hasMetalnessTexture() ? 0.0f : pbrMaterial.metalness();

    // Heuristic to map roughness to spec power.
    // Higher exponent makes the spec highlight larger (lower power)
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.1+%2B+%281+-+x%29%5E4.5+*+180.0+for+x+from+0+to+1
    // lower power for metal
    const float maxShininess = Mn::Math::lerp(250.0f, 120.0f, metalness);
    const float shininess =
        1.1f + Mn::Math::pow(1.0f - roughness, 4.5f) * maxShininess;

    // increase spec intensity for metal
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.0+%2B+10*%28x%5E1.7%29++from+0+to+1
    const float specIntensityScale =
        (metalness > 0.0f
             ? (metalness < 1.0f
                    ? (1.0f + 10.0f * Mn::Math::pow(metalness, 1.7f))
                    : 11.0f)
             : 1.0f);
    // Heuristic to map roughness to spec intensity.
    // higher exponent decreases intensity.
    // example calc :
    // https://www.wolframalpha.com/input/?i=1.4+*+%281-x%29%5E2.5++from+0+to+1
    const float specIntensity =
        Mn::Math::pow(1.0f - roughness, 2.5f) * 1.4f * specIntensityScale;

    const Mn::Color4 diffuseColor = ambientColor;

    // Set spec base color to white or material base color, depending on
    // metalness.
    const Mn::Color4 specBaseColor =
        (metalness > 0.0f
             ? (metalness < 1.0f
                    ? Mn::Math::lerp(0xffffffff_rgbaf, ambientColor,
                                     Mn::Math::pow(metalness, 0.5f))
                    : ambientColor)
             : 0xffffffff_rgbaf);

    const Mn::Color4 specColor = specBaseColor * specIntensity;

    /////////////////
    // set Phong attributes appropriately from precalculated value
    ////////////////
    // normal mapping is already present in copied array if present in
    // original material.

    arrayAppend(newAttributes, {{MaterialAttribute::Shininess, shininess},
                                {MaterialAttribute::AmbientColor, ambientColor},
                                {MaterialAttribute::DiffuseColor, diffuseColor},
                                {MaterialAttribute::SpecularColor, specColor}});
    // texture transforms, if there's none the returned matrix is an
    // identity only copy if we don't already have TextureMatrix
    // attribute (if original pbrMaterial does not have that specific
    // matrix)
    if (!setTexMatrix &&
        (!pbrMaterial.hasAttribute(MaterialAttribute::TextureMatrix))) {
      arrayAppend(newAttributes, {MaterialAttribute::TextureMatrix,
                                  pbrMaterial.commonTextureMatrix()});
      setTexMatrix = true;
    }

    if (pbrMaterial.hasAttribute(MaterialAttribute::BaseColorTexture)) {
      // only provide texture indices if BaseColorTexture attribute
      // exists
      const Mn::UnsignedInt BCTexture = pbrMaterial.baseColorTexture();
      arrayAppend(newAttributes,
                  {{MaterialAttribute::AmbientTexture, BCTexture},
                   {MaterialAttribute::DiffuseTexture, BCTexture}});
      if (metalness >= 0.5) {
        arrayAppend(newAttributes,
                    {MaterialAttribute::SpecularTexture, BCTexture});
      }
    }
  }  // if no phong material support exists in material

  if (!(origMatTypes & Mn::Trade::MaterialType::PbrMetallicRoughness)) {
    // add appropriate values for expected attributes for PbrMetallicRoughness
    // derived from Phong attributes
    const auto& phongMaterial =
        origMaterialData.as<Mn::Trade::PhongMaterialData>();

    /////////////////
    // calculate PBR values from Phong material values
    ////////////////

    // derive base color from Phong diffuse or ambient color, depending on which
    // is present.  set to white if neither is present
    const Mn::Color4 baseColor = phongMaterial.diffuseColor();

    // Experimental metalness heuristic using saturation of spec color
    // to derive approximation of metalness
    const Mn::Color4 specColor = phongMaterial.specularColor();

    // if specColor alpha == 0 then no metalness
    float metalness = 0.0f;
    // otherwise, this hacky heuristic will derive a value for metalness based
    // on how non-grayscale the specular color is (HSV Saturation).
    if (specColor.a() != 0.0f) {
      metalness = specColor.saturation();
    }

    /////////////////
    // set PbrMetallicRoughness attributes appropriately from precalculated
    // values
    ////////////////

    // normal mapping is already present in copied array if present in
    // original material.
    arrayAppend(newAttributes, {{MaterialAttribute::BaseColor, baseColor},
                                {MaterialAttribute::Metalness, metalness}});

    // if diffuse texture is present, use as base color texture in pbr.
    if (phongMaterial.hasAttribute(MaterialAttribute::DiffuseTexture)) {
      uint32_t bcTextureVal = phongMaterial.diffuseTexture();
      arrayAppend(newAttributes,
                  {MaterialAttribute::BaseColorTexture, bcTextureVal});
    }
    // texture transforms, if there's none the returned matrix is an
    // identity Only copy if we don't already have TextureMatrix attribute
    // (if original phongMaterial does not have that specific attribute)
    if (!setTexMatrix &&
        (!phongMaterial.hasAttribute(MaterialAttribute::TextureMatrix))) {
      arrayAppend(newAttributes, {MaterialAttribute::TextureMatrix,
                                  phongMaterial.commonTextureMatrix()});
      // setTexMatrix = true;
    }

    // base texture

  }  // if no PbrMetallicRoughness material support exists in material

  // build flags to support all materials
  constexpr auto flags = Mn::Trade::MaterialType::Flat |
                         Mn::Trade::MaterialType::Phong |
                         Mn::Trade::MaterialType::PbrMetallicRoughness;

  // create new material from attributes array
  Mn::Trade::MaterialData newMaterialData{flags, std::move(newAttributes)};

  return newMaterialData;
}  // namespace

}  // namespace

// Specifically for building materials that relied on old defaults
Mn::Trade::MaterialData ResourceManager::buildDefaultPhongMaterial() {
  Mn::Trade::MaterialData materialData{
      Mn::Trade::MaterialType::Phong,
      {{Mn::Trade::MaterialAttribute::AmbientColor, Mn::Color4{0.1}},
       {Mn::Trade::MaterialAttribute::DiffuseColor, Mn::Color4{0.7}},
       {Mn::Trade::MaterialAttribute::SpecularColor, Mn::Color4{0.2}},
       {Mn::Trade::MaterialAttribute::Shininess, 80.0f}}};
  return materialData;
}  // ResourceManager::buildDefaultPhongMaterial

Mn::Trade::MaterialData ResourceManager::setMaterialDefaultUserAttributes(
    const Mn::Trade::MaterialData& material,
    ObjectInstanceShaderType shaderTypeToUse,
    bool hasVertObjID,
    bool hasTxtrObjID,
    int txtrIdx) const {
  // New material's attributes
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> newAttributes;
  arrayAppend(newAttributes, Cr::InPlaceInit, "hasPerVertexObjectId",
              hasVertObjID);
  if (hasTxtrObjID) {
    arrayAppend(newAttributes, Cr::InPlaceInit, "objectIdTexturePointer",
                textures_.at(txtrIdx).get());
  }
  arrayAppend(newAttributes, Cr::InPlaceInit, "shaderTypeToUse",
              static_cast<int>(shaderTypeToUse));

  Cr::Containers::Optional<Mn::Trade::MaterialData> finalMaterial =
      Mn::MaterialTools::merge(
          material, Mn::Trade::MaterialData{{}, std::move(newAttributes), {}});

  return std::move(*finalMaterial);
}  // ResourceManager::setMaterialDefaultUserAttributes

std::string ResourceManager::createColorMaterial(
    const esp::assets::PhongMaterialColor& materialColor) {
  std::string newMaterialID =
      Cr::Utility::formatString("phong_amb_{}_dif_{}_spec_{}",
                                materialColor.ambientColor.toSrgbAlphaInt(),
                                materialColor.diffuseColor.toSrgbAlphaInt(),
                                materialColor.specularColor.toSrgbAlphaInt());

  auto materialResource =
      shaderManager_.get<Mn::Trade::MaterialData>(newMaterialID);

  if (materialResource.state() == Mn::ResourceState::NotLoadedFallback) {
    // Build a new default phong material
    Mn::Trade::MaterialData materialData = buildDefaultPhongMaterial();
    materialData.mutableAttribute<Mn::Color4>(
        Mn::Trade::MaterialAttribute::AmbientColor) =
        materialColor.ambientColor;
    materialData.mutableAttribute<Mn::Color4>(
        Mn::Trade::MaterialAttribute::DiffuseColor) =
        materialColor.diffuseColor;
    materialData.mutableAttribute<Mn::Color4>(
        Mn::Trade::MaterialAttribute::SpecularColor) =
        materialColor.specularColor;

    // Set expected user-defined attributes
    materialData = setMaterialDefaultUserAttributes(
        materialData, ObjectInstanceShaderType::Phong);
    shaderManager_.set<Mn::Trade::MaterialData>(newMaterialID,
                                                std::move(materialData));
  }
  return newMaterialID;
}  // ResourceManager::createColorMaterial

void ResourceManager::initDefaultMaterials() {
  // Build default phong materials
  Mn::Trade::MaterialData dfltMaterialData = buildDefaultPhongMaterial();
  // Set expected user-defined attributes
  dfltMaterialData = setMaterialDefaultUserAttributes(
      dfltMaterialData, ObjectInstanceShaderType::Phong);
  // Add to shaderManager at specified key location
  shaderManager_.set<Mn::Trade::MaterialData>(DEFAULT_MATERIAL_KEY,
                                              std::move(dfltMaterialData));
  // Build white material
  Mn::Trade::MaterialData whiteMaterialData = buildDefaultPhongMaterial();
  whiteMaterialData.mutableAttribute<Mn::Color4>(
      Mn::Trade::MaterialAttribute::AmbientColor) = Mn::Color4{1.0};
  // Set expected user-defined attributes
  whiteMaterialData = setMaterialDefaultUserAttributes(
      whiteMaterialData, ObjectInstanceShaderType::Phong);
  // Add to shaderManager at specified key location
  shaderManager_.set<Mn::Trade::MaterialData>(WHITE_MATERIAL_KEY,
                                              std::move(whiteMaterialData));
  // Buiild white vertex ID material
  Mn::Trade::MaterialData vertIdMaterialData = buildDefaultPhongMaterial();
  vertIdMaterialData.mutableAttribute<Mn::Color4>(
      Mn::Trade::MaterialAttribute::AmbientColor) = Mn::Color4{1.0};
  // Set expected user-defined attributes
  vertIdMaterialData = setMaterialDefaultUserAttributes(
      vertIdMaterialData, ObjectInstanceShaderType::Phong, true);
  // Add to shaderManager at specified key location
  shaderManager_.set<Mn::Trade::MaterialData>(PER_VERTEX_OBJECT_ID_MATERIAL_KEY,
                                              std::move(vertIdMaterialData));

  // Build default material for fallback material
  auto fallBackMaterial = buildDefaultPhongMaterial();
  // Set expected user-defined attributes
  fallBackMaterial = setMaterialDefaultUserAttributes(
      fallBackMaterial, ObjectInstanceShaderType::Phong);
  // Add to shaderManager as fallback material
  shaderManager_.setFallback<Mn::Trade::MaterialData>(
      std::move(fallBackMaterial));
}  // ResourceManager::initDefaultMaterials

void ResourceManager::loadMaterials(Importer& importer,
                                    LoadedAssetData& loadedAssetData) {
  // Specify the shaderType to use to render the materials being imported
  ObjectInstanceShaderType shaderTypeToUse =
      getMaterialShaderType(loadedAssetData.assetInfo);

  const std::string shaderTypeToUseName =
      metadata::attributes::getShaderTypeName(shaderTypeToUse);

  // name of asset, for debugging purposes
  const std::string assetName =
      Cr::Utility::Path::split(loadedAssetData.assetInfo.filepath).second();
  int numMaterials = importer.materialCount();
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Building " << numMaterials << " materials for asset named '"
      << assetName << "' : ";

  // starting index for all textures corresponding to this material
  int textureBaseIndex = loadedAssetData.meshMetaData.textureIndex.first;

  if (loadedAssetData.assetInfo.hasSemanticTextures) {
    for (int iMaterial = 0; iMaterial < numMaterials; ++iMaterial) {
      // Build material key
      std::string materialKey = std::to_string(nextMaterialID_++);
      // Retrieving the material just to verify it exists
      Cr::Containers::Optional<Mn::Trade::MaterialData> materialData =
          importer.material(iMaterial);

      if (!materialData) {
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << "Material load failed for index " << iMaterial
            << " so skipping that material for asset `" << assetName << "`.";
        continue;
      }
      // Semantic texture-based mapping

      // Build a phong material for semantics.  TODO: Should this be a
      // FlatMaterialData? Populate with defaults from deprecated
      // gfx::PhongMaterialData
      Mn::Trade::MaterialData newMaterialData = buildDefaultPhongMaterial();
      // Override default values
      newMaterialData.mutableAttribute<Mn::Color4>(
          Mn::Trade::MaterialAttribute::AmbientColor) = Mn::Color4{1.0};
      newMaterialData.mutableAttribute<Mn::Color4>(
          Mn::Trade::MaterialAttribute::DiffuseColor) = Mn::Color4{};
      newMaterialData.mutableAttribute<Mn::Color4>(
          Mn::Trade::MaterialAttribute::SpecularColor) = Mn::Color4{};

      // Set expected user-defined attributes - force to use phong shader for
      // semantics
      newMaterialData = setMaterialDefaultUserAttributes(
          newMaterialData, ObjectInstanceShaderType::Phong, false, true,
          textureBaseIndex + iMaterial);

      shaderManager_.set<Mn::Trade::MaterialData>(materialKey,
                                                  std::move(newMaterialData));
    }
  } else {
    for (int iMaterial = 0; iMaterial < numMaterials; ++iMaterial) {
      // Build material key
      std::string materialKey = std::to_string(nextMaterialID_++);

      Cr::Containers::Optional<Mn::Trade::MaterialData> materialData =
          importer.material(iMaterial);

      if (!materialData) {
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << "Material load failed for index " << iMaterial
            << " so skipping that material for asset `" << assetName << "`.";
        continue;
      }

      int numMaterialLayers = materialData->layerCount();

      // If we are not using the material's native shadertype, or flat
      // (Which all materials already support), expand the
      // Mn::Trade::MaterialData with appropriate data for all possible
      // shadertypes

      std::string materialExpandStr = shaderTypeToUseName;
      if ((shaderTypeToUse != ObjectInstanceShaderType::Material) &&
          (shaderTypeToUse != ObjectInstanceShaderType::Flat) &&
          !(compareShaderTypeToMnMatType(shaderTypeToUse, *materialData))) {
        // Only create this string if veryverbose logging is enabled
        if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
          materialExpandStr = Cr::Utility::formatString(
              "Forcing to {} shader (material requires expansion to support it "
              "via createUniversalMaterial)",
              shaderTypeToUseName);
        }

        materialData = createUniversalMaterial(*materialData);
      }

      // This material data has any per-shader as well as global custom
      // user-defined attributes set excluding texture pointer mappings
      Corrade::Containers::Optional<Mn::Trade::MaterialData> custMaterialData;
      Cr::Containers::StringView shaderBeingUsed;
      // Build based on desired shader to use
      // pbr shader spec, of material-specified and material specifies pbr
      if ((checkForPassedShaderType(
              shaderTypeToUse, *materialData, ObjectInstanceShaderType::PBR,
              Mn::Trade::MaterialType::PbrMetallicRoughness)) ||
          (checkForPassedShaderType(shaderTypeToUse, *materialData,
                                    ObjectInstanceShaderType::PBR,
                                    Mn::Trade::MaterialType::PbrClearCoat))) {
        // Material with custom settings appropriately set for PBR material
        custMaterialData =
            buildCustomAttributePbrMaterial(*materialData, textureBaseIndex);
        shaderBeingUsed = "PBR";
        // phong shader spec, of material-specified and material specifies phong
      } else if (checkForPassedShaderType(shaderTypeToUse, *materialData,
                                          ObjectInstanceShaderType::Phong,
                                          Mn::Trade::MaterialType::Phong)) {
        // Material with custom settings appropriately set for Phong material
        custMaterialData =
            buildCustomAttributePhongMaterial(*materialData, textureBaseIndex);
        shaderBeingUsed = "Phong";
        // flat shader spec or material-specified and material specifies flat
      } else if (checkForPassedShaderType(shaderTypeToUse, *materialData,
                                          ObjectInstanceShaderType::Flat,
                                          Mn::Trade::MaterialType::Flat)) {
        // Material with custom settings appropriately set for Flat materials to
        // be used in our Phong shader
        custMaterialData =
            buildCustomAttributeFlatMaterial(*materialData, textureBaseIndex);
        shaderBeingUsed = "Flat";
      } else {
        ESP_CHECK(
            false,
            Cr::Utility::formatString(
                "Unhandled ShaderType specification : {} and/or unmanaged "
                "type specified in material @ idx: {} for asset `{}`.",
                shaderTypeToUseName, iMaterial, assetName));
      }

      std::string debugStr;
      if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
        debugStr = Cr::Utility::formatString(
            "Idx {:.02d} has {:.02} layers| shader being used: {} for: {}.",
            iMaterial, numMaterialLayers, shaderBeingUsed, materialExpandStr);
      }
      // Merge all custom attribute except remapped texture pointers with
      // original material for final material. custMaterialData should never be
      // Cr::Containers::NullOpt since every non-error branch is covered.
      Cr::Containers::Optional<Mn::Trade::MaterialData> mergedCustomMaterial =
          Mn::MaterialTools::merge(
              *custMaterialData, *materialData,
              Mn::MaterialTools::MergeConflicts::KeepFirstIfSameType);

      // Now build texture pointer array, with appropriate layers based on
      // number of layers in original material

      // New txtrptr-holding material's attributes
      Cr::Containers::Array<Mn::Trade::MaterialAttributeData> newAttributes{};
      // New txtrptr-holding material's layers
      Cr::Containers::Array<Mn::UnsignedInt> newLayers{};

      // Copy all texture pointers into array
      // list of all this material's attributes
      const Cr::Containers::ArrayView<const Mn::Trade::MaterialAttributeData>
          materialAttributes = materialData->attributeData();
      // Returns nullptr if has no attributes
      if (materialAttributes != nullptr) {
        // For all layers
        for (int layerIdx = 0; layerIdx < numMaterialLayers; ++layerIdx) {
          // Add all material texture pointers into new attributes
          // find start and end idxs for each layer
          int stIdx = materialData->attributeDataOffset(layerIdx);
          int endIdx = materialData->attributeDataOffset(layerIdx + 1);

          for (int mIdx = stIdx; mIdx < endIdx; ++mIdx) {
            const Mn::Trade::MaterialAttributeData& materialAttribute =
                materialAttributes[mIdx];
            auto attrName = materialAttribute.name();
            const auto matType = materialAttribute.type();
            // Find textures and add them to newAttributes
            // bool found = (std::string::npos != key.find(strToLookFor));
            if ((matType == Mn::Trade::MaterialAttributeType::UnsignedInt) &&
                attrName.hasSuffix("Texture")) {
              // texture index, copy texture pointer into newAttributes
              const Mn::UnsignedInt txtrIdx =
                  materialAttribute.value<Mn::UnsignedInt>();
              // copy texture into new attributes tagged with lowercase material
              // name
              auto newAttrName = Cr::Utility::formatString(
                  "{}{}Pointer",
                  Cr::Utility::String::lowercase(attrName.slice(0, 1)),
                  attrName.slice(1, attrName.size()));

              // Debug display of layer pointers
              if (ESP_LOG_LEVEL_ENABLED(logging::LoggingLevel::Debug)) {
                Cr::Utility::formatInto(debugStr, debugStr.size(),
                                        "| txtr ptr name:{} | idx :{} Layer {}",
                                        newAttrName,
                                        (textureBaseIndex + txtrIdx), layerIdx);
              }
              arrayAppend(newAttributes,
                          {newAttrName,
                           textures_.at(textureBaseIndex + txtrIdx).get()});
            }  // if texture found
          }    // for each material attribute in layer
          // Save this layer's offset
          arrayAppend(newLayers, newAttributes.size());
        }  // for each layer
      }    // if material has attributes

      // Merge all texture-pointer custom attributes with material holding
      // original attributes + non-texture-pointer custom attributes for final
      // material
      Cr::Containers::Optional<Mn::Trade::MaterialData> finalMaterial =
          Mn::MaterialTools::merge(
              *mergedCustomMaterial,
              Mn::Trade::MaterialData{
                  {}, std::move(newAttributes), std::move(newLayers)});

      ESP_DEBUG() << debugStr;
      // for now, just use unique ID for material key. This may change if we
      // expose materials to user for post-load modification

      shaderManager_.set<Mn::Trade::MaterialData>(materialKey,
                                                  std::move(*finalMaterial));
    }
  }
}  // ResourceManager::loadMaterials

Mn::Trade::MaterialData ResourceManager::buildCustomAttributeFlatMaterial(
    const Mn::Trade::MaterialData& materialData,
    int textureBaseIndex) {
  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;
  // Custom/remapped attributes for material, to match required Phong shader
  // mapping.
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> custAttributes;

  // To save on shader switching, a Phong shader with zero lights is used for
  // flat materials. This requires custom mapping of material quantities so that
  // the Phong shader can find what it is looking for.
  const auto& flatMat = materialData.as<Mn::Trade::FlatMaterialData>();
  // Populate base/diffuse color and texture (if present) into flat
  // material array
  arrayAppend(
      custAttributes,
      {// Set ambient color from flat material's base/diffuse color
       {Mn::Trade::MaterialAttribute::AmbientColor, flatMat.color()},
       // Clear out diffuse and specular colors for flat material
       {Mn::Trade::MaterialAttribute::DiffuseColor, 0x00000000_rgbaf},
       // No default shininess in Magnum materials
       {Mn::Trade::MaterialAttribute::Shininess, 80.0f},

       {Mn::Trade::MaterialAttribute::SpecularColor, 0x00000000_rgbaf}});
  // Only populate into ambient texture if present in original
  // material
  if (flatMat.hasTexture()) {
    arrayAppend(custAttributes,
                {"ambientTexturePointer",
                 textures_.at(textureBaseIndex + flatMat.texture()).get()});
  }
  // Merge new attributes with those specified in original material
  // overridding original ambient, diffuse and specular colors
  // Using owning MaterialData constructor to handle potential
  // layers
  auto finalMaterial = Mn::MaterialTools::merge(
      Mn::Trade::MaterialData{{}, std::move(custAttributes), {}}, materialData,
      Mn::MaterialTools::MergeConflicts::KeepFirstIfSameType);

  // Set default, expected user attributes for the final material
  // and return
  return setMaterialDefaultUserAttributes(*finalMaterial,
                                          ObjectInstanceShaderType::Flat);
}  // ResourceManager::buildFlatShadedMaterialData

Mn::Trade::MaterialData ResourceManager::buildCustomAttributePhongMaterial(
    const Mn::Trade::MaterialData& materialData,
    CORRADE_UNUSED int textureBaseIndex) const {
  // Custom/remapped attributes for material, to match required Phong shader
  // mapping.
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> custAttributes;

  // TODO : specify custom non-texture pointer mappings for Phong materials
  // here.

  // Merge new attributes with those specified in original material
  // overridding original ambient, diffuse and specular colors
  if (custAttributes.size() > 0) {
    // Using owning MaterialData constructor to handle potential layers
    auto finalMaterial = Mn::MaterialTools::merge(
        Mn::Trade::MaterialData{{}, std::move(custAttributes), {}},
        materialData, Mn::MaterialTools::MergeConflicts::KeepFirstIfSameType);

    // Set default, expected user attributes for the final material and return
    return setMaterialDefaultUserAttributes(*finalMaterial,
                                            ObjectInstanceShaderType::Phong);
  }
  return setMaterialDefaultUserAttributes(materialData,
                                          ObjectInstanceShaderType::Phong);

}  // ResourceManager::buildPhongShadedMaterialData

Mn::Trade::MaterialData ResourceManager::buildCustomAttributePbrMaterial(
    const Mn::Trade::MaterialData& materialData,
    CORRADE_UNUSED int textureBaseIndex) const {
  // Custom/remapped attributes for material, to match required PBR shader
  // mapping.
  Cr::Containers::Array<Mn::Trade::MaterialAttributeData> custAttributes;

  // TODO : specify custom non-texture pointer mappings for PBR materials
  // here.

  // Merge new attributes with those specified in original material
  // overridding original ambient, diffuse and specular colors

  if (custAttributes.size() > 0) {
    // Using owning MaterialData constructor to handle potential layers
    auto finalMaterial = Mn::MaterialTools::merge(
        Mn::Trade::MaterialData{{}, std::move(custAttributes), {}},
        materialData, Mn::MaterialTools::MergeConflicts::KeepFirstIfSameType);

    // Set default, expected user attributes for the final material and return
    return setMaterialDefaultUserAttributes(*finalMaterial,
                                            ObjectInstanceShaderType::PBR);
  }
  return setMaterialDefaultUserAttributes(materialData,
                                          ObjectInstanceShaderType::PBR);
}  // ResourceManager::buildPbrShadedMaterialData

ObjectInstanceShaderType ResourceManager::getMaterialShaderType(
    const AssetInfo& info) const {
  // if specified to be force-flat, then should be flat shaded, regardless of
  // material or other settings.
  if (info.forceFlatShading) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Asset `" << Cr::Utility::Path::split(info.filepath).second()
        << "` is being forced to use Flat shader by configuration.";
    return ObjectInstanceShaderType::Flat;
  }
  ObjectInstanceShaderType infoSpecShaderType = info.shaderTypeToUse;

  if (infoSpecShaderType == ObjectInstanceShaderType::Unspecified) {
    // use the material's inherent shadertype
    infoSpecShaderType = ObjectInstanceShaderType::Material;
  }
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "Asset `" << Cr::Utility::Path::split(info.filepath).second()
      << "` is using shadertype `"
      << metadata::attributes::getShaderTypeName(infoSpecShaderType) << "`.";
  return infoSpecShaderType;
}  // ResourceManager::getMaterialShaderType

bool ResourceManager::checkForPassedShaderType(
    const ObjectInstanceShaderType typeToCheck,
    const Mn::Trade::MaterialData& materialData,
    const ObjectInstanceShaderType verificationType,
    const Mn::Trade::MaterialType mnVerificationType) const {
  return (
      (typeToCheck == verificationType) ||
      ((typeToCheck == ObjectInstanceShaderType::Material) &&
       ((materialData.types() & mnVerificationType) == mnVerificationType)));
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

void ResourceManager::loadSkins(Importer& importer,
                                LoadedAssetData& loadedAssetData) {
  if (importer.skin3DCount() == 0)
    return;

  const int skinStart = nextSkinID_;
  const int skinEnd = skinStart + importer.skin3DCount() - 1;
  nextSkinID_ = skinEnd + 1;
  loadedAssetData.meshMetaData.setSkinIndices(skinStart, skinEnd);

  for (int iSkin = 0; iSkin < importer.skin3DCount(); ++iSkin) {
    auto skinData = std::make_shared<gfx::SkinData>();

    Cr::Containers::Optional<Mn::Trade::SkinData3D> skin =
        importer.skin3D(iSkin);
    CORRADE_INTERNAL_ASSERT(skin);

    // Cache bone names for later association with instance transforms
    for (auto jointIt : skin->joints()) {
      const auto gfxBoneName = fileImporter_->objectName(jointIt);
      skinData->boneNameJointIdMap[gfxBoneName] = jointIt;
    }

    skinData->skin = std::make_shared<Mn::Trade::SkinData3D>(std::move(*skin));
    skins_.emplace(skinStart + iSkin, std::move(skinData));
  }
}  // ResourceManager::loadSkins

Mn::Image2D ResourceManager::convertRGBToSemanticId(
    const Mn::ImageView2D& srcImage,
    Cr::Containers::Array<Mn::UnsignedShort>& clrToSemanticId) {
  // convert image to semantic image here

  const Mn::Vector2i size = srcImage.size();
  // construct empty integer image
  Mn::Image2D resImage{
      Mn::PixelFormat::R16UI, size,
      Cr::Containers::Array<char>{
          Mn::NoInit, std::size_t(size.product() *
                                  pixelFormatSize(Mn::PixelFormat::R16UI))}};

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
        ESP_ERROR() << "Cannot load texture" << iTexture << "so skipping";
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
        ESP_ERROR() << "Cannot load semantic texture image, skipping";
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
        ESP_ERROR() << "Cannot load texture" << iTexture << "so skipping";
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
          const auto pixelFormat = image->format();
          format = Mn::GL::textureFormat(pixelFormat);
          // Modify swizzle for single channel textures so that they are
          // greyscale
          Mn::UnsignedInt channelCount = pixelFormatChannelCount(pixelFormat);
          if (channelCount == 1) {
#ifdef MAGNUM_TARGET_WEBGL
            ESP_WARNING() << "Single Channel Greyscale Texture : ID" << iTexture
                          << "incorrectly displays as red instead of "
                             "greyscale due to greyscale expansion"
                             "not yet implemented in WebGL.";
#else
            currentTexture->setSwizzle<'r', 'r', 'r', '1'>();
#endif
          } else if (channelCount == 2) {
#ifdef MAGNUM_TARGET_WEBGL
            ESP_WARNING() << "Two Channel Greyscale + Alpha Texture : ID"
                          << iTexture
                          << "incorrectly displays due to greyscale expansion"
                             "not yet implemented in WebGL.";
#else
            currentTexture->setSwizzle<'r', 'r', 'r', 'g'>();
#endif
          }
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
        (getObjectAttributesManager()->registerObject(
             objectAttributes, objectTemplateHandle) != ID_UNDEFINED),
        "::instantiateAssetsOnDemand : Unknown failure "
        "attempting to register modified template :"
            << objectTemplateHandle << "before asset instantiation. Aborting. ",
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
                    << "so instantiateAssetsOnDemand of primitive-based render "
                       "object failed.";
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

const std::vector<assets::CollisionMeshData>& ResourceManager::getCollisionMesh(
    const std::string& collisionAssetHandle) const {
  auto colMeshGroupIter = collisionMeshGroups_.find(collisionAssetHandle);
  CORRADE_INTERNAL_ASSERT(colMeshGroupIter != collisionMeshGroups_.end());
  return colMeshGroupIter->second;
}

metadata::managers::AssetAttributesManager::ptr
ResourceManager::getAssetAttributesManager() const {
  return metadataMediator_->getAssetAttributesManager();
}

metadata::managers::LightLayoutAttributesManager::ptr
ResourceManager::getLightLayoutAttributesManager() const {
  return metadataMediator_->getLightLayoutAttributesManager();
}

metadata::managers::ObjectAttributesManager::ptr
ResourceManager::getObjectAttributesManager() const {
  return metadataMediator_->getObjectAttributesManager();
}

metadata::managers::AOAttributesManager::ptr
ResourceManager::getAOAttributesManager() const {
  return metadataMediator_->getAOAttributesManager();
}

metadata::managers::PhysicsAttributesManager::ptr
ResourceManager::getPhysicsAttributesManager() const {
  return metadataMediator_->getPhysicsAttributesManager();
}

metadata::managers::StageAttributesManager::ptr
ResourceManager::getStageAttributesManager() const {
  return metadataMediator_->getStageAttributesManager();
}

const MeshMetaData& ResourceManager::getMeshMetaData(
    const std::string& metaDataName) const {
  auto resDictMDIter = resourceDict_.find(metaDataName);
  CORRADE_INTERNAL_ASSERT(resDictMDIter != resourceDict_.end());
  return resDictMDIter->second.meshMetaData;
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
    std::vector<StaticDrawableInfo>& staticDrawableInfo,
    const std::shared_ptr<gfx::InstanceSkinData>& skinData /* = nullptr */) {
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
      CORRADE_ASSERT(mesh, "addComponent() : GL mesh expected but not found", );
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

    auto material = shaderManager_.get<Mn::Trade::MaterialData>(materialKey);

    ObjectInstanceShaderType materialDataType =
        static_cast<ObjectInstanceShaderType>(
            material->mutableAttribute<int>("shaderTypeToUse"));

    // If shader type to use has not been explicitly specified, use best shader
    // supported by material
    if ((materialDataType < ObjectInstanceShaderType::Flat) ||
        (materialDataType > ObjectInstanceShaderType::PBR)) {
      const auto types = material->types();
      if (types >= Mn::Trade::MaterialType::PbrMetallicRoughness) {
        materialDataType = ObjectInstanceShaderType::PBR;
      } else {
        materialDataType = ObjectInstanceShaderType::Phong;
      }
    }
    gfx::DrawableConfiguration drawableConfig{
        lightSetupKey,     // lightSetup Key
        materialKey,       // material key
        materialDataType,  // shader type to use
        drawables,         // drawable group
        skinData,          // instance skinning data
        nullptr,           // PbrIBLHelper - set only if PBR
        nullptr};          // PbrShaderAttributes - set only if PBR

    if (materialDataType == ObjectInstanceShaderType::PBR) {
      // TODO : Query which PbrShaderAttributes to use based on `region` of
      // drawable. Need to have some way of propagating region value - perhaps
      // through scene node like semantic ID, although semantic ID of objects is
      // not present yet.

      // Currently always using default PbrShaderAttributes for the current
      // Scene.
      esp::metadata::attributes::PbrShaderAttributes::ptr pbrAttributesPtr =
          metadataMediator_->getDefaultPbrShaderConfig();
      // get pointer to PbrIBL Helper for this attributes, creating the helper
      // if it does not exist. Should always exist by here, unless a new or
      // modified PbrShaderAttributes had been added to the library by the user.
      std::shared_ptr<gfx::PbrIBLHelper> pbrIblData_ =
          getOrBuildPBRIBLHelper(pbrAttributesPtr);

      drawableConfig.setPbrIblData(pbrIblData_);
      drawableConfig.setPbrShaderConfig(pbrAttributesPtr);
    }  // if pbr, add appropriate config information

    createDrawable(mesh,                // render mesh
                   meshAttributeFlags,  // mesh attribute flags
                   node,                // scene node
                   drawableConfig);     // instance skinning data

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
                 staticDrawableInfo,    // a vector of static drawable info
                 skinData);             // instance skinning data
  }
}  // addComponent

void ResourceManager::mapSkinnedModelToArticulatedObject(
    const MeshTransformNode& meshTransformNode,
    const std::shared_ptr<physics::ArticulatedObject>& rig,
    const std::shared_ptr<gfx::InstanceSkinData>& skinData) {
  // Find skin joint ID that matches the node
  const auto& gfxBoneName = meshTransformNode.name;
  const auto& boneNameJointIdMap = skinData->skinData->boneNameJointIdMap;
  const auto jointIt = boneNameJointIdMap.find(gfxBoneName);
  if (jointIt != boneNameJointIdMap.end()) {
    int jointId = jointIt->second;

    // Find articulated object link ID that matches the node
    const auto linkIds = rig->getLinkIdsWithBase();
    const auto linkId =
        std::find_if(linkIds.begin(), linkIds.end(),
                     [&](int i) { return gfxBoneName == rig->getLinkName(i); });

    // Map the articulated object link associated with the skin joint
    if (linkId != linkIds.end()) {
      auto* articulatedObjectNode = &rig->getLink(*linkId.base()).node();

      // This node will be used for rendering.
      auto& transformNode = articulatedObjectNode->createChild();
      skinData->jointIdToTransformNode[jointId] = &transformNode;

      // First node found is the root
      if (!skinData->rootArticulatedObjectNode) {
        skinData->rootArticulatedObjectNode = articulatedObjectNode;
      }
    }
  }

  for (const auto& child : meshTransformNode.children) {
    mapSkinnedModelToArticulatedObject(child, rig, skinData);
  }
}  // mapSkinnedModelToArticulatedObject

void ResourceManager::addPrimitiveToDrawables(int primitiveID,
                                              scene::SceneNode& node,
                                              DrawableGroup* drawables) {
  auto primMeshIter = primitive_meshes_.find(primitiveID);
  CORRADE_INTERNAL_ASSERT(primMeshIter != primitive_meshes_.end());
  gfx::DrawableConfiguration drawableConfig{
      NO_LIGHT_KEY,                    // lightSetupKey
      WHITE_MATERIAL_KEY,              // materialDataKey
      ObjectInstanceShaderType::Flat,  // shader to use
      drawables,                       // DrawableGroup
      nullptr,                         // No skinData
      nullptr,                         // No PbrIBLHelper for flat/phong
      nullptr};                        // PbrShaderAttributes for flat/phong
  // TODO:
  // currently we assume the primitives do not have normal texture
  // so do not need to worry about the tangent or bitangent.
  // it might be changed in the future.
  // NOTE : TBN frame is synthesized in PBR shader if precomputed tangent frame
  // is not provided, but this is not done when using Phong/Flat Shader.
  gfx::Drawable::Flags meshAttributeFlags{};
  createDrawable(primMeshIter->second.get(),  // render mesh
                 meshAttributeFlags,          // meshAttributeFlags
                 node,                        // sceneNode
                 drawableConfig);             // configuration for drawable
}

void ResourceManager::removePrimitiveMesh(int primitiveID) {
  auto primMeshIter = primitive_meshes_.find(primitiveID);
  CORRADE_INTERNAL_ASSERT(primMeshIter != primitive_meshes_.end());
  primitive_meshes_.erase(primMeshIter);
}

void ResourceManager::createDrawable(Mn::GL::Mesh* mesh,
                                     gfx::Drawable::Flags& meshAttributeFlags,
                                     scene::SceneNode& node,
                                     gfx::DrawableConfiguration& drawableCfg) {
  switch (drawableCfg.materialDataType_) {
    case ObjectInstanceShaderType::Flat:
    case ObjectInstanceShaderType::Phong:
      node.addFeature<gfx::GenericDrawable>(
          mesh,                // render mesh
          meshAttributeFlags,  // mesh attribute flags
          shaderManager_,      // shader manager
          drawableCfg);
      break;
    case ObjectInstanceShaderType::PBR:
      node.addFeature<gfx::PbrDrawable>(
          mesh,                // render mesh
          meshAttributeFlags,  // mesh attribute flags
          shaderManager_,      // shader manager
          drawableCfg);
      break;
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  drawableCountAndNumFaces_.first += 1;
  if (mesh) {
    drawableCountAndNumFaces_.second += mesh->count() / 3;
  }

}  // ResourceManager::createDrawable

void ResourceManager::initDefaultLightSetups() {
  shaderManager_.set(NO_LIGHT_KEY, gfx::LightSetup{});
  shaderManager_.setFallback(gfx::LightSetup{});
}

std::shared_ptr<gfx::PbrIBLHelper> ResourceManager::getOrBuildPBRIBLHelper(
    const std::shared_ptr<esp::metadata::attributes::PbrShaderAttributes>&
        pbrShaderAttr) {
  auto helperKey = pbrShaderAttr->getPbrShaderHelperKey();

  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Handle :`" << pbrShaderAttr->getHandle()
      << "` : PbrIBLHelper key : (brdfLUT Handle)_(envMap Handle) `"
      << helperKey << "`";
  // Try to find image in IBL texture library
  std::unordered_map<
      std::string, std::shared_ptr<gfx::PbrIBLHelper>>::const_iterator mapIter =
      pbrIBLHelpers_.find(helperKey);

  std::shared_ptr<gfx::PbrIBLHelper> pbrIBLHelper = nullptr;
  if (mapIter != pbrIBLHelpers_.end()) {
    // If found don't reload/remake
    pbrIBLHelper = mapIter->second;
  } else {
    // PbrIBLHelper not found so build it.
    // First verify that pbr image resources are available, and load if not
    // TODO should we consider this resource as a method variable?
    if (!Cr::Utility::Resource::hasGroup("pbr-images")) {
      importPbrImageResources();
    }
    const Cr::Utility::Resource rs{"pbr-images"};

    std::shared_ptr<Mn::GL::Texture2D> blutTexture = nullptr;
    std::shared_ptr<Mn::GL::Texture2D> envMapTexture = nullptr;
    // ==== load the brdf lookup table texture ====
    auto bLUTImageFilename = pbrShaderAttr->getIBLBrdfLUTAssetHandle();
    if (!bLUTImageFilename.empty()) {
      // Only loads if bLUTImageFilename hasn't been loaded already.
      // Also caches texture
      blutTexture = loadIBLImageIntoTexture(bLUTImageFilename, false, rs);
    }

    // ==== load the equirectangular texture ====
    auto envMapFilename = pbrShaderAttr->getIBLEnvMapAssetHandle();
    if (!envMapFilename.empty()) {
      // Only loads if envMapFilename hasn't been loaded already.
      // Also caches texture
      envMapTexture = loadIBLImageIntoTexture(envMapFilename, true, rs);
    }

    // ==== build helper for these assets ====
    // This helper uses shaders to perform the calculations required to
    // convert the Environment Map equirectangular image into the Irradiance
    // Map and Prefiltered Environment Map CubeMaps. Only build if both
    // images were found and successfully converted into textures.

    if (blutTexture && envMapTexture) {
      pbrIBLHelper = std::make_shared<gfx::PbrIBLHelper>(
          shaderManager_, blutTexture, envMapTexture);
      pbrIBLHelpers_.emplace(helperKey, pbrIBLHelper);
    }
  }  // if found else create
  return pbrIBLHelper;

}  // ResourceManager::buildPBRIBLHelper

std::shared_ptr<Mn::GL::Texture2D> ResourceManager::loadIBLImageIntoTexture(
    const std::string& imageFilename,
    bool useImageTxtrFormat,
    const Cr::Utility::Resource& rs) {
  if (imageFilename.empty()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Failed loading "
        << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
        << " image file due to empty name, so aborting.";
    return nullptr;
  }

  // Try to find image in IBL texture library
  std::unordered_map<
      std::string, std::shared_ptr<Mn::GL::Texture2D>>::const_iterator mapIter =
      iblBLUTsAndEnvMaps_.find(imageFilename);

  std::shared_ptr<Mn::GL::Texture2D> resTexture;
  if (mapIter != iblBLUTsAndEnvMaps_.end()) {
    // If found don't reload
    resTexture = mapIter->second;
  } else {
    // load image
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Checking if IBL "
        << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
        << " image file `" << imageFilename << "` is in compiled resource.";
    if (rs.hasFile(imageFilename)) {
      ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
          << "IBL "
          << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
          << " image file `" << imageFilename
          << "` exists in compiled resource.";
      imageImporter_->openData(rs.getRaw(imageFilename));
    } else {
      // Not found as-is in resource, try with directory prefix and then
      // search in filesystem
      const std::string prefixedImageFilename = Cr::Utility::formatString(
          "{}/{}", (useImageTxtrFormat ? "env_maps" : "bluts"), imageFilename);
      if (rs.hasFile(prefixedImageFilename)) {
        ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
            << "IBL "
            << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
            << " image file `" << prefixedImageFilename
            << "` exists in compiled resource.";
        imageImporter_->openData(rs.getRaw(prefixedImageFilename));
      } else {
        ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
            << "IBL "
            << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
            << " image file `" << imageFilename
            << "` was not found in resource so attempting to load from disk.";
        // TODO verify file exists on disk before attempting to load.

        if (!imageImporter_->openFile(imageFilename)) {
          // If unable to load then not found
          ESP_ERROR(Mn::Debug::Flag::NoSpace)
              << "Requested IBL "
              << (useImageTxtrFormat ? "Environment Map" : "BRDF Lookup Table")
              << " image file (required for IBL rendering) named `"
              << imageFilename
              << "` not found in resource file or on disk, so skipping load.";
          return nullptr;
        }
      }
    }
    Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
        imageImporter_->image2D(0);

    // sanity check
    CORRADE_INTERNAL_ASSERT(imageData);

    // brdfLUT should use RGBA8, based on past work
    // envMap should use image's format
    Mn::GL::TextureFormat textureFmtToUse =
        useImageTxtrFormat ? Mn::GL::textureFormat(imageData->format())
                           : Mn::GL::TextureFormat::RGBA8;
    // appropriately configure target texture and map image into it.
    resTexture = std::make_shared<Mn::GL::Texture2D>();
    (*resTexture)
        .setMinificationFilter(Mn::GL::SamplerFilter::Linear)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, textureFmtToUse, imageData->size());

    if (!imageData->isCompressed()) {
      resTexture->setSubImage(0, {}, *imageData);
    } else {
      resTexture->setCompressedSubImage(0, {}, *imageData);
    }
    // add to IBL texture library
    iblBLUTsAndEnvMaps_.emplace(imageFilename, resTexture);
  }
  return resTexture;
}  // ResourceManager::loadIBLImageIntoTexture

void ResourceManager::loadAllIBLAssets() {
  // map is keyed by config name, value is PbrShaderAttributes, describing the
  // desired configuration of the PBR shader.
  auto mapOfPbrConfigs = metadataMediator_->getAllPbrShaderConfigs();

  // Only load if rendering is enabled.
  if (requiresTextures_) {
    // Load BLUTs and Envmaps specified in scene dataset
    // First verify that pbr image resources are available, and load if not
    if (!Cr::Utility::Resource::hasGroup("pbr-images")) {
      importPbrImageResources();
    }
    const Cr::Utility::Resource rs{"pbr-images"};

    ESP_DEBUG() << "PBR/IBL asset file sets (IBL brdf LUTs and environment "
                   "maps) being loaded :"
                << mapOfPbrConfigs.size();
    for (const auto& entry : mapOfPbrConfigs) {
      // Build required pbrIBL Helpers
      getOrBuildPBRIBLHelper(entry.second);

    }  // for each PbrShaderAttributes defined

  } else {
    if (mapOfPbrConfigs.size() > 1) {
      // There will always be 1 config (default) but if more than 1 exist then
      // perhaps having no renderer was not desired.
      ESP_WARNING() << "Unable to load and convert" << mapOfPbrConfigs.size()
                    << "PBR/IBL asset sets specified in the Scene Dataset due "
                       "to no renderer being instantiated; "
                       "simConfig.requiresTextures_ is false.";
    }
  }
  //
}  // ResourceManager::loadAndBuildAllIBLAssets

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
    if (meshData.primitive != Mn::MeshPrimitive::Triangles) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Unsupported mesh primitive in join: `" << meshData.primitive
          << "` so skipping join.";
    } else {
      for (const auto& pos : meshData.positions) {
        mesh.vbo.push_back(Mn::EigenIntegration::cast<vec3f>(
            transformFromLocalToWorld.transformPoint(pos)));
      }
      for (const auto& index : meshData.indices) {
        mesh.ibo.push_back(index + lastIndex);
      }
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
  Mn::Matrix4 transformFromLocalToWorld =
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
      mesh.vbo.push_back(Mn::EigenIntegration::cast<vec3f>(
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

void ResourceManager::setLightSetup(gfx::LightSetup setup,
                                    const Mn::ResourceKey& key) {
  // add lights to recorder keyframe
  if (gfxReplayRecorder_) {
    gfxReplayRecorder_->clearLightsFromKeyframe();
    for (const auto& light : setup) {
      gfxReplayRecorder_->addLightToKeyframe(light);
    }
  }

  shaderManager_.set(key, std::move(setup), Mn::ResourceDataState::Mutable,
                     Mn::ResourcePolicy::Manual);
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

  Mn::Matrix4 identity;
  joinSemanticHierarchy(*mesh, objectIds, metaData, metaData.root, identity);

  return mesh;
}

}  // namespace assets
}  // namespace esp
