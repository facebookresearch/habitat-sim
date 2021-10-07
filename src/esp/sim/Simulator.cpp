// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <memory>
#include <string>
#include <utility>

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Renderer.h>

#include "esp/core/Esp.h"
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/PbrDrawable.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/VarianceShadowMapDrawable.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletCollisionHelper.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/SensorFactory.h"
#include "esp/sensor/VisualSensor.h"

namespace Cr = Corrade;

namespace esp {
namespace sim {

using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::SceneAOInstanceAttributes;
using metadata::attributes::SceneObjectInstanceAttributes;
using metadata::attributes::StageAttributes;

namespace {
constexpr const char* shadowMapDrawableGroupName = "static-shadow-map";
constexpr const char* defaultRenderingGroupName = "";
const int shadowMapSize = 1024;
const int maxNumShadowMaps = 3;  // the max number of point shadow maps

};  // namespace

Simulator::Simulator(const SimulatorConfiguration& cfg,
                     metadata::MetadataMediator::ptr _metadataMediator)
    : metadataMediator_{std::move(_metadataMediator)},
      random_{core::Random::create(cfg.randomSeed)},
      requiresTextures_{Cr::Containers::NullOpt} {
  // initialize members according to cfg
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

Simulator::~Simulator() {
  ESP_DEBUG() << "Deconstructing Simulator";
  close(true);
}

void Simulator::close(const bool destroy) {
  if (renderer_)
    renderer_->acquireGlContext();
  pathfinder_ = nullptr;
  navMeshVisPrimID_ = esp::ID_UNDEFINED;
  navMeshVisNode_ = nullptr;
  agents_.clear();

  physicsManager_ = nullptr;
  gfxReplayMgr_ = nullptr;
  semanticScene_ = nullptr;

  sceneID_.clear();
  sceneManager_ = nullptr;

  resourceManager_ = nullptr;

  if (debugLineRender_) {
    // Python may keep around other shared_ptrs to this object, but we need
    // to release GL resources here.
    debugLineRender_->releaseGLResources();
    debugLineRender_ = nullptr;
  }

  // Keeping the renderer and the context only matters when the
  // background renderer was initialized.
  if (destroy || !renderer_->wasBackgroundRendererInitialized()) {
    renderer_ = nullptr;
    context_ = nullptr;
  }

  activeSceneID_ = ID_UNDEFINED;
  activeSemanticSceneID_ = ID_UNDEFINED;
  config_ = SimulatorConfiguration{};

  frustumCulling_ = true;
  requiresTextures_ = Cr::Containers::NullOpt;
}

void Simulator::reconfigure(const SimulatorConfiguration& cfg) {
  // set metadata mediator's cfg  upon creation or reconfigure
  if (!metadataMediator_) {
    metadataMediator_ = metadata::MetadataMediator::create(cfg);
  } else {
    metadataMediator_->setSimulatorConfiguration(cfg);
  }

  // assign MM to RM on create or reconfigure
  if (!resourceManager_) {
    assets::ResourceManager::Flags flags{};
    if (cfg.pbrImageBasedLighting) {
      flags |= assets::ResourceManager::Flag::PbrImageBasedLighting;
    }
    resourceManager_ =
        std::make_unique<assets::ResourceManager>(metadataMediator_, flags);
    // needs to be called after ResourceManager exists but before any assets
    // have been loaded
    reconfigureReplayManager(cfg.enableGfxReplaySave);
  } else {
    resourceManager_->setMetadataMediator(metadataMediator_);
  }

  if (!sceneManager_) {
    sceneManager_ = scene::SceneManager::create_unique();
  }

  // if configuration is unchanged, just reset and return
  if (cfg == config_) {
    // This is a check to make sure that pathfinder_ is not null after
    // a reconfigure. We check to see if it's null so that an existing
    // one isn't overwritten.
    if (!pathfinder_) {
      pathfinder_ = nav::PathFinder::create();
    }
    reset();
    return;
  }
  // otherwise set current configuration and initialize
  // TODO can optimize to do partial re-initialization instead of from-scratch
  config_ = cfg;

  if (!config_.createRenderer) {
    config_.requiresTextures = false;
  }

  if (requiresTextures_ == Cr::Containers::NullOpt) {
    requiresTextures_ = config_.requiresTextures;
    resourceManager_->setRequiresTextures(config_.requiresTextures);
  } else if (!(*requiresTextures_) && config_.requiresTextures) {
    throw std::runtime_error(
        "requiresTextures was changed to True from False.  Must call close() "
        "before changing this value.");
  } else if ((*requiresTextures_) && !config_.requiresTextures) {
    ESP_WARNING() << "Not changing requiresTextures as the simulator was "
                     "initialized with True.  Call close() to change this.";
  }

  if (config_.createRenderer) {
    /* When creating a viewer based app, there is no need to create a
    WindowlessContext since a (windowed) context already exists. */
    if (!context_ && !Magnum::GL::Context::hasCurrent()) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    // reinitialize members
    if (!renderer_) {
      gfx::Renderer::Flags flags;
      if (!(*requiresTextures_))
        flags |= gfx::Renderer::Flag::NoTextures;

#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER
      if (context_)
        flags |= gfx::Renderer::Flag::BackgroundRenderer;

      if (context_ && config_.leaveContextWithBackgroundRenderer)
        flags |= gfx::Renderer::Flag::LeaveContextWithBackgroundRenderer;
#endif

      renderer_ = gfx::Renderer::create(context_.get(), flags);
    }

    renderer_->acquireGlContext();
  } else {
    CORRADE_ASSERT(
        !Magnum::GL::Context::hasCurrent(),
        "Simulator::reconfigure() : Unexpected existing context when "
        "createRenderer==false", );
  }

  // (re) create scene instance
  bool success = createSceneInstance(config_.activeSceneName);

  ESP_DEBUG() << "CreateSceneInstance success =="
              << (success ? "true" : "false")
              << "for active scene name :" << config_.activeSceneName
              << (config_.createRenderer ? " with" : " without") << "renderer.";

}  // Simulator::reconfigure

metadata::attributes::SceneAttributes::cptr
Simulator::setSceneInstanceAttributes(const std::string& activeSceneName) {
  namespace FileUtil = Cr::Utility::Directory;

  // Get scene instance attributes corresponding to passed active scene name
  // This will retrieve, or construct, an appropriately configured scene
  // instance attributes, depending on what exists in the Scene Dataset library
  // for the current dataset.

  metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
      metadataMediator_->getSceneAttributesByName(activeSceneName);
  // check if attributes is null - should not happen
  ESP_CHECK(
      curSceneInstanceAttributes,
      Cr::Utility::formatString(
          "Simulator::setSceneInstanceAttributes() : Attempt to load scene "
          "instance :{} failed due to scene instance not being found. Aborting",
          activeSceneName));

  // 1. Load navmesh specified in current scene instance attributes.
  const std::string& navmeshFileLoc = metadataMediator_->getNavmeshPathByHandle(
      curSceneInstanceAttributes->getNavmeshHandle());

  ESP_DEBUG() << "Navmesh file location in scene instance :" << navmeshFileLoc;
  // Get name of navmesh and use to create pathfinder and load navmesh
  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  if (FileUtil::exists(navmeshFileLoc)) {
    ESP_DEBUG() << "Loading navmesh from" << navmeshFileLoc;
    bool pfSuccess = pathfinder_->loadNavMesh(navmeshFileLoc);
    ESP_DEBUG() << (pfSuccess ? "Navmesh Loaded." : "Navmesh load error.");
  } else {
    ESP_WARNING() << "Navmesh file not found, checked at filename : '"
                  << Mn::Debug::nospace << navmeshFileLoc << Mn::Debug::nospace
                  << "'";
  }
  // Calling to seeding needs to be done after the pathfinder creation but
  // before anything else.
  seed(config_.randomSeed);

  // initialize scene graph CAREFUL! previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when instancing a new scene, shall we delete all of the previous scene
  // graphs?

  activeSceneID_ = sceneManager_->initSceneGraph();
  sceneID_.push_back(activeSceneID_);

  // 2. Load the Semantic Scene Descriptor file appropriate for the current
  // scene instance.
  // get name of desired semantic scene descriptor file
  const std::string semanticSceneDescFilename =
      metadataMediator_->getSemanticSceneDescriptorPathByHandle(
          curSceneInstanceAttributes->getSemanticSceneHandle());

  if (semanticSceneDescFilename != "") {
    const std::string& filenameToUse = semanticSceneDescFilename;
    bool success = false;
    // semantic scene descriptor might not exist, so
    semanticScene_ = nullptr;
    semanticScene_ = scene::SemanticScene::create();
    ESP_DEBUG() << "SceneInstance :" << activeSceneName
                << "proposed Semantic Scene Descriptor filename :"
                << filenameToUse;

    bool fileExists = FileUtil::exists(filenameToUse);
    if (fileExists) {
      // Attempt to load semantic scene descriptor specified in scene instance
      // file, agnostic to file type inferred by name, if file exists.
      success = scene::SemanticScene::loadSemanticSceneDescriptor(
          filenameToUse, *semanticScene_);
      if (success) {
        ESP_DEBUG() << "SSD with SceneAttributes-provided name "
                    << filenameToUse << "successfully found and loaded";
      } else {
        // here if provided file exists but does not correspond to appropriate
        // SSD
        ESP_ERROR()
            << "SSD Load Failure! File with SceneAttributes-provided name "
            << filenameToUse << "exists but was unable to be loaded.";
      }
      // if not success then try to construct a name
    } else {
      // attempt to look for specified file failed, attempt to build new file
      // name by searching in path specified of specified file for
      // info_semantic.json file for replica dataset
      const std::string constructedFilename =
          FileUtil::join(FileUtil::path(filenameToUse), "info_semantic.json");
      fileExists = FileUtil::exists(constructedFilename);
      if (fileExists) {
        success = scene::SemanticScene::loadReplicaHouse(constructedFilename,
                                                         *semanticScene_);
        if (success) {
          ESP_DEBUG() << "SSD for Replica using constructed file :"
                      << constructedFilename << "in directory with"
                      << semanticSceneDescFilename << "loaded successfully";
        } else {
          // here if constructed file exists but does not correspond to
          // appropriate SSD or some loading error occurred.
          ESP_ERROR() << "SSD Load Failure! Replica file with constructed name "
                      << filenameToUse << "exists but was unable to be loaded.";
        }
      } else {
        // neither provided non-empty filename nor constructed filename
        // exists. This is probably due to an incorrect naming in the
        // SceneAttributes
        ESP_WARNING()
            << "SSD File Naming Issue! Neither SceneAttributes-provided name :"
            << filenameToUse
            << " nor constructed filename :" << constructedFilename
            << "exist on disk.";
      }
    }  // if given SSD file name specifiedd exists
  }    // if semantic scene descriptor specified in scene instance

  // 3. Specify frustumCulling based on value from config
  frustumCulling_ = config_.frustumCulling;

  // return a const ptr to the cur scene instance attributes
  return curSceneInstanceAttributes;
}  // Simulator::setSceneInstanceAttributes

bool Simulator::createSceneInstance(const std::string& activeSceneName) {
  if (renderer_) {
    renderer_->acquireGlContext();
  }
  // 1. initial setup for scene instancing - sets or creates the
  // current scene instance to correspond to the given name.
  metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
      setSceneInstanceAttributes(activeSceneName);

  // 2. (re)seat & (re)init physics manager using the physics manager
  // attributes specified in current simulator configuration held in
  // metadataMediator.
  // get rootNode
  auto& rootNode = sceneManager_->getSceneGraph(activeSceneID_).getRootNode();

  resourceManager_->initPhysicsManager(
      physicsManager_, &rootNode,
      metadataMediator_->getCurrentPhysicsManagerAttributes());
  // Set PM's reference to this simulator
  physicsManager_->setSimulator(this);

  // 3. Load lighting as specified for scene instance - perform before stage
  // load so lighting key can be set appropriately. get name of light setup
  // for this scene instance
  std::string lightSetupKey;
  if (config_.overrideSceneLightDefaults) {
    lightSetupKey = config_.sceneLightSetupKey;
    ESP_DEBUG() << "Using SimulatorConfiguration-specified Light key : -"
                << lightSetupKey << "-";
  } else {
    lightSetupKey = metadataMediator_->getLightSetupFullHandle(
        curSceneInstanceAttributes->getLightingHandle());
    ESP_DEBUG() << "Using scene instance-specified Light key : -"
                << lightSetupKey << "-";
    if (lightSetupKey != NO_LIGHT_KEY) {
      // lighting attributes corresponding to this key should exist unless it
      // is empty; if empty, the following does nothing.
      esp::gfx::LightSetup lightingSetup =
          metadataMediator_->getLightLayoutAttributesManager()
              ->createLightSetupFromAttributes(lightSetupKey);
      // set lightsetup in resource manager
      resourceManager_->setLightSetup(lightingSetup,
                                      Mn::ResourceKey{lightSetupKey});
    }
  }
  // set config's sceneLightSetupKey to track currently specified light setup
  // key
  config_.sceneLightSetupKey = lightSetupKey;
  metadataMediator_->setSimulatorConfiguration(config_);

  // 4. Load stage specified by Scene Instance Attributes
  bool success = instanceStageForActiveScene(curSceneInstanceAttributes);

  // 5. Load object instances as spceified by Scene Instance Attributes.
  if (success) {
    success = instanceObjectsForActiveScene(curSceneInstanceAttributes);
    if (success) {
      // 6. Load articulated object instances as specified by Scene Instance
      // Attributes.
      success =
          instanceArticulatedObjectsForActiveScene(curSceneInstanceAttributes);
      if (success) {
        // TODO : reset may eventually have all the scene instance instantiation
        // code so that scenes can be reset
        reset();
      }
    }
  }

  return success;
}  // Simulator::createSceneInstance

bool Simulator::instanceStageForActiveScene(
    const metadata::attributes::SceneAttributes::cptr&
        curSceneInstanceAttributes) {
  // Load stage specified by Scene Instance Attributes
  // Get Stage Instance Attributes - contains name of stage and initial
  // transformation of stage in scene.
  // TODO : need to support stageInstanceAttributes transformation upon
  // creation.
  const SceneObjectInstanceAttributes::cptr stageInstanceAttributes =
      curSceneInstanceAttributes->getStageInstance();

  // check if attributes is null - should not happen
  ESP_CHECK(
      stageInstanceAttributes,
      Cr::Utility::formatString(
          "Simulator::instanceStageForActiveScene() : Attempt to load stage "
          "instance specified in current scene instance :{} failed due to "
          "stage instance configuration not being found. Aborting",
          config_.activeSceneName));

  // Get full library name of StageAttributes
  const std::string stageAttributesHandle =
      metadataMediator_->getStageAttrFullHandle(
          stageInstanceAttributes->getHandle());
  // Get StageAttributes copy
  auto stageAttributes =
      metadataMediator_->getStageAttributesManager()->getObjectCopyByHandle(
          stageAttributesHandle);

  // set defaults for stage creation

  // set shader type to use for stage - if no valid value is specified in
  // instance attributes, this field will be whatever was specified in the
  // stage attributes.
  auto stageShaderType = stageInstanceAttributes->getShaderType();
  if (stageShaderType !=
      metadata::attributes::ObjectInstanceShaderType::Unknown) {
    stageAttributes->setShaderType(getShaderTypeName(stageShaderType));
  }
  // set lighting key based on curent config value
  stageAttributes->setLightSetupKey(config_.sceneLightSetupKey);
  // set frustum culling from simulator config
  stageAttributes->setFrustumCulling(frustumCulling_);
  // set scaling values for this instance of stage attributes
  stageAttributes->setScale(stageAttributes->getScale() *
                            stageInstanceAttributes->getUniformScale());
  // set visibility if explicitly specified in stage instance configs
  int visSet = stageInstanceAttributes->getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // specfied in scene instance
    stageAttributes->setIsVisible(visSet == 1);
  }
  // create a structure to manage active scene and active semantic scene ID
  // passing to and from loadStage
  std::vector<int> tempIDs{activeSceneID_, activeSemanticSceneID_};
  ESP_DEBUG() << "Start to load stage named :" << stageAttributes->getHandle()
              << "with render asset :"
              << stageAttributes->getRenderAssetHandle()
              << "and collision asset :"
              << stageAttributes->getCollisionAssetHandle();

  // Load stage
  bool loadSuccess = resourceManager_->loadStage(
      stageAttributes, stageInstanceAttributes, physicsManager_,
      sceneManager_.get(), tempIDs);

  if (!loadSuccess) {
    ESP_ERROR() << "Cannot load stage :" << stageAttributesHandle;
    // Pass the error to the python through pybind11 allowing graceful exit
    throw std::invalid_argument("Cannot load: " + stageAttributesHandle);
    return false;
  } else {
    ESP_DEBUG() << "Successfully loaded stage named :"
                << stageAttributes->getHandle();
  }

  // refresh the NavMesh visualization if necessary after loading a new
  // SceneGraph
  resetNavMeshVisIfActive();

  // set activeSemanticSceneID_ values and push onto sceneID vector if
  // appropriate - tempIDs[1] will either be old activeSemanticSceneID_ (if
  // no semantic mesh was requested in loadStage); ID_UNDEFINED if desired
  // was not found; activeSceneID_, or a unique value, the last of which means
  // the semantic scene mesh is loaded.

  if (activeSemanticSceneID_ != tempIDs[1]) {
    // id has changed so act - if ID has not changed, do nothing
    activeSemanticSceneID_ = tempIDs[1];
    if ((activeSemanticSceneID_ != ID_UNDEFINED) &&
        (activeSemanticSceneID_ != activeSceneID_)) {
      sceneID_.push_back(activeSemanticSceneID_);
    } else {  // activeSemanticSceneID_ == activeSceneID_;
      assets::AssetType stageType =
          static_cast<assets::AssetType>(stageAttributes->getRenderAssetType());
      // instance meshes and suncg houses contain their semantic annotations
      // empty scene has none to worry about
      if (!(stageType == assets::AssetType::SUNCG_SCENE ||
            stageType == assets::AssetType::INSTANCE_MESH ||
            stageAttributesHandle == assets::EMPTY_SCENE)) {
        // TODO: programmatic generation of semantic meshes when no
        // annotations are provided.
        ESP_WARNING() << "\n---\nThe active scene does not contain semantic "
                         "annotations. \n---";
      }
    }
  }  // if ID has changed - needs to be reset
  return true;
}  // Simulator::instanceStageForActiveScene()

bool Simulator::instanceObjectsForActiveScene(
    const metadata::attributes::SceneAttributes::cptr&
        curSceneInstanceAttributes) {
  // 5. Load object instances as spceified by Scene Instance Attributes.
  // Get all instances of objects described in scene
  const std::vector<SceneObjectInstanceAttributes::cptr> objectInstances =
      curSceneInstanceAttributes->getObjectInstances();

  // node to attach object to
  scene::SceneNode* attachmentNode = nullptr;
  // int objID = 0;

  // whether or not to correct for COM shift - only do for blender-sourced
  // scene attributes
  bool defaultCOMCorrection =
      (curSceneInstanceAttributes->getTranslationOrigin() ==
       metadata::attributes::SceneInstanceTranslationOrigin::AssetLocal);

  // Iterate through instances, create object and implement initial
  // transformation.
  for (const auto& objInst : objectInstances) {
    // check if attributes is null - should not happen
    ESP_CHECK(
        objInst,
        Cr::Utility::formatString(
            "Simulator::instanceObjectsForActiveScene() : Attempt to load "
            "object instance specified in current scene instance :{} failed "
            "due to object instance configuration not being found. Aborting",
            config_.activeSceneName));

    const std::string objAttrFullHandle =
        metadataMediator_->getObjAttrFullHandle(objInst->getHandle());
    // make sure full handle is not empty
    ESP_CHECK(
        !objAttrFullHandle.empty(),
        Cr::Utility::formatString(
            "Simulator::instanceObjectsForActiveScene() : Attempt to load "
            "object instance specified in current scene instance :{} failed "
            "due to object instance configuration handle '{}' being empty or "
            "unknown. Aborting",
            config_.activeSceneName, objInst->getHandle()));
    // objID =
    physicsManager_->addObjectInstance(objInst, objAttrFullHandle,
                                       defaultCOMCorrection, attachmentNode,
                                       config_.sceneLightSetupKey);
  }  // for each object attributes
  return true;
}  // Simulator::instanceObjectsForActiveScene()

bool Simulator::instanceArticulatedObjectsForActiveScene(
    const metadata::attributes::SceneAttributes::cptr&
        curSceneInstanceAttributes) {
  // 6. Load all articulated object instances
  // Get all instances of articulated objects described in scene
  const std::vector<SceneAOInstanceAttributes::cptr> artObjInstances =
      curSceneInstanceAttributes->getArticulatedObjectInstances();

  // int aoID = 0;
  auto& drawables = getDrawableGroup();
  // Iterate through instances, create object and implement initial
  // transformation.
  for (const auto& artObjInst : artObjInstances) {
    // check if instance attributes is null - should not happen
    ESP_CHECK(artObjInst,
              Cr::Utility::formatString(
                  "Simulator::instanceArticulatedObjectsForActiveScene() "
                  ": Attempt to load articulated object instance "
                  "specified in current scene instance :{} failed due to "
                  "AO instance configuration not being found. Aborting",
                  config_.activeSceneName));

    // get model file name
    const std::string artObjFilePath =
        metadataMediator_->getArticulatedObjModelFullHandle(
            artObjInst->getHandle());

    // make sure full handle is not empty
    ESP_CHECK(
        !artObjFilePath.empty(),
        Cr::Utility::formatString(
            "Simulator::instanceArticulatedObjectsForActiveScene() : Attempt "
            "to load articualted object instance specified in current scene "
            "instance :{} failed due to AO instance configuration file handle "
            "'{}' being empty or unknown. Aborting",
            config_.activeSceneName, artObjInst->getHandle()));

    // create articulated object
    // aoID =
    physicsManager_->addArticulatedObjectInstance(artObjFilePath, artObjInst,
                                                  config_.sceneLightSetupKey);
  }  // for each articulated object instance
  return true;
}  // Simulator::instanceArticulatedObjectsForActiveScene
void Simulator::reset() {
  if (physicsManager_ != nullptr) {
    // Note: only resets time to 0 by default.
    physicsManager_->reset();
  }

  for (auto& agent : agents_) {
    agent->reset();
  }
  const Magnum::Range3D& sceneBB =
      getActiveSceneGraph().getRootNode().computeCumulativeBB();
  resourceManager_->setLightSetup(gfx::getDefaultLights());
}  // Simulator::reset()

void Simulator::seed(uint32_t newSeed) {
  random_->seed(newSeed);
  pathfinder_->seed(newSeed);
}

void Simulator::reconfigureReplayManager(bool enableGfxReplaySave) {
  gfxReplayMgr_ = std::make_shared<gfx::replay::ReplayManager>();

  // construct Recorder instance if requested
  gfxReplayMgr_->setRecorder(enableGfxReplaySave
                                 ? std::make_shared<gfx::replay::Recorder>()
                                 : nullptr);
  // assign Recorder to ResourceManager
  CORRADE_INTERNAL_ASSERT(resourceManager_);
  resourceManager_->setRecorder(gfxReplayMgr_->getRecorder());

  // provide Player callback to replay manager
  gfxReplayMgr_->setPlayerCallback(
      [this](const assets::AssetInfo& assetInfo,
             const assets::RenderAssetInstanceCreationInfo& creation)
          -> scene::SceneNode* {
        return loadAndCreateRenderAssetInstance(assetInfo, creation);
      });
}

void Simulator::updateShadowMapDrawableGroup() {
  scene::SceneGraph& sg = getActiveSceneGraph();
  // currently the method is naive: destroy the existing group, and recreate one
  sg.deleteDrawableGroup(shadowMapDrawableGroupName);
  sg.createDrawableGroup(shadowMapDrawableGroupName);
  const gfx::DrawableGroup& sourceGroup = sg.getDrawables();
  gfx::DrawableGroup* shadowMapGroup =
      sg.getDrawableGroup(shadowMapDrawableGroupName);
  CORRADE_INTERNAL_ASSERT(shadowMapGroup);

  for (size_t iDrawable = 0; iDrawable < sourceGroup.size(); ++iDrawable) {
    const gfx::Drawable& currentDrawable =
        static_cast<const gfx::Drawable&>(sourceGroup[iDrawable]);
    gfx::DrawableType type = currentDrawable.getDrawableType();
    // So far no support for the PTex (lighting-baked mesh)
    // SKIP!!!
    if ((type != gfx::DrawableType::Generic) &&
        (type != gfx::DrawableType::Pbr)) {
      continue;
    }

    esp::scene::SceneNode& node = currentDrawable.getSceneNode();
    node.addFeature<gfx::VarianceShadowMapDrawable>(
        &currentDrawable.getMesh(), resourceManager_->getShaderManager(),
        shadowMapGroup);
  }
}

void Simulator::computeShadowMaps(float lightNearPlane, float lightFarPlane) {
  scene::SceneGraph& sg = getActiveSceneGraph();
  auto& shadowManager = resourceManager_->getShadowMapManger();
  auto& shadowMapKeys = resourceManager_->getShadowMapKeys();
  std::vector<Mn::ResourceKey>& keys = shadowMapKeys[activeSceneID_];

  // TODO:
  // current implementation is for static lights for static scenes
  // We will have to refactor the code so that in the future dynamic lights and
  // dynamic scenes can be handled.

  // construct the lights from scene dataset config
  esp::gfx::LightSetup lightingSetup =
      metadataMediator_->getLightLayoutAttributesManager()
          ->createLightSetupFromAttributes(config_.sceneLightSetupKey);

  scene::SceneNode& lightNode = sg.getRootNode().createChild();
  gfx::CubeMapCamera camera{lightNode};

  int actualShadowMaps = maxNumShadowMaps <= lightingSetup.size()
                             ? maxNumShadowMaps
                             : lightingSetup.size();
  for (int iLight = 0; iLight < actualShadowMaps; ++iLight) {
    // sanity checks
    CORRADE_ASSERT(
        lightingSetup[iLight].model == esp::gfx::LightPositionModel::Global,
        "Simulator::computeShadowMaps: To compute the shadow map, the light"
            << iLight << "should be in `global` mode.", );
    CORRADE_ASSERT(lightingSetup[iLight].vector.w() == 1,
                   "Simulator::computeShadowMaps: Only point light shadow is "
                   "supported. However, the light"
                       << iLight << "is a directional light.", );
    Mn::Vector3 lightPos = Mn::Vector3{lightingSetup[iLight].vector.xyz()};

    Mn::ResourceKey key(Corrade::Utility::formatString(
        assets::ResourceManager::SHADOW_MAP_KEY_TEMPLATE, activeSceneID_,
        iLight));

    // insert the key to the data base
    keys.push_back(key);

    // create the resource if there is not one
    Mn::Resource<gfx::CubeMap> pointShadowMap =
        shadowManager.get<gfx::CubeMap>(key);
    if (!pointShadowMap) {
      shadowManager.set<gfx::CubeMap>(
          pointShadowMap.key(),
          new gfx::CubeMap{
              shadowMapSize,
              {gfx::CubeMap::Flag::VarianceShadowMapTexture |
               // gfx::CubeMap::Flag::ColorTexture | // for future visualization
               gfx::CubeMap::Flag::AutoBuildMipmap}},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::Resident);

      CORRADE_INTERNAL_ASSERT(pointShadowMap && pointShadowMap.key() == key);
    }
    if (pointShadowMap->getCubeMapSize() != shadowMapSize) {
      pointShadowMap->reset(shadowMapSize);
    }

    // setup a CubeMapCamera in the root of scene graph, and set its position to
    // light global position
    lightNode.setTranslation(lightPos);

    camera.setProjectionMatrix(shadowMapSize,   // width of the square
                               lightNearPlane,  // near plane
                               lightFarPlane);  // far plane
    pointShadowMap->renderToTexture(camera, sg, shadowMapDrawableGroupName,
                                    {gfx::RenderCamera::Flag::FrustumCulling |
                                     gfx::RenderCamera::Flag::ClearDepth});

    Mn::ResourceKey helperKey("helper-shadow-cubemap");
    Mn::Resource<gfx::CubeMap> helperShadowMap =
        shadowManager.get<gfx::CubeMap>(helperKey);
    if (!helperShadowMap) {
      shadowManager.set<gfx::CubeMap>(
          helperShadowMap.key(),
          new gfx::CubeMap{shadowMapSize,
                           {gfx::CubeMap::Flag::VarianceShadowMapTexture |
                            gfx::CubeMap::Flag::AutoBuildMipmap}},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);

      CORRADE_INTERNAL_ASSERT(helperShadowMap &&
                              helperShadowMap.key() == helperKey);
    }

    // for VSM only !!!
    renderer_->applyGaussianFiltering(
        *pointShadowMap, *helperShadowMap,
        gfx::CubeMap::TextureType::VarianceShadowMap);
  }
}

void Simulator::setShadowMapsToDrawables() {
  scene::SceneGraph& sg = getActiveSceneGraph();
  gfx::DrawableGroup& defaultRenderingGroup = sg.getDrawables();
  auto& shadowManager = resourceManager_->getShadowMapManger();
  auto& shadowMapKeys = resourceManager_->getShadowMapKeys();

  for (size_t iDrawable = 0; iDrawable < defaultRenderingGroup.size();
       ++iDrawable) {
    const gfx::Drawable& currentDrawable =
        static_cast<const gfx::Drawable&>(defaultRenderingGroup[iDrawable]);
    gfx::DrawableType type = currentDrawable.getDrawableType();
    // So far only pbr drawables support shadow
    // SKIP!!!
    if (type != gfx::DrawableType::Pbr) {
      continue;
    }
    auto& pbrDrawable = const_cast<gfx::PbrDrawable&>(
        static_cast<const gfx::PbrDrawable&>(currentDrawable));
    CORRADE_ASSERT(shadowMapKeys[activeSceneID_].size(),
                   "Simulator::setShadowMapsToDrawables(): there are no shadow "
                   "maps for the current active scene graph.", );
    pbrDrawable.setShadowData(shadowManager, shadowMapKeys[activeSceneID_],
                              esp::gfx::PbrShader::Flag::ShadowsVSM);
  }
}

// === Physics Simulator Functions ===

int Simulator::addObject(const int objectLibId,
                         scene::SceneNode* attachmentNode,
                         const std::string& lightSetupKey,
                         const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    if (renderer_)
      renderer_->acquireGlContext();
    // TODO: change implementation to support multi-world and physics worlds
    // to own reference to a sceneGraph to avoid this.
    auto& drawables = getDrawableGroup(sceneID);
    return physicsManager_->addObject(objectLibId, &drawables, attachmentNode,
                                      lightSetupKey);
  }
  return ID_UNDEFINED;
}

int Simulator::addObjectByHandle(const std::string& objectLibHandle,
                                 scene::SceneNode* attachmentNode,
                                 const std::string& lightSetupKey,
                                 const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    if (renderer_)
      renderer_->acquireGlContext();
    // TODO: change implementation to support multi-world and physics worlds
    // to own reference to a sceneGraph to avoid this.
    auto& drawables = getDrawableGroup(sceneID);
    return physicsManager_->addObject(objectLibHandle, &drawables,
                                      attachmentNode, lightSetupKey);
  }
  return ID_UNDEFINED;
}

// remove object objectID instance in sceneID
void Simulator::removeObject(const int objectID,
                             bool deleteObjectNode,
                             bool deleteVisualNode,
                             const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->removeObject(objectID, deleteObjectNode, deleteVisualNode);
    if (trajVisNameByID.count(objectID) > 0) {
      std::string trajVisAssetName = trajVisNameByID[objectID];
      trajVisNameByID.erase(objectID);
      trajVisIDByName.erase(trajVisAssetName);
      // TODO : if object is trajectory visualization, remove its assets as
      // well once this is supported.
      // resourceManager_->removeResourceByName(trajVisAssetName);
    }
  }
}

double Simulator::stepWorld(const double dt) {
  if (physicsManager_ != nullptr) {
    physicsManager_->deferNodesUpdate();
    physicsManager_->stepPhysics(dt);
    if (renderer_)
      renderer_->waitSceneGraph();

    physicsManager_->updateNodes();
  }
  return getWorldTime();
}

// get the simulated world time (0 if no physics enabled)
double Simulator::getWorldTime() {
  if (physicsManager_ != nullptr) {
    return physicsManager_->getWorldTime();
  }
  return NO_TIME;
}

bool Simulator::recomputeNavMesh(nav::PathFinder& pathfinder,
                                 const nav::NavMeshSettings& navMeshSettings,
                                 bool includeStaticObjects) {
  CORRADE_ASSERT(config_.createRenderer,
                 "::recomputeNavMesh: "
                 "SimulatorConfiguration::createRenderer is "
                 "false. Scene geometry is required to recompute navmesh. No "
                 "geometry is "
                 "loaded without renderer initialization.",
                 false);

  assets::MeshData::uptr joinedMesh = assets::MeshData::create_unique();
  auto stageInitAttrs = physicsManager_->getStageInitAttributes();
  if (stageInitAttrs != nullptr) {
    joinedMesh = resourceManager_->createJoinedCollisionMesh(
        stageInitAttrs->getRenderAssetHandle());
  }

  // add STATIC collision objects
  if (includeStaticObjects) {
    // update nodes so SceneNode transforms are up-to-date
    if (renderer_)
      renderer_->waitSceneGraph();

    physicsManager_->updateNodes();

    // collect mesh components from all objects and then merge them.
    // Each mesh component could be duplicated multiple times w/ different
    // transforms.
    std::map<std::string,
             std::vector<Eigen::Transform<float, 3, Eigen::Affine>>>
        meshComponentStates;

    // collect RigidObject mesh components
    for (auto objectID : physicsManager_->getExistingObjectIDs()) {
      auto objWrapper = queryRigidObjWrapper(activeSceneID_, objectID);
      if (objWrapper->getMotionType() == physics::MotionType::STATIC) {
        auto objectTransform = Magnum::EigenIntegration::cast<
            Eigen::Transform<float, 3, Eigen::Affine>>(
            physicsManager_->getObjectVisualSceneNode(objectID)
                .absoluteTransformationMatrix());
        const metadata::attributes::ObjectAttributes::cptr
            initializationTemplate = objWrapper->getInitializationAttributes();
        objectTransform.scale(Magnum::EigenIntegration::cast<vec3f>(
            initializationTemplate->getScale()));
        std::string meshHandle =
            initializationTemplate->getCollisionAssetHandle();
        if (meshHandle.empty()) {
          meshHandle = initializationTemplate->getRenderAssetHandle();
        }
        meshComponentStates[meshHandle].push_back(objectTransform);
      }
    }

    // collect ArticulatedObject mesh components
    for (auto& objectID : physicsManager_->getExistingArticulatedObjectIds()) {
      auto articulatedObject =
          getArticulatedObjectManager()->getObjectByID(objectID);
      if (articulatedObject->getMotionType() == physics::MotionType::STATIC) {
        for (int linkIx = -1; linkIx < articulatedObject->getNumLinks();
             ++linkIx) {
          //-1 is baseLink_
          std::vector<std::pair<esp::scene::SceneNode*, std::string>>
              visualAttachments =
                  physicsManager_->getArticulatedObject(objectID)
                      .getLink(linkIx)
                      .visualAttachments_;
          for (auto& visualAttachment : visualAttachments) {
            auto objectTransform = Magnum::EigenIntegration::cast<
                Eigen::Transform<float, 3, Eigen::Affine>>(
                visualAttachment.first->absoluteTransformationMatrix());
            std::string meshHandle = visualAttachment.second;
            meshComponentStates[meshHandle].push_back(objectTransform);
          }
        }
      }
    }

    // merge mesh components into the final mesh
    for (auto& meshComponent : meshComponentStates) {
      assets::MeshData::uptr joinedObjectMesh =
          resourceManager_->createJoinedCollisionMesh(meshComponent.first);
      for (auto& meshTransform : meshComponent.second) {
        int prevNumIndices = joinedMesh->ibo.size();
        int prevNumVerts = joinedMesh->vbo.size();
        joinedMesh->ibo.resize(prevNumIndices + joinedObjectMesh->ibo.size());
        for (size_t ix = 0; ix < joinedObjectMesh->ibo.size(); ++ix) {
          joinedMesh->ibo[ix + prevNumIndices] =
              joinedObjectMesh->ibo[ix] + prevNumVerts;
        }
        joinedMesh->vbo.reserve(joinedObjectMesh->vbo.size() + prevNumVerts);
        for (auto& vert : joinedObjectMesh->vbo) {
          joinedMesh->vbo.push_back(meshTransform * vert);
        }
      }
    }
  }

  if (!pathfinder.build(navMeshSettings, *joinedMesh)) {
    ESP_ERROR() << "Failed to build navmesh";
    return false;
  }

  if (&pathfinder == pathfinder_.get()) {
    resetNavMeshVisIfActive();
  }

  ESP_DEBUG() << "reconstruct navmesh successful";
  return true;
}

bool Simulator::setNavMeshVisualization(bool visualize) {
  if (renderer_)
    renderer_->acquireGlContext();
  // clean-up the NavMesh visualization if necessary
  if (!visualize && navMeshVisNode_ != nullptr) {
    delete navMeshVisNode_;
    navMeshVisNode_ = nullptr;
    if (navMeshVisPrimID_ != ID_UNDEFINED)
      resourceManager_->removePrimitiveMesh(navMeshVisPrimID_);
    navMeshVisPrimID_ = ID_UNDEFINED;
  }

  // Create new visualization asset and SceneNode
  if (visualize && pathfinder_ != nullptr && navMeshVisNode_ == nullptr &&
      pathfinder_->isLoaded()) {
    auto& sceneGraph = sceneManager_->getSceneGraph(activeSceneID_);
    auto& rootNode = sceneGraph.getRootNode();
    auto& drawables = sceneGraph.getDrawables();
    navMeshVisNode_ = &rootNode.createChild();
    navMeshVisPrimID_ = resourceManager_->loadNavMeshVisualization(
        *pathfinder_, navMeshVisNode_, &drawables);
    if (navMeshVisPrimID_ == ID_UNDEFINED) {
      ESP_ERROR() << "Failed to load navmesh visualization.";
      delete navMeshVisNode_;
    }
  }
  return isNavMeshVisualizationActive();
}

bool Simulator::isNavMeshVisualizationActive() {
  return (navMeshVisNode_ != nullptr && navMeshVisPrimID_ != ID_UNDEFINED);
}

int Simulator::addTrajectoryObject(const std::string& trajVisName,
                                   const std::vector<Mn::Vector3>& pts,
                                   int numSegments,
                                   float radius,
                                   const Magnum::Color4& color,
                                   bool smooth,
                                   int numInterp) {
  if (renderer_)
    renderer_->acquireGlContext();

  // 0. Deduplicate sequential points
  std::vector<Magnum::Vector3> uniquePts;
  uniquePts.push_back(pts[0]);
  for (const auto& loc : pts) {
    if (loc != uniquePts.back()) {
      uniquePts.push_back(loc);
    }
  }

  auto& drawables = getDrawableGroup();

  // 1. create trajectory tube asset from points and save it
  bool success = resourceManager_->buildTrajectoryVisualization(
      trajVisName, uniquePts, numSegments, radius, color, smooth, numInterp);
  if (!success) {
    ESP_ERROR() << "Failed to create Trajectory visualization mesh for"
                << trajVisName;
    return ID_UNDEFINED;
  }
  // 2. create object attributes for the trajectory
  auto objAttrMgr = metadataMediator_->getObjectAttributesManager();
  auto trajObjAttr = objAttrMgr->createObject(trajVisName, false);
  // turn off collisions
  trajObjAttr->setIsCollidable(false);
  trajObjAttr->setComputeCOMFromShape(false);
  objAttrMgr->registerObject(trajObjAttr, trajVisName, true);

  // 3. add trajectory object to manager
  auto trajVisID = physicsManager_->addObject(trajVisName, &drawables);
  if (trajVisID == ID_UNDEFINED) {
    // failed to add object - need to delete asset from resourceManager.
    ESP_ERROR() << "Failed to create Trajectory visualization object for"
                << trajVisName;
    // TODO : support removing asset by removing from resourceDict_ properly
    // using trajVisName
    return ID_UNDEFINED;
  }
  auto trajObj = getRigidObjectManager()->getObjectCopyByID(trajVisID);
  ESP_DEBUG() << "Trajectory visualization object created with ID" << trajVisID;
  trajObj->setMotionType(esp::physics::MotionType::KINEMATIC);
  // add to internal references of object ID and resourceDict name
  // this is for eventual asset deletion/resource freeing.
  trajVisIDByName[trajVisName] = trajVisID;
  trajVisNameByID[trajVisID] = trajVisName;

  return trajVisID;
}  // Simulator::showTrajectoryVisualization

// Agents
void Simulator::sampleRandomAgentState(agent::AgentState& agentState) {
  if (pathfinder_->isLoaded()) {
    agentState.position = pathfinder_->getRandomNavigablePoint();
    const float randomAngleRad = random_->uniform_float_01() * M_PI;
    quatf rotation(Eigen::AngleAxisf(randomAngleRad, vec3f::UnitY()));
    agentState.rotation = rotation.coeffs();
    // TODO: any other AgentState members should be randomized?
  } else {
    ESP_ERROR() << "No loaded PathFinder, aborting sampleRandomAgentState.";
  }
}

scene::SceneNode* Simulator::loadAndCreateRenderAssetInstance(
    const assets::AssetInfo& assetInfo,
    const assets::RenderAssetInstanceCreationInfo& creation) {
  if (renderer_)
    renderer_->acquireGlContext();
  // Note this pattern of passing the scene manager and two scene ids to
  // resource manager. This is similar to ResourceManager::loadStage.
  std::vector<int> tempIDs{activeSceneID_, activeSemanticSceneID_};
  return resourceManager_->loadAndCreateRenderAssetInstance(
      assetInfo, creation, sceneManager_.get(), tempIDs);
}

#ifdef ESP_BUILD_WITH_VHACD
std::string Simulator::convexHullDecomposition(
    const std::string& filename,
    const assets::ResourceManager::VHACDParameters& params,
    const bool renderChd,
    const bool saveChdToObj) {
  ESP_DEBUG() << "VHACD PARAMS RESOLUTION:" << params.m_resolution;

  // generate a unique filename
  std::string chdFilename =
      Cr::Utility::Directory::splitExtension(filename).first + ".chd";
  if (resourceManager_->isAssetDataRegistered(chdFilename)) {
    int nameAttempt = 1;
    chdFilename += "_";
    // Iterate until a unique filename is found.
    while (resourceManager_->isAssetDataRegistered(
        chdFilename + std::to_string(nameAttempt))) {
      nameAttempt++;
    }
    chdFilename += std::to_string(nameAttempt);
  }

  // run VHACD on the given filename mesh with the given params, store the
  // results in the resourceDict_ registered under chdFilename
  resourceManager_->createConvexHullDecomposition(filename, chdFilename, params,
                                                  saveChdToObj);

  // create object attributes for the new chd object
  auto objAttrMgr = metadataMediator_->getObjectAttributesManager();
  auto chdObjAttr = objAttrMgr->createObject(chdFilename, false);

  // specify collision asset handle & other attributes
  chdObjAttr->setCollisionAssetHandle(chdFilename);
  chdObjAttr->setIsCollidable(true);
  chdObjAttr->setCollisionAssetIsPrimitive(false);
  chdObjAttr->setJoinCollisionMeshes(false);

  // if the renderChd flag is set to true, set the convex hull decomposition
  // to be the render asset (useful for testing)

  chdObjAttr->setRenderAssetHandle(renderChd ? chdFilename : filename);

  chdObjAttr->setRenderAssetIsPrimitive(false);

  // register object and return handle
  objAttrMgr->registerObject(chdObjAttr, chdFilename, true);
  return chdObjAttr->getHandle();
}
#endif

agent::Agent::ptr Simulator::addAgent(
    const agent::AgentConfiguration& agentConfig,
    scene::SceneNode& agentParentNode) {
  // initialize the agent, as well as all the sensors on it.

  // attach each agent, each sensor to a scene node, set the local
  // transformation of the sensor w.r.t. the agent (done internally in the
  // constructor of Agent)
  auto& agentNode = agentParentNode.createChild();
  agent::Agent::ptr ag = agent::Agent::create(agentNode, agentConfig);
  esp::sensor::SensorFactory::createSensors(agentNode,
                                            agentConfig.sensorSpecifications);
  agent::AgentState state;
  sampleRandomAgentState(state);
  ag->setInitialState(state);

  // Add a RenderTarget to each of the agent's visual sensors
  for (auto& it : ag->getSubtreeSensors()) {
    if (it.second.get().isVisualSensor()) {
      sensor::VisualSensor& sensor =
          static_cast<sensor::VisualSensor&>(it.second.get());
      renderer_->bindRenderTarget(sensor);
    }
  }

  agents_.push_back(ag);
  // TODO: just do this once
  if (pathfinder_->isLoaded()) {
    scene::ObjectControls::MoveFilterFunc moveFilterFunction;
    if (config_.allowSliding) {
      moveFilterFunction = [&](const vec3f& start, const vec3f& end) {
        return pathfinder_->tryStep(start, end);
      };
    } else {
      moveFilterFunction = [&](const vec3f& start, const vec3f& end) {
        return pathfinder_->tryStepNoSliding(start, end);
      };
    }
    ag->getControls()->setMoveFilterFunction(moveFilterFunction);
  }

  return ag;
}

agent::Agent::ptr Simulator::addAgent(
    const agent::AgentConfiguration& agentConfig) {
  return addAgent(agentConfig, getActiveSceneGraph().getRootNode());
}

agent::Agent::ptr Simulator::getAgent(const int agentId) {
  CORRADE_INTERNAL_ASSERT(0 <= agentId && agentId < agents_.size());
  return agents_[agentId];
}

esp::sensor::Sensor& Simulator::addSensorToObject(
    const int objectId,
    const esp::sensor::SensorSpec::ptr& sensorSpec) {
  if (renderer_)
    renderer_->acquireGlContext();
  esp::sensor::SensorSetup sensorSpecifications = {sensorSpec};
  esp::scene::SceneNode& objectNode = *getObjectSceneNode(objectId);
  esp::sensor::SensorFactory::createSensors(objectNode, sensorSpecifications);
  return objectNode.getNodeSensorSuite().get(sensorSpec->uuid);
}

void Simulator::setPathFinder(nav::PathFinder::ptr pathfinder) {
  pathfinder_ = std::move(pathfinder);
}
gfx::RenderTarget* Simulator::getRenderTarget(int agentId,
                                              const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor& sensor = ag->getSubtreeSensorSuite().get(sensorId);
    if (sensor.isVisualSensor()) {
      return &(static_cast<sensor::VisualSensor&>(sensor).renderTarget());
    }
  }
  return nullptr;
}

bool Simulator::displayObservation(const int agentId,
                                   const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor& sensor = ag->getSubtreeSensorSuite().get(sensorId);
    return sensor.displayObservation(*this);
  }
  return false;
}

bool Simulator::drawObservation(const int agentId,
                                const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor& sensor = ag->getSubtreeSensorSuite().get(sensorId);
    if (sensor.isVisualSensor()) {
      return static_cast<sensor::VisualSensor&>(sensor).drawObservation(*this);
    }
  }
  return false;
}

bool Simulator::visualizeObservation(int agentId,
                                     const std::string& sensorId,
                                     float colorMapOffset,
                                     float colorMapScale) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor& sensor = ag->getSubtreeSensorSuite().get(sensorId);
    if (sensor.isVisualSensor()) {
      renderer_->visualize(static_cast<sensor::VisualSensor&>(sensor),
                           colorMapOffset, colorMapScale);
    }
    return true;
  }
  return false;
}

bool Simulator::getAgentObservation(const int agentId,
                                    const std::string& sensorId,
                                    sensor::Observation& observation) {
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    return ag->getSubtreeSensorSuite().get(sensorId).getObservation(
        *this, observation);
  }
  return false;
}

int Simulator::getAgentObservations(
    const int agentId,
    std::map<std::string, sensor::Observation>& observations) {
  observations.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    for (auto& s : ag->getSubtreeSensors()) {
      sensor::Observation obs;
      if (s.second.get().getObservation(*this, obs)) {
        observations[s.first] = obs;
      }
    }
  }
  return observations.size();
}

bool Simulator::getAgentObservationSpace(const int agentId,
                                         const std::string& sensorId,
                                         sensor::ObservationSpace& space) {
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    return ag->getSubtreeSensorSuite().get(sensorId).getObservationSpace(space);
  }
  return false;
}

int Simulator::getAgentObservationSpaces(
    const int agentId,
    std::map<std::string, sensor::ObservationSpace>& spaces) {
  spaces.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    for (auto& s : ag->getSubtreeSensors()) {
      sensor::ObservationSpace space;
      if (s.second.get().getObservationSpace(space)) {
        spaces[s.first] = space;
      }
    }
  }
  return spaces.size();
}
}  // namespace sim
}  // namespace esp
