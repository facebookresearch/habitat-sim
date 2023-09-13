// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <memory>
#include <string>
#include <utility>

#include <Corrade/Containers/Pair.h>
#include <Corrade/Utility/Path.h>
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
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletCollisionHelper.h"
#include "esp/physics/objectManagers/ArticulatedObjectManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/scene/ObjectControls.h"
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
  getRenderGLContext();

  pathfinder_ = nullptr;
  navMeshVisPrimID_ = esp::ID_UNDEFINED;
  navMeshVisNode_ = nullptr;
  agents_.clear();

  physicsManager_ = nullptr;
  curSceneInstanceAttributes_ = nullptr;
  gfxReplayMgr_ = nullptr;

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
  semanticSceneMeshLoaded_ = false;
  config_ = SimulatorConfiguration{};

  frustumCulling_ = true;
  requiresTextures_ = Cr::Containers::NullOpt;
}

void Simulator::reconfigure(const SimulatorConfiguration& cfg) {
// Fail early if physics is enabled in config but no bullet support is
// installed.
#ifndef ESP_BUILD_WITH_BULLET
  ESP_CHECK(!cfg.enablePhysics,
            "Physics has been enabled in the SimulatorConfiguration but "
            "Habitat-Sim was not built with Bullet support enabled. Either set "
            "cfg.enable_physics to False, or verify your Bullet installation "
            "(e.g recompile Habitat-Sim using the '--bullet' flag or choose a "
            "'withbullet' conda build.)");
#endif

  // set metadata mediator's cfg  upon creation or reconfigure
  if (!metadataMediator_) {
    metadataMediator_ = metadata::MetadataMediator::create(cfg);
  } else {
    metadataMediator_->setSimulatorConfiguration(cfg);
  }

  // assign MM to RM on create or reconfigure
  if (!resourceManager_) {
    resourceManager_ =
        std::make_unique<assets::ResourceManager>(metadataMediator_);
    // needs to be called after ResourceManager exists but before any assets
    // have been loaded
    reconfigureReplayManager(cfg.enableGfxReplaySave);
  } else {
    resourceManager_->setMetadataMediator(metadataMediator_);
  }

  // reset this count and number here because we are resetting (clearing) the
  // scene
  resourceManager_->resetDrawableCountAndNumFaces();

  if (!sceneManager_) {
    sceneManager_ = scene::SceneManager::create_unique();
  }

  // This is a check to make sure that pathfinder_ is not null after
  // a reconfigure. We check to see if it's null so that an existing
  // one isn't overwritten.
  if (!pathfinder_) {
    pathfinder_ = nav::PathFinder::create();
  }

  // if configuration is unchanged, just reset and return
  if (cfg == config_) {
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
      if (context_) {
        flags |= gfx::Renderer::Flag::BackgroundRenderer;
      }

      if (context_ && config_.leaveContextWithBackgroundRenderer) {
        flags |= gfx::Renderer::Flag::LeaveContextWithBackgroundRenderer;
      }
#endif

      if (config_.enableHBAO) {
        flags |= gfx::Renderer::Flag::HorizonBasedAmbientOcclusion;
      }

      renderer_ = gfx::Renderer::create(context_.get(), flags);
    }
#ifndef CORRADE_TARGET_EMSCRIPTEN
    flextGLInit(Magnum::GL::Context::current());
#endif
    renderer_->acquireGlContext();
  }

  // (re) create scene instance
  bool success = createSceneInstance(config_.activeSceneName);

  ESP_DEBUG() << "CreateSceneInstance success =="
              << (success ? "true" : "false")
              << "for active scene name :" << config_.activeSceneName
              << (config_.createRenderer ? " with" : " without") << "renderer.";

  // Handle the NavMesh configuration
  if (config_.navMeshSettings != nullptr &&
      Cr::Utility::String::lowercase(config_.activeSceneName) != "none") {
    // If the NavMesh is unloaded or does not match the requested configuration
    // then recompute it.
    if (!pathfinder_->isLoaded() ||
        (pathfinder_->getNavMeshSettings() != *config_.navMeshSettings)) {
      ESP_DEBUG() << "NavMesh recompute was necessary.";
      recomputeNavMesh(*pathfinder_, *config_.navMeshSettings);
    }
  }

}  // Simulator::reconfigure

bool Simulator::createSceneInstance(const std::string& activeSceneName) {
  getRenderGLContext();

  // 1. initial setup for scene instancing - sets or creates the
  // current scene instance to correspond to the given name.

  // Get scene instance attributes corresponding to passed active scene name
  // This will retrieve, or construct, an appropriately configured scene
  // instance attributes, depending on what exists in the Scene Dataset library
  // for the current dataset.
  curSceneInstanceAttributes_ =
      metadataMediator_->getSceneInstanceAttributesByName(activeSceneName);

  // check if attributes is null - should not happen
  ESP_CHECK(
      curSceneInstanceAttributes_,
      Cr::Utility::formatString(
          "Simulator::setSceneInstanceAttributes() : Attempt to load scene "
          "instance :{} failed due to scene instance not being found. Aborting",
          activeSceneName));

  // 2. Load navmesh specified in current scene instance attributes.

  const std::string& navmeshFileHandle =
      curSceneInstanceAttributes_->getNavmeshHandle();
  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  if (navmeshFileHandle.empty()) {
    ESP_DEBUG() << "No navmesh file handle provided in scene instance.";

  } else {
    const std::string& navmeshFileLoc =
        metadataMediator_->getNavmeshPathByHandle(navmeshFileHandle);
    // Get name of navmesh and use to create pathfinder and load navmesh
    if (navmeshFileLoc.empty()) {
      ESP_DEBUG() << "No navmesh file location provided in scene dataset that "
                     "maps to handle :"
                  << navmeshFileHandle;
    } else if (Cr::Utility::Path::exists(navmeshFileLoc)) {
      ESP_DEBUG() << "Loading navmesh from" << navmeshFileLoc;
      bool pfSuccess = pathfinder_->loadNavMesh(navmeshFileLoc);
      ESP_DEBUG() << (pfSuccess ? "Navmesh Loaded." : "Navmesh load error.");
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Navmesh file not found, checked at filename : '" << navmeshFileLoc
          << "'";
    }
  }
  // Calling to seeding needs to be done after the pathfinder creation but
  // before anything else.
  seed(config_.randomSeed);

  // This code deletes the instances in the previously loaded scene from
  // gfx-replay. Because of the issue below, scene graphs are leaked, so we
  // cannot rely on node deletion to issue gfx-replay deletion entries.
  auto recorder = gfxReplayMgr_->getRecorder();
  if (recorder && activeSceneID_ >= 0 &&
      activeSceneID_ < sceneManager_->getSceneGraphCount()) {
    recorder->onHideSceneGraph(sceneManager_->getSceneGraph(activeSceneID_));
  }

  // initialize scene graph CAREFUL! previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when instancing a new scene, shall we delete all of the previous scene
  // graphs?
  activeSceneID_ = sceneManager_->initSceneGraph();
  sceneID_.push_back(activeSceneID_);

  // 3. Load the Semantic Scene Descriptor file appropriate for the current
  // scene instance.
  // get name of desired semantic scene descriptor file
  const std::string semanticSceneDescFilename =
      metadataMediator_->getSemanticSceneDescriptorPathByHandle(
          curSceneInstanceAttributes_->getSemanticSceneHandle());

  // load semantic scene descriptor
  resourceManager_->loadSemanticSceneDescriptor(semanticSceneDescFilename,
                                                activeSceneName);

  // 4. Specify frustumCulling based on value from config
  frustumCulling_ = config_.frustumCulling;

  // 5. (re)seat & (re)init physics manager using the physics manager
  // attributes specified in current simulator configuration held in
  // metadataMediator.
  // get rootNode
  auto& rootNode = sceneManager_->getSceneGraph(activeSceneID_).getRootNode();

  resourceManager_->initPhysicsManager(
      physicsManager_, &rootNode,
      metadataMediator_->getCurrentPhysicsManagerAttributes());
  // Set PM's reference to this simulator
  physicsManager_->setSimulator(this);

  // 6. Load lighting as specified for scene instance - perform before stage
  // load so lighting key can be set appropriately. get name of light setup
  // for this scene instance
  std::string lightSetupKey;
  if (config_.overrideSceneLightDefaults) {
    // SimulatorConfiguration set to override any dataset configuration specs
    // regarding lighting.
    lightSetupKey = config_.sceneLightSetupKey;
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Using SimulatorConfiguration-specified Light Setup key : `"
        // empty lightSetupKey denotes using the default
        << (lightSetupKey == DEFAULT_LIGHTING_KEY ? "DEFAULT_LIGHTING_KEY"
                                                  : lightSetupKey)
        << "`.";
  } else {
    // Get scene instance specified lighting
    lightSetupKey = metadataMediator_->getLightSetupFullHandle(
        curSceneInstanceAttributes_->getLightingHandle());
    ESP_DEBUG() << "Using scene instance-specified Light Setup key : `"
                // empty lightSetupKey denotes using the default
                << (lightSetupKey == DEFAULT_LIGHTING_KEY
                        ? "DEFAULT_LIGHTING_KEY"
                        : lightSetupKey)
                << "`.";
    // Do not query Scene Instance Attributes for lightsetup for these keys.
    // Both should already be handled by ResourceManager
    if ((lightSetupKey != NO_LIGHT_KEY) &&
        (lightSetupKey != DEFAULT_LIGHTING_KEY)) {
      // lighting attributes corresponding to this key should exist in
      // LightLayoutAttributesManager.
      esp::gfx::LightSetup lightingSetup =
          metadataMediator_->getLightLayoutAttributesManager()
              ->createLightSetupFromAttributes(lightSetupKey);
      // set lightsetup in resource manager
      resourceManager_->setLightSetup(std::move(lightingSetup),
                                      Mn::ResourceKey{lightSetupKey});
    }
  }
  // Set config's sceneLightSetupKey to track currently specified light setup
  // key
  config_.sceneLightSetupKey = lightSetupKey;

  // 7. Update MetadataMediator's copy of now-final SimulatorConfiguration to be
  // in sync, and set default PbrShaderAttributes based on current scene
  // instance
  metadataMediator_->setSimulatorConfiguration(config_);

  // Set default PbrShaderAttributes based on current scene instance
  metadataMediator_->setCurrDefaultPbrAttributesHandle(
      curSceneInstanceAttributes_->getDefaultPbrShaderAttributesHandle());
  // Set the mappings from region tags to handles
  metadataMediator_->setCurrScenePbrShaderRegionMap(
      curSceneInstanceAttributes_->getRegionPbrShaderAttributesHandles());

  // Update ResourceManager's loaded Pbr/Ibl assets based on most up to date
  // state of metadataMediator_'s currently active scene dataset.
  resourceManager_->loadAllIBLAssets();

  // 8. Load stage specified by Scene Instance Attributes
  bool success = instanceStageForSceneAttributes(curSceneInstanceAttributes_);
  // 9. Load object instances as specified by Scene Instance Attributes.
  if (success) {
    success = instanceObjectsForSceneAttributes(curSceneInstanceAttributes_);
    if (success) {
      // 10. Load articulated object instances as specified by Scene Instance
      // Attributes.
      success = instanceArticulatedObjectsForSceneAttributes(
          curSceneInstanceAttributes_);
      if (success) {
        // TODO : reset may eventually have all the scene instantiation code so
        // that scenes can be reset
        reset();
      }
    }
  }

  return success;
}  // Simulator::createSceneInstance

bool Simulator::instanceStageForSceneAttributes(
    const metadata::attributes::SceneInstanceAttributes::cptr&
        curSceneInstanceAttributes_) {
  // Load stage specified by Scene Instance Attributes
  // Get Stage Instance Attributes - contains name of stage and initial
  // transformation of stage in scene.
  // TODO : need to support stageInstanceAttributes transformation upon
  // creation.
  const SceneObjectInstanceAttributes::cptr stageInstanceAttributes =
      curSceneInstanceAttributes_->getStageInstance();

  // check if attributes is null - should not happen
  ESP_CHECK(
      stageInstanceAttributes,
      Cr::Utility::formatString(
          "Simulator::instanceStageForSceneAttributes() : Attempt to load "
          "stage instance specified in current scene instance :{} failed due "
          "to stage instance configuration not being found. Aborting",
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
      metadata::attributes::ObjectInstanceShaderType::Unspecified) {
    stageAttributes->setShaderType(getShaderTypeName(stageShaderType));
  }
  // set lighting key based on curent config value
  stageAttributes->setLightSetupKey(config_.sceneLightSetupKey);
  // set frustum culling from simulator config
  stageAttributes->setFrustumCulling(frustumCulling_);
  // set scaling values for this instance of stage attributes - first uniform
  // scaling
  stageAttributes->setScale(stageAttributes->getScale() *
                            stageInstanceAttributes->getUniformScale());
  // set scaling values for this instance of stage attributes - next non-uniform
  // scaling
  stageAttributes->setScale(stageAttributes->getScale() *
                            stageInstanceAttributes->getNonUniformScale());

  // this will only be true if semantic textures have been set to be available
  // from the dataset config
  stageAttributes->setUseSemanticTextures(config_.useSemanticTexturesIfFound);
  // set stage's ref to ssd file
  stageAttributes->setSemanticDescriptorFilename(
      metadataMediator_->getSemanticSceneDescriptorPathByHandle(
          curSceneInstanceAttributes_->getSemanticSceneHandle()));

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

  ESP_CHECK(loadSuccess, Cr::Utility::formatString("Cannot load stage :{}",
                                                   stageAttributesHandle));

  // get colormap from loading semantic scene mesh
  Cr::Containers::ArrayView<const Mn::Vector3ub> colorMap =
      resourceManager_->getSemanticSceneColormap();

  // sending semantic colormap to shader for visualizations.
  // This map may have color mappings specified in supported semantic screen
  // descriptor text files with vertex mappings to ids on semantic scene mesh.
  if (renderer_ && !colorMap.isEmpty()) {
    // send colormap to TextureVisualizeShader via renderer.
    renderer_->setSemanticVisualizerColormap(colorMap);
  }

  // if successful display debug message
  ESP_DEBUG() << "Successfully loaded stage named :"
              << stageAttributes->getHandle();

  // refresh the NavMesh visualization if necessary after loading a new
  // SceneGraph
  resetNavMeshVisIfActive();

  // set activeSemanticSceneID_ values and push onto sceneID vector if
  // appropriate - tempIDs[1] will either be old activeSemanticSceneID_ (if
  // no semantic mesh was requested in loadStage); ID_UNDEFINED if desired
  // was not found; activeSceneID_, or a unique value, the last of which means
  // the semantic scene mesh is loaded.

  if (activeSemanticSceneID_ != tempIDs[1]) {
    // check if semantic scene mesh has been loaded
    // assume it has if tempIDs[1] is different
    semanticSceneMeshLoaded_ = true;
    // id has changed so act - if ID has not changed, do nothing
    activeSemanticSceneID_ = tempIDs[1];
    if ((activeSemanticSceneID_ != ID_UNDEFINED) &&
        (activeSemanticSceneID_ != activeSceneID_)) {
      sceneID_.push_back(activeSemanticSceneID_);
    } else {  // activeSemanticSceneID_ == activeSceneID_;
      assets::AssetType stageType =
          static_cast<assets::AssetType>(stageAttributes->getRenderAssetType());
      // instance meshes contain their semantic annotations
      // empty scene has none to worry about
      if (!(stageType == assets::AssetType::INSTANCE_MESH ||
            stageAttributesHandle == assets::EMPTY_SCENE)) {
        semanticSceneMeshLoaded_ = false;
        // TODO: programmatic generation of semantic meshes when no
        // annotations are provided.
        ESP_WARNING() << "The active scene does not contain semantic "
                         "annotations : activeSemanticSceneID_ ="
                      << activeSemanticSceneID_;
      }
    }
  }  // if ID has changed - needs to be reset
  return true;
}  // Simulator::instanceStageForSceneAttributes()

bool Simulator::instanceObjectsForSceneAttributes(
    const metadata::attributes::SceneInstanceAttributes::cptr&
        curSceneInstanceAttributes_) {
  // Load object instances as specified by Scene Instance Attributes.
  // Get all instances of objects described in scene
  const std::vector<SceneObjectInstanceAttributes::cptr> objectInstances =
      curSceneInstanceAttributes_->getObjectInstances();

  // node to attach object to
  scene::SceneNode* attachmentNode = nullptr;

  // whether or not to correct for COM shift - only do for blender-sourced
  // scene attributes
  bool defaultCOMCorrection =
      (curSceneInstanceAttributes_->getTranslationOrigin() ==
       metadata::attributes::SceneInstanceTranslationOrigin::AssetLocal);

  // Iterate through instances, create object and implement initial
  // transformation.
  for (const auto& objInst : objectInstances) {
    // check if attributes is null - should not happen
    ESP_CHECK(
        objInst,
        Cr::Utility::formatString(
            "Simulator::instanceObjectsForSceneAttributes() : Attempt to load "
            "object instance specified in current scene instance :{} failed "
            "due to object instance configuration not being found. Aborting",
            config_.activeSceneName));

    const std::string objAttrFullHandle =
        metadataMediator_->getObjAttrFullHandle(objInst->getHandle());
    // make sure full handle is not empty
    ESP_CHECK(
        !objAttrFullHandle.empty(),
        Cr::Utility::formatString(
            "Simulator::instanceObjectsForSceneAttributes() : Attempt to load "
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
}  // Simulator::instanceObjectsForSceneAttributes()

bool Simulator::instanceArticulatedObjectsForSceneAttributes(
    const metadata::attributes::SceneInstanceAttributes::cptr&
        curSceneInstanceAttributes_) {
  // 6. Load all articulated object instances
  // Get all instances of articulated objects described in scene
  const std::vector<SceneAOInstanceAttributes::cptr> artObjInstances =
      curSceneInstanceAttributes_->getArticulatedObjectInstances();

  // Iterate through instances, create object and implement initial
  // transformation.
  for (const auto& artObjInst : artObjInstances) {
    // check if instance attributes is null - should not happen
    ESP_CHECK(artObjInst,
              Cr::Utility::formatString(
                  "Simulator::instanceArticulatedObjectsForSceneAttributes() "
                  ": Attempt to load articulated object instance "
                  "specified in current scene instance :{} failed due to "
                  "AO instance configuration not being found. Aborting",
                  config_.activeSceneName));

    // get model file name
    const std::string artObjAttrHandle =
        metadataMediator_->getArticulatedObjModelFullHandle(
            artObjInst->getHandle());

    // make sure full handle is not empty
    ESP_CHECK(
        !artObjAttrHandle.empty(),
        Cr::Utility::formatString(
            "Simulator::instanceArticulatedObjectsForSceneAttributes() : "
            "Attempt "
            "to load articulated object instance specified in current scene "
            "instance :{} failed due to AO instance configuration file handle "
            "'{}' being empty or unknown. Aborting",
            config_.activeSceneName, artObjInst->getHandle()));

    // create articulated object
    // aoID =
    physicsManager_->addArticulatedObjectInstance(artObjInst, artObjAttrHandle,
                                                  config_.sceneLightSetupKey);
  }  // for each articulated object instance
  return true;
}  // Simulator::instanceArticulatedObjectsForSceneAttributes

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

metadata::attributes::SceneInstanceAttributes::ptr
Simulator::buildCurrentStateSceneAttributes() const {
  // 1. Get SceneInstanceAttributes copy corresponding to initial scene setup
  // Get a ref to current scene dataset's SceneInstanceAttributesManager
  auto sceneAttrMgr = metadataMediator_->getSceneInstanceAttributesManager();
  // Get a copy of SceneInstanceAttributes used to create the scene
  auto initSceneInstanceAttr = sceneAttrMgr->getObjectCopyByHandle(
      curSceneInstanceAttributes_->getHandle());
  // Make sure this exists - this should never trigger, all scenes should be
  // built from SceneInstanceAttributes via reconfigure, and we're just asking
  // for a copy of curSceneInstanceAttributes_.
  ESP_CHECK(initSceneInstanceAttr,
            "Simulator::saveCurrentSceneInstance : SceneInstanceAttributes "
            "used to initialize current scene never registered or has been "
            "removed from SceneInstanceAttributesManager. Aborting.");

  // 2. Pass the copy to Physics Manager so that it can be updated with current
  // scene setup - stage instance, object instances and articulated object
  // instances
  physicsManager_->buildCurrentStateSceneAttributes(initSceneInstanceAttr);

  // 3. Return the copy
  return initSceneInstanceAttr;
}  // Simulator::buildCurrentStateSceneAttributes

void Simulator::seed(uint32_t newSeed) {
  random_->seed(newSeed);
  pathfinder_->seed(newSeed);
}
const std::shared_ptr<metadata::managers::AOAttributesManager>&
Simulator::getAOAttributesManager() const {
  return metadataMediator_->getAOAttributesManager();
}
const metadata::managers::AssetAttributesManager::ptr&
Simulator::getAssetAttributesManager() const {
  return metadataMediator_->getAssetAttributesManager();
}

const metadata::managers::LightLayoutAttributesManager::ptr&
Simulator::getLightLayoutAttributesManager() const {
  return metadataMediator_->getLightLayoutAttributesManager();
}

const metadata::managers::ObjectAttributesManager::ptr&
Simulator::getObjectAttributesManager() const {
  return metadataMediator_->getObjectAttributesManager();
}

const metadata::managers::PhysicsAttributesManager::ptr&
Simulator::getPhysicsAttributesManager() const {
  return metadataMediator_->getPhysicsAttributesManager();
}

const metadata::managers::StageAttributesManager::ptr&
Simulator::getStageAttributesManager() const {
  return metadataMediator_->getStageAttributesManager();
}

std::string Simulator::getActiveSceneDatasetName() {
  return metadataMediator_->getActiveSceneDatasetName();
}

void Simulator::setActiveSceneDatasetName(const std::string& _dsHandle) {
  metadataMediator_->setActiveSceneDatasetName(_dsHandle);
}

bool Simulator::saveCurrentSceneInstance(
    const std::string& saveFilename) const {
  if (sceneHasPhysics()) {
    ESP_DEBUG() << "Attempting to save current scene layout as "
                   "SceneInstanceAttributes with filename :"
                << saveFilename;
    return metadataMediator_->getSceneInstanceAttributesManager()
        ->saveManagedObjectToFile(buildCurrentStateSceneAttributes(),
                                  saveFilename);
  }
  return false;
}  // saveCurrentSceneInstance

bool Simulator::saveCurrentSceneInstance(bool overwrite) const {
  if (sceneHasPhysics()) {
    ESP_DEBUG() << "Attempting to save current scene layout as "
                   "SceneInstanceAttributes.";
    return metadataMediator_->getSceneInstanceAttributesManager()
        ->saveManagedObjectToFile(buildCurrentStateSceneAttributes(),
                                  overwrite);
  }
  return false;
}  // saveCurrentSceneInstance

void Simulator::reconfigureReplayManager(bool enableGfxReplaySave) {
  gfxReplayMgr_ = std::make_shared<gfx::replay::ReplayManager>();

  // construct Recorder instance if requested
  gfxReplayMgr_->setRecorder(enableGfxReplaySave
                                 ? std::make_shared<gfx::replay::Recorder>()
                                 : nullptr);
  // assign Recorder to ResourceManager
  CORRADE_INTERNAL_ASSERT(resourceManager_);
  resourceManager_->setRecorder(gfxReplayMgr_->getRecorder());

  // provide Player backend implementation to replay manager
  class SceneGraphPlayerImplementation
      : public gfx::replay::AbstractSceneGraphPlayerImplementation {
   public:
    explicit SceneGraphPlayerImplementation(Simulator& self) : self_{self} {}

   private:
    gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
        const assets::AssetInfo& assetInfo,
        const assets::RenderAssetInstanceCreationInfo& creation) override {
      return reinterpret_cast<gfx::replay::NodeHandle>(
          self_.loadAndCreateRenderAssetInstance(assetInfo, creation));
    }

    void changeLightSetup(const gfx::LightSetup& lights) override {
      self_.setLightSetup(lights);
    }

    Simulator& self_;
  };
  gfxReplayMgr_->setPlayerImplementation(
      std::make_shared<SceneGraphPlayerImplementation>(*this));
}

// === Physics Simulator Functions ===

double Simulator::stepWorld(const double dt) {
  if (physicsManager_ != nullptr) {
    physicsManager_->deferNodesUpdate();
    physicsManager_->stepPhysics(dt);
    if (renderer_) {
      renderer_->waitSceneGraph();
    }

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

double Simulator::getPhysicsTimeStep() {
  if (physicsManager_ != nullptr) {
    return physicsManager_->getTimestep();
  }
  return -1;
}

bool Simulator::recomputeNavMesh(nav::PathFinder& pathfinder,
                                 const nav::NavMeshSettings& navMeshSettings) {
  assets::MeshData::ptr joinedMesh =
      getJoinedMesh(navMeshSettings.includeStaticObjects);

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

assets::MeshData::ptr Simulator::getJoinedMesh(
    const bool includeStaticObjects) {
  assets::MeshData::ptr joinedMesh = assets::MeshData::create();
  auto stageInitAttrs = physicsManager_->getStageInitAttributes();
  if (stageInitAttrs != nullptr) {
    joinedMesh = resourceManager_->createJoinedCollisionMesh(
        stageInitAttrs->getRenderAssetHandle());
  }

  // add STATIC collision objects
  if (includeStaticObjects) {
    // update nodes so SceneNode transforms are up-to-date
    if (renderer_) {
      renderer_->waitSceneGraph();
    }

    physicsManager_->updateNodes();

    // collect mesh components from all objects and then merge them.
    // Each mesh component could be duplicated multiple times w/ different
    // transforms.
    std::map<std::string,
             std::vector<Eigen::Transform<float, 3, Eigen::Affine>>>
        meshComponentStates;
    auto rigidObjMgr = getRigidObjectManager();
    // collect RigidObject mesh components
    for (auto objectID : physicsManager_->getExistingObjectIDs()) {
      auto objWrapper = rigidObjMgr->getObjectCopyByID(objectID);
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
  ESP_CHECK(joinedMesh->vbo.size() > 0,
            "::recomputeNavMesh: "
            "Unable to compute a navmesh upon a non-existent mesh - "
            "the underlying joined collision mesh has no vertices. This is "
            "probably due to the current scene being NONE. Aborting");

  return joinedMesh;
}

assets::MeshData::ptr Simulator::getJoinedSemanticMesh(
    std::vector<std::uint16_t>& objectIds) {
  assets::MeshData::ptr joinedSemanticMesh = assets::MeshData::create();
  auto stageInitAttrs = physicsManager_->getStageInitAttributes();
  if (stageInitAttrs != nullptr) {
    joinedSemanticMesh = resourceManager_->createJoinedSemanticCollisionMesh(
        objectIds, stageInitAttrs->getSemanticAssetHandle());
  }

  return joinedSemanticMesh;
}

bool Simulator::setNavMeshVisualization(bool visualize) {
  getRenderGLContext();

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

esp::physics::ManagedArticulatedObject::ptr
Simulator::queryArticulatedObjWrapper(int objID) const {
  if (!sceneHasPhysics()) {
    return nullptr;
  }
  return getArticulatedObjectManager()->getObjectCopyByID(objID);
}

void Simulator::setMetadataMediator(
    metadata::MetadataMediator::ptr _metadataMediator) {
  metadataMediator_ = std::move(_metadataMediator);
  // set newly added MM to have current Simulator Config
  metadataMediator_->setSimulatorConfiguration(this->config_);
}

scene::SceneNode* Simulator::loadAndCreateRenderAssetInstance(
    const assets::AssetInfo& assetInfo,
    const assets::RenderAssetInstanceCreationInfo& creation) {
  getRenderGLContext();

  // Note this pattern of passing the scene manager and two scene ids to
  // resource manager. This is similar to ResourceManager::loadStage.
  std::vector<int> tempIDs{activeSceneID_, activeSemanticSceneID_};
  return resourceManager_->loadAndCreateRenderAssetInstance(
      assetInfo, creation, sceneManager_.get(), tempIDs);
}

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
    ag->getControls()->setMoveFilterFunction(std::move(moveFilterFunction));
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
  getRenderGLContext();

  esp::sensor::SensorSetup sensorSpecifications = {sensorSpec};
  esp::scene::SceneNode& objectNode =
      *(getRigidObjectManager()->getObjectCopyByID(objectId)->getSceneNode());
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

bool Simulator::visualizeObservation(int agentId, const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor& sensor = ag->getSubtreeSensorSuite().get(sensorId);
    if (sensor.isVisualSensor()) {
      renderer_->visualize(static_cast<sensor::VisualSensor&>(sensor));
    }
    return true;
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

std::vector<std::string> Simulator::getRuntimePerfStatNames() {
  return {"num rigid",
          "num active rigid",
          "num artic",
          "num active overlaps",
          "num active contacts",
          "num drawables",
          "num faces"};
}

std::vector<float> Simulator::getRuntimePerfStatValues() {
  int drawableCount = 0;
  int drawableNumFaces = 0;
  std::tie(drawableCount, drawableNumFaces) =
      resourceManager_->getDrawableCountAndNumFaces();

  runtimePerfStatValues_.clear();
  runtimePerfStatValues_.push_back(physicsManager_->getNumRigidObjects());
  runtimePerfStatValues_.push_back(physicsManager_->checkActiveObjects());
  runtimePerfStatValues_.push_back(physicsManager_->getNumArticulatedObjects());
  runtimePerfStatValues_.push_back(
      physicsManager_->getNumActiveOverlappingPairs());
  runtimePerfStatValues_.push_back(
      physicsManager_->getNumActiveContactPoints());
  runtimePerfStatValues_.push_back(drawableCount);
  runtimePerfStatValues_.push_back(drawableNumFaces);

  return runtimePerfStatValues_;
}

}  // namespace sim
}  // namespace esp
