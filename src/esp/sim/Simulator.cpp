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

#include "esp/core/esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/Recorder.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletDebugManager.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/SensorFactory.h"
#include "esp/sensor/VisualSensor.h"

namespace Cr = Corrade;

namespace esp {
namespace sim {

using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::SceneObjectInstanceAttributes;
using metadata::attributes::StageAttributes;

Simulator::Simulator(const SimulatorConfiguration& cfg,
                     metadata::MetadataMediator::ptr _metadataMediator)
    : metadataMediator_{std::move(_metadataMediator)},
      random_{core::Random::create(cfg.randomSeed)},
      requiresTextures_{Cr::Containers::NullOpt} {
  // initalize members according to cfg
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

Simulator::~Simulator() {
  LOG(INFO) << "Deconstructing Simulator";
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

  if (destroy) {
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
    resourceManager_ = std::make_unique<assets::ResourceManager>(
        metadataMediator_, assets::ResourceManager::Flag::BuildPhongFromPbr);
    if (cfg.createRenderer) {
      // needs to be called after ResourceManager exists but before any assets
      // have been loaded
      reconfigureReplayManager(cfg.enableGfxReplaySave);
    }
  } else {
    resourceManager_->setMetadataMediator(metadataMediator_);
  }

  if (!sceneManager_) {
    sceneManager_ = scene::SceneManager::create_unique();
  }

  // if configuration is unchanged, just reset and return
  if (cfg == config_) {
    reset();
    return;
  }
  // otherwise set current configuration and initialize
  // TODO can optimize to do partial re-initialization instead of from-scratch
  config_ = cfg;

  if (requiresTextures_ == Cr::Containers::NullOpt) {
    requiresTextures_ = config_.requiresTextures;
    resourceManager_->setRequiresTextures(config_.requiresTextures);
  } else if (!(*requiresTextures_) && config_.requiresTextures) {
    throw std::runtime_error(
        "requiresTextures was changed to True from False.  Must call close() "
        "before changing this value.");
  } else if ((*requiresTextures_) && !config_.requiresTextures) {
    LOG(WARNING) << "Not changing requiresTextures as the simulator was "
                    "initialized with True.  Call close() to change this.";
  }

  bool success = false;
  // (re) create scene instance based on whether or not a renderer is requested.
  if (config_.createRenderer) {
    /* When creating a viewer based app, there is no need to create a
    WindowlessContext since a (windowed) context already exists. */
    if (!context_ && !Magnum::GL::Context::hasCurrent()) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    // reinitalize members
    if (!renderer_) {
      gfx::Renderer::Flags flags;
      if (!(*requiresTextures_))
        flags |= gfx::Renderer::Flag::NoTextures;

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
      if (context_)
        flags |= gfx::Renderer::Flag::BackgroundThread;
#endif

      renderer_ = gfx::Renderer::create(context_.get(), flags);
    }

    renderer_->acquireGlContext();

    // (re) create scene instance
    success = createSceneInstance(config_.activeSceneName);
  } else {
    // (re) create scene instance without renderer
    success = createSceneInstanceNoRenderer(config_.activeSceneName);
  }

  LOG(INFO) << "Simulator::reconfigure : createSceneInstance success == "
            << (success ? "true" : "false")
            << " for active scene name : " << config_.activeSceneName
            << (config_.createRenderer ? " with" : " without") << " renderer.";

}  // Simulator::reconfigure

metadata::attributes::SceneAttributes::cptr
Simulator::setSceneInstanceAttributes(const std::string& activeSceneName) {
  namespace FileUtil = Cr::Utility::Directory;

  // This should always/only be called by either createSceneInstance or
  // createSceneInstanceNoRendere.

  // Get scene instance attributes corresponding to passed active scene name
  // This will retrieve, or construct, an appropriately configured scene
  // instance attributes, depending on what exists in the Scene Dataset library
  // for the current dataset.

  metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
      metadataMediator_->getSceneAttributesByName(activeSceneName);

  // 1. Load navmesh specified in current scene instance attributes.

  const std::string& navmeshFileLoc = metadataMediator_->getNavmeshPathByHandle(
      curSceneInstanceAttributes->getNavmeshHandle());

  LOG(INFO)
      << "Simulator::setSceneInstanceAttributes : Navmesh file location in "
         "scene instance : "
      << navmeshFileLoc;
  // Get name of navmesh and use to create pathfinder and load navmesh
  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  if (FileUtil::exists(navmeshFileLoc)) {
    LOG(INFO) << "Simulator::setSceneInstanceAttributes : Loading navmesh from "
              << navmeshFileLoc;
    bool pfSuccess = pathfinder_->loadNavMesh(navmeshFileLoc);
    LOG(INFO) << "Simulator::setSceneInstanceAttributes : "
              << (pfSuccess ? "Navmesh Loaded." : "Navmesh load error.");
  } else {
    LOG(WARNING)
        << "Simulator::setSceneInstanceAttributes : Navmesh file not found, "
           "checked at filename : '"
        << navmeshFileLoc << "'";
  }
  // Calling to seeding needs to be done after the pathfinder creation but
  // before anything else.
  seed(config_.randomSeed);

  // initalize scene graph CAREFUL! previous scene graph is not deleted!
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

  if (semanticSceneDescFilename.compare("") != 0) {
    bool fileExists = false;
    bool success = false;
    const std::string msgPrefix =
        "Simulator::setSceneInstanceAttributes : Attempt to load ";
    // semantic scene descriptor might not exist, so
    semanticScene_ = nullptr;
    semanticScene_ = scene::SemanticScene::create();
    LOG(INFO) << "Simulator::setSceneInstanceAttributes : SceneInstance : "
              << activeSceneName
              << " proposed Semantic Scene Descriptor filename : "
              << semanticSceneDescFilename;

    // Attempt to load semantic scene descriptor specified in scene instance
    // file, agnostic to file type inferred by name,
    success = scene::SemanticScene::loadSemanticSceneDescriptor(
        semanticSceneDescFilename, *semanticScene_);
    if (!success) {
      // attempt to look for specified file failed, attempt to build new file
      // name by searching in path specified of specified file for
      // info_semantic.json file for replica dataset
      const std::string tmpFName = FileUtil::join(
          FileUtil::path(semanticSceneDescFilename), "info_semantic.json");
      if (FileUtil::exists(tmpFName)) {
        success =
            scene::SemanticScene::loadReplicaHouse(tmpFName, *semanticScene_);
        LOG(INFO) << msgPrefix
                  << "Replica w/existing constructed file : " << tmpFName
                  << " in directory with " << semanticSceneDescFilename << " : "
                  << (success ? "" : "not ") << "successful";
      }
    }  // if given SSD file name specifiedd exists
    LOG(WARNING)
        << "Simulator::setSceneInstanceAttributes : All attempts to load "
           "SSD with SceneAttributes-provided name "
        << semanticSceneDescFilename << " : exist : " << fileExists
        << " : loaded as expected type : " << success;

  }  // if semantic scene descriptor specified in scene instance

  // 3. Specify frustumCulling based on value either from config (if override
  // is specified) or from scene instance attributes.
  frustumCulling_ = config_.frustumCulling;

  // return a const ptr to the cur scene instance attributes
  return curSceneInstanceAttributes;

}  // Simulator::setSceneInstanceAttributes

bool Simulator::createSceneInstance(const std::string& activeSceneName) {
  if (renderer_)
    renderer_->acquireGlContext();
  // 1. initial setup for scene instancing - sets or creates the
  // current scene instance to correspond to the given name.
  metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
      setSceneInstanceAttributes(activeSceneName);

  // get sceneGraph and rootNode
  auto& sceneGraph = sceneManager_->getSceneGraph(activeSceneID_);
  auto& rootNode = sceneGraph.getRootNode();

  // 2. (re)seat & (re)init physics manager using the physics manager
  // attributes specified in current simulator configuration held in
  // metadataMediator.
  resourceManager_->initPhysicsManager(
      physicsManager_, config_.enablePhysics, &rootNode,
      metadataMediator_->getCurrentPhysicsManagerAttributes());

  // 3. Load lighting as specified for scene instance - perform before stage
  // load so lighting key can be set appropriately. get name of light setup
  // for this scene instance
  std::string lightSetupKey;

  if (config_.overrideSceneLightDefaults) {
    lightSetupKey = config_.sceneLightSetup;
    LOG(INFO) << "Simulator::createSceneInstance : Using config-specified "
                 "Light key : -"
              << lightSetupKey << "-";
  } else {
    lightSetupKey = metadataMediator_->getLightSetupFullHandle(
        curSceneInstanceAttributes->getLightingHandle());
    LOG(INFO)
        << "Simulator::createSceneInstance : Using scene instance-specified "
           "Light key : -"
        << lightSetupKey << "-";
    if (lightSetupKey.compare(NO_LIGHT_KEY) != 0) {
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

  // 4. Load stage specified by Scene Instance Attributes
  // Get Stage Instance Attributes - contains name of stage and initial
  // transformation of stage in scene.
  // TODO : need to support stageInstanceAttributes transformation upon
  // creation.

  const SceneObjectInstanceAttributes::ptr stageInstanceAttributes =
      curSceneInstanceAttributes->getStageInstance();

  // Get full library name of StageAttributes
  const std::string stageAttributesHandle =
      metadataMediator_->getStageAttrFullHandle(
          stageInstanceAttributes->getHandle());
  // Get StageAttributes
  auto stageAttributes =
      metadataMediator_->getStageAttributesManager()->getObjectCopyByHandle(
          stageAttributesHandle);

  // constant representing unknown shader type
  const int unknownShaderType =
      static_cast<int>(metadata::attributes::ObjectInstanceShaderType::Unknown);

  // set defaults for stage creation

  // set shader type to use for stage
  int stageShaderType = stageInstanceAttributes->getShaderType();
  if (stageShaderType != unknownShaderType) {
    stageAttributes->setShaderType(stageShaderType);
  }
  // set lighting key
  stageAttributes->setLightSetup(lightSetupKey);
  // set frustum culling from simulator config
  stageAttributes->setFrustumCulling(frustumCulling_);

  // create a structure to manage active scene and active semantic scene ID
  // passing to and from loadStage
  std::vector<int> tempIDs{activeSceneID_, activeSemanticSceneID_};
  LOG(INFO) << "Simulator::createSceneInstance : Start to load stage named : "
            << stageAttributes->getHandle() << " with render asset : "
            << stageAttributes->getRenderAssetHandle()
            << " and collision asset : "
            << stageAttributes->getCollisionAssetHandle();

  // Load stage
  bool loadSuccess = resourceManager_->loadStage(
      stageAttributes, physicsManager_, sceneManager_.get(), tempIDs,
      config_.loadSemanticMesh, config_.forceSeparateSemanticSceneGraph);

  if (!loadSuccess) {
    LOG(ERROR) << "Simulator::createSceneInstance : Cannot load stage : "
               << stageAttributesHandle;
    // Pass the error to the python through pybind11 allowing graceful exit
    throw std::invalid_argument(
        "Simulator::createSceneInstance : Cannot load: " +
        stageAttributesHandle);
  } else {
    LOG(INFO) << "Simulator::createSceneInstance : Successfully loaded stage "
                 "named : "
              << stageAttributes->getHandle();
  }

  // refresh the NavMesh visualization if necessary after loading a new
  // SceneGraph
  if (isNavMeshVisualizationActive()) {
    // if updating pathfinder_ instance, refresh the visualization.
    setNavMeshVisualization(false);  // first clear the old instance
    setNavMeshVisualization(true);
  }

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
    } else {  // activeSemanticSceneID_ = activeSceneID_;
      assets::AssetType stageType =
          static_cast<assets::AssetType>(stageAttributes->getRenderAssetType());
      // instance meshes and suncg houses contain their semantic annotations
      // empty scene has none to worry about
      if (!(stageType == assets::AssetType::SUNCG_SCENE ||
            stageType == assets::AssetType::INSTANCE_MESH ||
            stageAttributesHandle.compare(assets::EMPTY_SCENE) == 0)) {
        // TODO: programmatic generation of semantic meshes when no
        // annotations are provided.
        LOG(WARNING) << "\n---\nSimulator::createSceneInstance : The active "
                        "scene does not contain semantic "
                        "annotations. \n---";
      }
    }
  }  // if ID has changed - needs to be reset

  // 5. Load object instances as spceified by Scene Instance Attributes.

  // Get all instances of objects described in scene
  const std::vector<SceneObjectInstanceAttributes::ptr> objectInstances =
      curSceneInstanceAttributes->getObjectInstances();

  // current scene graph's drawables
  auto& drawables = sceneGraph.getDrawables();
  // node to attach object to
  scene::SceneNode* attachmentNode = nullptr;
  // vector holding all objects added
  std::vector<int> objectsAdded;
  int objID = 0;

  // whether or not to correct for COM shift - only do for blender-sourced
  // scene attributes
  bool Default_COM_Correction =
      (static_cast<metadata::managers::SceneInstanceTranslationOrigin>(
           curSceneInstanceAttributes->getTranslationOrigin()) ==
       metadata::managers::SceneInstanceTranslationOrigin::AssetLocal);

  std::string errMsgTmplt =
      "Simulator::createSceneInstance : Error instancing scene : " +
      activeSceneName + " : ";
  // Iterate through instances, create object and implement initial
  // transformation.
  for (const auto& objInst : objectInstances) {
    const std::string objAttrFullHandle =
        metadataMediator_->getObjAttrFullHandle(objInst->getHandle());
    if (objAttrFullHandle == "") {
      LOG(ERROR) << errMsgTmplt
                 << "Unable to find objectAttributes whose handle contains "
                 << objInst->getHandle()
                 << " as specified in object instance attributes.";
      return false;
    }

    // Get ObjectAttributes
    auto objAttributes =
        metadataMediator_->getObjectAttributesManager()->getObjectCopyByHandle(
            objAttrFullHandle);
    if (!objAttributes) {
      LOG(ERROR) << errMsgTmplt
                 << "Missing/improperly configured objectAttributes "
                 << objAttrFullHandle << ", whose handle contains "
                 << objInst->getHandle()
                 << " as specified in object instance attributes.";
      return false;
    }
    // set shader type to use for stage
    int objShaderType = objInst->getShaderType();
    if (objShaderType != unknownShaderType) {
      objAttributes->setShaderType(objShaderType);
    }

    // create object using attributes copy.
    objID = physicsManager_->addObject(objAttributes, &drawables,
                                       attachmentNode, lightSetupKey);
    if (objID == ID_UNDEFINED) {
      // instancing failed for some reason.
      LOG(ERROR) << errMsgTmplt << "Object create failed for objectAttributes "
                 << objAttrFullHandle << ", whose handle contains "
                 << objInst->getHandle()
                 << " as specified in object instance attributes.";
      return false;
    }
    // set object's location and rotation based on translation and rotation
    // params specified in instance attributes
    auto translate = objInst->getTranslation();
    // get instance override value, if exists
    auto Instance_COM_Origin =
        static_cast<metadata::managers::SceneInstanceTranslationOrigin>(
            objInst->getTranslationOrigin());
    if (((Default_COM_Correction) &&
         (Instance_COM_Origin !=
          metadata::managers::SceneInstanceTranslationOrigin::COM)) ||
        (Instance_COM_Origin ==
         metadata::managers::SceneInstanceTranslationOrigin::AssetLocal)) {
      // if default COM correction is set and no object-based override, or if
      // Object set to correct for COM.

      translate -= objInst->getRotation().transformVector(
          physicsManager_->getObjectVisualSceneNodes(objID)[0]->translation());
    }
    physicsManager_->setTranslation(objID, translate);
    physicsManager_->setRotation(objID, objInst->getRotation());
    // set object's motion type if different than set value
    const physics::MotionType attrObjMotionType =
        static_cast<physics::MotionType>(objInst->getMotionType());
    if (attrObjMotionType != physics::MotionType::UNDEFINED) {
      physicsManager_->setObjectMotionType(objID, attrObjMotionType);
    }
    objectsAdded.push_back(objID);
  }  // for each object attributes
  // objectsAdded holds all ids of added objects.

  // TODO : reset may eventually have all the scene instance instantiation
  // code so that scenes can be reset
  reset();

  return true;
}  // Simulator::createSceneInstance

bool Simulator::createSceneInstanceNoRenderer(
    const std::string& activeSceneName) {
  // Initial setup for scene instancing without renderer - sets or creates the
  // current scene instance to correspond to the given name.  Also builds
  // navmesh and semantic scene descriptor file if appropriate.
  metadata::attributes::SceneAttributes::cptr curSceneInstanceAttributes =
      setSceneInstanceAttributes(activeSceneName);

  // TODO : reset may eventually have all the scene instance instantiation
  // code so that scenes can be reset
  reset();
  return true;
}  // Simulator::createSceneInstanceNoRenderer

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

scene::SceneGraph& Simulator::getActiveSceneGraph() {
  CORRADE_INTERNAL_ASSERT(std::size_t(activeSceneID_) < sceneID_.size());
  return sceneManager_->getSceneGraph(activeSceneID_);
}

//! return the semantic scene's SceneGraph for rendering
scene::SceneGraph& Simulator::getActiveSemanticSceneGraph() {
  CORRADE_INTERNAL_ASSERT(std::size_t(activeSemanticSceneID_) <
                          sceneID_.size());
  return sceneManager_->getSceneGraph(activeSemanticSceneID_);
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
    auto& sceneGraph = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph.getDrawables();
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
    auto& sceneGraph = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph.getDrawables();
    return physicsManager_->addObject(objectLibHandle, &drawables,
                                      attachmentNode, lightSetupKey);
  }
  return ID_UNDEFINED;
}

const metadata::attributes::ObjectAttributes::cptr
Simulator::getObjectInitializationTemplate(const int objectId,
                                           const int sceneID) const {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getObjectInitAttributes(objectId);
  }
  return nullptr;
}

const metadata::attributes::StageAttributes::cptr
Simulator::getStageInitializationTemplate(const int sceneID) const {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getStageInitAttributes();
  }
  return nullptr;
}

// return a list of existing objected IDs in a physical scene
std::vector<int> Simulator::getExistingObjectIDs(const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getExistingObjectIDs();
  }
  return std::vector<int>();  // empty if no simulator exists
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

esp::physics::MotionType Simulator::getObjectMotionType(const int objectID,
                                                        const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getObjectMotionType(objectID);
  }
  return esp::physics::MotionType::UNDEFINED;
}

void Simulator::setObjectMotionType(const esp::physics::MotionType& motionType,
                                    const int objectID,
                                    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setObjectMotionType(objectID, motionType);
  }
}

physics::VelocityControl::ptr Simulator::getObjectVelocityControl(
    const int objectID,
    const int sceneID) const {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getVelocityControl(objectID);
  }
  return nullptr;
}

// apply forces and torques to objects
void Simulator::applyTorque(const Magnum::Vector3& tau,
                            const int objectID,
                            const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->applyTorque(objectID, tau);
  }
}

void Simulator::applyForce(const Magnum::Vector3& force,
                           const Magnum::Vector3& relPos,
                           const int objectID,
                           const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->applyForce(objectID, force, relPos);
  }
}

void Simulator::applyImpulse(const Magnum::Vector3& impulse,
                             const Magnum::Vector3& relPos,
                             const int objectID,
                             const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->applyImpulse(objectID, impulse, relPos);
  }
}

scene::SceneNode* Simulator::getObjectSceneNode(const int objectID,
                                                const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return &physicsManager_->getObjectSceneNode(objectID);
  }
  return nullptr;
}

std::vector<scene::SceneNode*> Simulator::getObjectVisualSceneNodes(
    const int objectID,
    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getObjectVisualSceneNodes(objectID);
  }
  return std::vector<scene::SceneNode*>();
}

// set object transform (kinemmatic control)
void Simulator::setTransformation(const Magnum::Matrix4& transform,
                                  const int objectID,
                                  const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setTransformation(objectID, transform);
  }
}

Magnum::Matrix4 Simulator::getTransformation(const int objectID,
                                             const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getTransformation(objectID);
  }
  return Magnum::Matrix4::fromDiagonal(Magnum::Vector4(1));
}

esp::core::RigidState Simulator::getRigidState(const int objectID,
                                               const int sceneID) const {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getRigidState(objectID);
  }
  return esp::core::RigidState();
}

void Simulator::setRigidState(const esp::core::RigidState& rigidState,
                              const int objectID,
                              const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setRigidState(objectID, rigidState);
  }
}

// set object translation directly
void Simulator::setTranslation(const Magnum::Vector3& translation,
                               const int objectID,
                               const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setTranslation(objectID, translation);
  }
}

Magnum::Vector3 Simulator::getTranslation(const int objectID,
                                          const int sceneID) {
  // can throw if physicsManager is not initialized or either objectID/sceneID
  // is invalid
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getTranslation(objectID);
  }
  return Magnum::Vector3();
}

// set object orientation directly
void Simulator::setRotation(const Magnum::Quaternion& rotation,
                            const int objectID,
                            const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setRotation(objectID, rotation);
  }
}

Magnum::Quaternion Simulator::getRotation(const int objectID,
                                          const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getRotation(objectID);
  }
  return Magnum::Quaternion();
}

void Simulator::setLinearVelocity(const Magnum::Vector3& linVel,
                                  const int objectID,
                                  const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->setLinearVelocity(objectID, linVel);
  }
}

Magnum::Vector3 Simulator::getLinearVelocity(const int objectID,
                                             const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getLinearVelocity(objectID);
  }
  return Magnum::Vector3();
}

void Simulator::setAngularVelocity(const Magnum::Vector3& angVel,
                                   const int objectID,
                                   const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->setAngularVelocity(objectID, angVel);
  }
}

Magnum::Vector3 Simulator::getAngularVelocity(const int objectID,
                                              const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getAngularVelocity(objectID);
  }
  return Magnum::Vector3();
}

bool Simulator::contactTest(const int objectID,
                            bool staticAsStage,
                            const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->contactTest(objectID, staticAsStage);
  }
  return false;
}

std::vector<esp::physics::ContactPointData> Simulator::getPhysicsContactPoints(
    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getContactPoints();
  }
  return {};
}

esp::physics::RaycastResults Simulator::castRay(const esp::geo::Ray& ray,
                                                float maxDistance,
                                                const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->castRay(ray, maxDistance);
  }
  return esp::physics::RaycastResults();
}

void Simulator::setObjectBBDraw(bool drawBB,
                                const int objectID,
                                const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    if (drawBB && renderer_)
      renderer_->acquireGlContext();
    auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
    physicsManager_->setObjectBBDraw(objectID, &drawables, drawBB);
  }
}

#ifdef ESP_BUILD_WITH_VHACD
void Simulator::createObjectVoxelization(int objectID, int resolution) {
  physicsManager_->generateVoxelization(objectID, resolution);
}
#endif

void Simulator::setObjectVoxelizationDraw(bool drawV,
                                          int objectID,
                                          const std::string& gridName) {
  auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
  auto& drawables = sceneGraph_.getDrawables();
  physicsManager_->setObjectVoxelizationDraw(objectID, gridName, &drawables,
                                             drawV);
}

std::shared_ptr<esp::geo::VoxelWrapper> Simulator::getObjectVoxelization(
    int objectID) {
  return physicsManager_->getObjectVoxelization(objectID);
}

#ifdef ESP_BUILD_WITH_VHACD
void Simulator::createStageVoxelization(int resolution) {
  physicsManager_->generateStageVoxelization(resolution);
}
#endif

void Simulator::setStageVoxelizationDraw(bool drawV,
                                         const std::string& gridName) {
  auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
  auto& drawables = sceneGraph_.getDrawables();
  physicsManager_->setStageVoxelizationDraw(gridName, &drawables, drawV);
}

std::shared_ptr<esp::geo::VoxelWrapper> Simulator::getStageVoxelization() {
  return physicsManager_->getStageVoxelization();
}

void Simulator::setObjectSemanticId(uint32_t semanticId,
                                    const int objectID,
                                    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setSemanticId(objectID, semanticId);
  }
}

double Simulator::stepWorld(const double dt) {
  if (physicsManager_ != nullptr) {
    physicsManager_->deferNodesUpdate();
    physicsManager_->stepPhysics(dt);
#if !defined(CORRADE_TARGET_EMSCRIPTEN)
    if (renderer_)
      renderer_->waitSG();
#endif
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

void Simulator::setGravity(const Magnum::Vector3& gravity, const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setGravity(gravity);
  }
}

Magnum::Vector3 Simulator::getGravity(const int sceneID) const {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getGravity();
  }
  return Magnum::Vector3();
}

bool Simulator::recomputeNavMesh(nav::PathFinder& pathfinder,
                                 const nav::NavMeshSettings& navMeshSettings,
                                 bool includeStaticObjects) {
  CORRADE_ASSERT(config_.createRenderer,
                 "Simulator::recomputeNavMesh: "
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
    physicsManager_->updateNodes();

    // collect mesh components from all objects and then merge them.
    // Each mesh component could be duplicated multiple times w/ different
    // transforms.
    std::map<std::string,
             std::vector<Eigen::Transform<float, 3, Eigen::Affine>>>
        meshComponentStates;

    // collect RigidObject mesh components
    for (auto& objectID : physicsManager_->getExistingObjectIDs()) {
      if (physicsManager_->getObjectMotionType(objectID) ==
          physics::MotionType::STATIC) {
        auto objectTransform = Magnum::EigenIntegration::cast<
            Eigen::Transform<float, 3, Eigen::Affine>>(
            physicsManager_->getObjectVisualSceneNode(objectID)
                .absoluteTransformationMatrix());
        const metadata::attributes::ObjectAttributes::cptr
            initializationTemplate =
                physicsManager_->getObjectInitAttributes(objectID);
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
    for (auto& objectID : physicsManager_->getExistingArticulatedObjectIDs()) {
      if (physicsManager_->getArticulatedObjectMotionType(objectID) ==
          physics::MotionType::STATIC) {
        for (int linkIx = -1;
             linkIx < physicsManager_->getNumArticulatedLinks(objectID);
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
    LOG(ERROR) << "Failed to build navmesh";
    return false;
  }

  if (&pathfinder == pathfinder_.get()) {
    if (isNavMeshVisualizationActive()) {
      // if updating pathfinder_ instance, refresh the visualization.
      setNavMeshVisualization(false);  // first clear the old instance
      setNavMeshVisualization(true);
    }
  }

  LOG(INFO) << "reconstruct navmesh successful";
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
      LOG(ERROR) << "Simulator::toggleNavMeshVisualization : Failed to load "
                    "navmesh visualization.";
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
  auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
  auto& drawables = sceneGraph_.getDrawables();

  // 1. create trajectory tube asset from points and save it
  bool success = resourceManager_->buildTrajectoryVisualization(
      trajVisName, pts, numSegments, radius, color, smooth, numInterp);
  if (!success) {
    LOG(ERROR) << "Simulator::showTrajectoryVisualization : Failed to create "
                  "Trajectory visualization mesh for "
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
    LOG(ERROR) << "Simulator::showTrajectoryVisualization : Failed to create "
                  "Trajectory visualization object for "
               << trajVisName;
    // TODO : support removing asset by removing from resourceDict_ properly
    // using trajVisName
    return ID_UNDEFINED;
  }
  LOG(INFO) << "Simulator::showTrajectoryVisualization : Trajectory "
               "visualization object created with ID "
            << trajVisID;
  physicsManager_->setObjectMotionType(trajVisID,
                                       esp::physics::MotionType::KINEMATIC);
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
    LOG(ERROR) << "No loaded PathFinder, aborting sampleRandomAgentState.";
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
  Cr::Utility::Debug() << "VHACD PARAMS RESOLUTION: " << params.m_resolution;

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

  // if the renderChd flag is set to true, set the convex hull decomposition to
  // be the render asset (useful for testing)

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
  ASSERT(0 <= agentId && agentId < agents_.size());
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

nav::PathFinder::ptr Simulator::getPathFinder() {
  return pathfinder_;
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

void Simulator::setLightSetup(gfx::LightSetup setup, const std::string& key) {
  resourceManager_->setLightSetup(std::move(setup), key);
}

gfx::LightSetup Simulator::getLightSetup(const std::string& key) {
  return *resourceManager_->getLightSetup(key);
}

void Simulator::setObjectLightSetup(const int objectID,
                                    const std::string& lightSetupKey,
                                    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    physicsManager_->setObjectLightSetup(objectID, lightSetupKey);
  }
}

//===============================================================================//
// Articulated Object API (UNSTABLE!)

int Simulator::addArticulatedObjectFromURDF(const std::string& filepath,
                                            bool fixedBase,
                                            float globalScale,
                                            float massScale,
                                            bool forceReload) {
  if (sceneHasPhysics(0)) {
    if (renderer_)
      renderer_->acquireGlContext();
    auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
    return physicsManager_->addArticulatedObjectFromURDF(
        filepath, &drawables, fixedBase, globalScale, massScale, forceReload);
  }
  Corrade::Utility::Debug()
      << "Simulator::loadURDF : failed - physics not enabled.";
  return esp::ID_UNDEFINED;
}

void Simulator::removeArticulatedObject(int objectId) {
  if (sceneHasPhysics(0)) {
    physicsManager_->removeArticulatedObject(objectId);
  }
}

scene::SceneNode* Simulator::getArticulatedLinkSceneNode(int objectID,
                                                         int linkId) {
  return &physicsManager_->getArticulatedLinkSceneNode(objectID, linkId);
}

std::vector<scene::SceneNode*> Simulator::getArticulatedLinkVisualSceneNodes(
    int objectID,
    int linkId) {
  return physicsManager_->getArticulatedLinkVisualSceneNodes(objectID, linkId);
}

std::vector<int> Simulator::getExistingArticulatedObjectIDs(
    CORRADE_UNUSED const int sceneID) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getExistingArticulatedObjectIDs();
  }
  return std::vector<int>();
}

void Simulator::setArticulatedObjectRootState(int objectId,
                                              const Magnum::Matrix4& state) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectRootState(objectId, state);
  }
};

const Magnum::Matrix4 Simulator::getArticulatedObjectRootState(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectRootState(objectId);
  }
  return Magnum::Matrix4();
};

void Simulator::setArticulatedObjectForces(int objectId,
                                           const std::vector<float>& forces) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectForces(objectId, forces);
  }
};

void Simulator::setArticulatedObjectVelocities(int objectId,
                                               const std::vector<float>& vels) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectVelocities(objectId, vels);
  }
};

void Simulator::setArticulatedObjectPositions(
    int objectId,
    const std::vector<float>& positions) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectPositions(objectId, positions);
  }
};

std::vector<float> Simulator::getArticulatedObjectPositions(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectPositions(objectId);
  }
  return std::vector<float>();
};

std::vector<float> Simulator::getArticulatedObjectVelocities(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectVelocities(objectId);
  }
  return std::vector<float>();
};

std::vector<float> Simulator::getArticulatedObjectForces(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectForces(objectId);
  }
  return std::vector<float>();
};

std::vector<float> Simulator::getArticulatedObjectPositionLimits(
    int objectId,
    bool upperLimits) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectPositionLimits(objectId,
                                                               upperLimits);
  }
  return std::vector<float>();
}

void Simulator::setAutoClampJointLimits(int objectId, bool autoClamp) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setAutoClampJointLimits(objectId, autoClamp);
  }
}

bool Simulator::getAutoClampJointLimits(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getAutoClampJointLimits(objectId);
  }
  return false;
}

void Simulator::resetArticulatedObject(int objectId) {
  if (sceneHasPhysics(0)) {
    physicsManager_->resetArticulatedObject(objectId);
  }
};

void Simulator::setArticulatedObjectSleep(int objectId, bool sleep) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectSleep(objectId, sleep);
  }
};

bool Simulator::getArticulatedObjectSleep(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectSleep(objectId);
  }
  return false;
}

void Simulator::setArticulatedObjectMotionType(int objectId,
                                               esp::physics::MotionType mt) {
  if (sceneHasPhysics(0)) {
    physicsManager_->setArticulatedObjectMotionType(objectId, mt);
  }
};

esp::physics::MotionType Simulator::getArticulatedObjectMotionType(
    int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedObjectMotionType(objectId);
  }
  return esp::physics::MotionType::UNDEFINED;
};

int Simulator::getNumArticulatedLinks(int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getNumArticulatedLinks(objectId);
  }
  return ID_UNDEFINED;
};

core::RigidState Simulator::getArticulatedLinkRigidState(int objectId,
                                                         int linkId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getArticulatedLinkRigidState(objectId, linkId);
  }
  return core::RigidState();
};

// --- Joint Motor API --- //
//-------------------------//

int Simulator::createJointMotor(
    const int objectId,
    const int dof,
    const esp::physics::JointMotorSettings& settings) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->createJointMotor(objectId, dof, settings);
  }
  return ID_UNDEFINED;
}

void Simulator::removeJointMotor(const int objectId, const int motorId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->removeJointMotor(objectId, motorId);
  }
}

esp::physics::JointMotorSettings Simulator::getJointMotorSettings(
    const int objectId,
    const int motorId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getJointMotorSettings(objectId, motorId);
  }
  return {};
}

void Simulator::updateJointMotor(
    const int objectId,
    const int motorId,
    const esp::physics::JointMotorSettings& settings) {
  if (sceneHasPhysics(0)) {
    physicsManager_->updateJointMotor(objectId, motorId, settings);
  }
}

std::map<int, int> Simulator::getExistingJointMotors(const int objectId) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->getExistingJointMotors(objectId);
  }
  return std::map<int, int>();
}

std::map<int, int> Simulator::createMotorsForAllDofs(
    const int objectId,
    esp::physics::JointMotorSettings settings) {
  if (sceneHasPhysics(0)) {
    return physicsManager_->createMotorsForAllDofs(objectId, settings);
  }
  return std::map<int, int>();
}

// END: Articulated Object API (UNSTABLE!)
//===============================================================================//

}  // namespace sim
}  // namespace esp
