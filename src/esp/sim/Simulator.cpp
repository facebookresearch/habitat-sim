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

#include "esp/core/esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/PinholeCamera.h"
#include "esp/sensor/VisualSensor.h"

namespace Cr = Corrade;

namespace esp {
namespace sim {

using metadata::attributes::PhysicsManagerAttributes;
using metadata::attributes::StageAttributes;

Simulator::Simulator(const SimulatorConfiguration& cfg)
    : random_{core::Random::create(cfg.randomSeed)},
      requiresTextures_{Cr::Containers::NullOpt} {
  // initalize members according to cfg
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

Simulator::~Simulator() {
  LOG(INFO) << "Deconstructing Simulator";
  close();
}

void Simulator::close() {
  pathfinder_ = nullptr;
  navMeshVisPrimID_ = esp::ID_UNDEFINED;
  navMeshVisNode_ = nullptr;
  agents_.clear();

  physicsManager_ = nullptr;
  semanticScene_ = nullptr;

  sceneID_.clear();
  sceneManager_ = nullptr;

  resourceManager_ = nullptr;

  renderer_ = nullptr;
  context_ = nullptr;

  activeSceneID_ = ID_UNDEFINED;
  activeSemanticSceneID_ = ID_UNDEFINED;
  config_ = SimulatorConfiguration{};

  frustumCulling_ = true;
  requiresTextures_ = Cr::Containers::NullOpt;
}

void Simulator::reconfigure(const SimulatorConfiguration& cfg) {
  // set dataset upon creation or reconfigure
  if (!metadataMediator_) {
    metadataMediator_ =
        metadata::MetadataMediator::create(cfg.sceneDatasetConfigFile);
  } else {
    metadataMediator_->setActiveSceneDatasetName(cfg.sceneDatasetConfigFile);
  }
  // assign MM to RM on create or reconfigure
  if (!resourceManager_) {
    resourceManager_ =
        std::make_unique<assets::ResourceManager>(metadataMediator_);
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

  // use physics attributes manager to get physics manager attributes
  // described by config file - this always exists to configure scene
  // attributes
  auto physicsManagerAttributes =
      metadataMediator_->getPhysicsAttributesManager()->createObject(
          config_.physicsConfigFile, true);
  // if physicsManagerAttributes have been successfully created, inform
  // stageAttributesManager of the config handle of the attributes, so that
  // stageAttributes initialization can use phys Mgr Attr values as defaults
  auto stageAttributesMgr = metadataMediator_->getStageAttributesManager();
  if (physicsManagerAttributes != nullptr) {
    stageAttributesMgr->setCurrPhysicsManagerAttributesHandle(
        physicsManagerAttributes->getHandle());
  }
  // set scene attributes defaults to cfg-based values, i.e. to construct
  // default semantic and navmesh file names, if they exist.  All values
  // set/built from these default values may be overridden by values in scene
  // json file, if present.
  stageAttributesMgr->setCurrCfgVals(
      config_.scene.filepaths, config_.sceneLightSetup, config_.frustumCulling);

  // Build scene file name based on config specification
  std::string stageFilename = config_.scene.id;
  if (config_.scene.filepaths.count("mesh")) {
    stageFilename = config_.scene.filepaths.at("mesh");
  }

  // Create scene attributes with values based on sceneFilename
  auto stageAttributes = stageAttributesMgr->createObject(stageFilename, true);

  std::string navmeshFilename = stageAttributes->getNavmeshAssetHandle();
  std::string houseFilename = stageAttributes->getHouseFilename();

  esp::assets::AssetType stageType = static_cast<esp::assets::AssetType>(
      stageAttributes->getRenderAssetType());

  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  if (io::exists(navmeshFilename)) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);
    LOG(INFO) << "Loaded.";
  } else {
    LOG(WARNING) << "Navmesh file not found, checked at " << navmeshFilename;
  }

  // Calling to seeding needs to be done after the pathfinder creation
  seed(config_.randomSeed);

  // initalize scene graph
  // CAREFUL!
  // previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when doing reconfigure, shall we delete all of the previous scene graphs

  activeSceneID_ = sceneManager_->initSceneGraph();

  // LOG(INFO) << "Active scene graph ID = " << activeSceneID_;
  sceneID_.push_back(activeSceneID_);

  if (config_.createRenderer) {
    /* When creating a viewer based app, there is no need to create a
    WindowlessContext since a (windowed) context already exists. */
    if (!context_ && !Magnum::GL::Context::hasCurrent()) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    // reinitalize members
    if (!renderer_) {
      gfx::Renderer::Flags flags;
      if (!config_.requiresTextures)
        flags |= gfx::Renderer::Flag::NoTextures;
      renderer_ = gfx::Renderer::create(flags);
    }

    auto& sceneGraph = sceneManager_->getSceneGraph(activeSceneID_);
    auto& rootNode = sceneGraph.getRootNode();
    // auto& drawables = sceneGraph.getDrawables();

    bool loadSuccess = false;

    // (re)seat & (re)init physics manager
    resourceManager_->initPhysicsManager(physicsManager_, config_.enablePhysics,
                                         &rootNode, physicsManagerAttributes);

    std::vector<int> tempIDs{activeSceneID_, activeSemanticSceneID_};
    // Load scene
    loadSuccess = resourceManager_->loadStage(stageAttributes, physicsManager_,
                                              sceneManager_.get(), tempIDs,
                                              config_.loadSemanticMesh);

    if (!loadSuccess) {
      LOG(ERROR) << "Cannot load " << stageFilename;
      // Pass the error to the python through pybind11 allowing graceful exit
      throw std::invalid_argument("Cannot load: " + stageFilename);
    }

    // refresh the NavMesh visualization if necessary after loading a new
    // SceneGraph
    if (isNavMeshVisualizationActive()) {
      // if updating pathfinder_ instance, refresh the visualization.
      setNavMeshVisualization(false);  // first clear the old instance
      setNavMeshVisualization(true);
    }

    const Magnum::Range3D& sceneBB = rootNode.computeCumulativeBB();
    resourceManager_->setLightSetup(gfx::getDefaultLights());

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
        // instance meshes and suncg houses contain their semantic annotations
        // empty scene has none to worry about
        if (!(stageType == assets::AssetType::SUNCG_SCENE ||
              stageType == assets::AssetType::INSTANCE_MESH ||
              stageFilename.compare(assets::EMPTY_SCENE) == 0)) {
          // TODO: programmatic generation of semantic meshes when no
          // annotations are provided.
          LOG(WARNING) << ":\n---\n The active scene does not contain semantic "
                          "annotations. \n---";
        }
      }
    }  // if ID has changed - needs to be reset
  }    // if (config_.createRenderer)

  semanticScene_ = nullptr;
  semanticScene_ = scene::SemanticScene::create();
  switch (stageType) {
    case assets::AssetType::INSTANCE_MESH:
      houseFilename = Cr::Utility::Directory::join(
          Cr::Utility::Directory::path(houseFilename), "info_semantic.json");
      if (io::exists(houseFilename)) {
        scene::SemanticScene::loadReplicaHouse(houseFilename, *semanticScene_);
      }
      break;
    case assets::AssetType::MP3D_MESH:
      // TODO(msb) Fix AssetType determination logic.
      if (io::exists(houseFilename)) {
        using Corrade::Utility::String::endsWith;
        if (endsWith(houseFilename, ".house")) {
          scene::SemanticScene::loadMp3dHouse(houseFilename, *semanticScene_);
        } else if (endsWith(houseFilename, ".scn")) {
          scene::SemanticScene::loadGibsonHouse(houseFilename, *semanticScene_);
        }
      }
      break;
    case assets::AssetType::SUNCG_SCENE:
      scene::SemanticScene::loadSuncgHouse(stageFilename, *semanticScene_);
      break;
    default:
      break;
  }

  reset();
}  // Simulator::reconfigure

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

scene::SceneGraph& Simulator::getActiveSceneGraph() {
  CHECK_GE(activeSceneID_, 0);
  CHECK_LT(activeSceneID_, sceneID_.size());
  return sceneManager_->getSceneGraph(activeSceneID_);
}

//! return the semantic scene's SceneGraph for rendering
scene::SceneGraph& Simulator::getActiveSemanticSceneGraph() {
  CHECK_GE(activeSemanticSceneID_, 0);
  CHECK_LT(activeSemanticSceneID_, sceneID_.size());
  return sceneManager_->getSceneGraph(activeSemanticSceneID_);
}

// === Physics Simulator Functions ===

int Simulator::addObject(const int objectLibId,
                         scene::SceneNode* attachmentNode,
                         const std::string& lightSetupKey,
                         const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    // TODO: change implementation to support multi-world and physics worlds
    // to own reference to a sceneGraph to avoid this.
    auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
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
    // TODO: change implementation to support multi-world and physics worlds
    // to own reference to a sceneGraph to avoid this.
    auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
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
  }
}

esp::physics::MotionType Simulator::getObjectMotionType(const int objectID,
                                                        const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->getObjectMotionType(objectID);
  }
  return esp::physics::MotionType::UNDEFINED;
}

bool Simulator::setObjectMotionType(const esp::physics::MotionType& motionType,
                                    const int objectID,
                                    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->setObjectMotionType(objectID, motionType);
  }
  return false;
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

bool Simulator::contactTest(const int objectID, const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->contactTest(objectID);
  }
  return false;
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
    auto& sceneGraph_ = sceneManager_->getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
    physicsManager_->setObjectBBDraw(objectID, &drawables, drawBB);
  }
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
    physicsManager_->stepPhysics(dt);
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
    for (auto objectID : physicsManager_->getExistingObjectIDs()) {
      if (physicsManager_->getObjectMotionType(objectID) ==
          physics::MotionType::STATIC) {
        auto objectTransform = Magnum::EigenIntegration::cast<
            Eigen::Transform<float, 3, Eigen::Affine> >(
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
        assets::MeshData::uptr joinedObjectMesh =
            resourceManager_->createJoinedCollisionMesh(meshHandle);
        int prevNumIndices = joinedMesh->ibo.size();
        int prevNumVerts = joinedMesh->vbo.size();
        joinedMesh->ibo.resize(prevNumIndices + joinedObjectMesh->ibo.size());
        for (size_t ix = 0; ix < joinedObjectMesh->ibo.size(); ++ix) {
          joinedMesh->ibo[ix + prevNumIndices] =
              joinedObjectMesh->ibo[ix] + prevNumVerts;
        }
        joinedMesh->vbo.reserve(joinedObjectMesh->vbo.size() + prevNumVerts);
        for (auto& vert : joinedObjectMesh->vbo) {
          joinedMesh->vbo.push_back(objectTransform * vert);
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

agent::Agent::ptr Simulator::addAgent(
    const agent::AgentConfiguration& agentConfig,
    scene::SceneNode& agentParentNode) {
  // initialize the agent, as well as all the sensors on it.

  // attach each agent, each sensor to a scene node, set the local
  // transformation of the sensor w.r.t. the agent (done internally in the
  // constructor of Agent)

  auto& agentNode = agentParentNode.createChild();
  agent::Agent::ptr ag = agent::Agent::create(agentNode, agentConfig);

  agent::AgentState state;
  sampleRandomAgentState(state);
  ag->setInitialState(state);

  // Add a RenderTarget to each of the agent's sensors
  for (auto& it : ag->getSensorSuite().getSensors()) {
    if (it.second->isVisualSensor()) {
      auto sensor = static_cast<sensor::VisualSensor*>(it.second.get());
      renderer_->bindRenderTarget(*sensor);
    }
  }

  agents_.push_back(ag);
  // TODO: just do this once
  if (pathfinder_->isLoaded()) {
    ag->getControls()->setMoveFilterFunction(
        [&](const vec3f& start, const vec3f& end) {
          return pathfinder_->tryStep(start, end);
        });
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
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr && sensor->isVisualSensor()) {
      return &(std::static_pointer_cast<sensor::VisualSensor>(sensor)
                   ->renderTarget());
    }
  }
  return nullptr;
}

bool Simulator::displayObservation(const int agentId,
                                   const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->displayObservation(*this);
    }
  }
  return false;
}

bool Simulator::drawObservation(const int agentId,
                                const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return std::static_pointer_cast<sensor::VisualSensor>(sensor)
          ->drawObservation(*this);
    }
  }
  return false;
}

bool Simulator::getAgentObservation(const int agentId,
                                    const std::string& sensorId,
                                    sensor::Observation& observation) {
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->getObservation(*this, observation);
    }
  }
  return false;
}

int Simulator::getAgentObservations(
    const int agentId,
    std::map<std::string, sensor::Observation>& observations) {
  observations.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    const std::map<std::string, sensor::Sensor::ptr>& sensors =
        ag->getSensorSuite().getSensors();
    for (std::pair<std::string, sensor::Sensor::ptr> s : sensors) {
      sensor::Observation obs;
      if (s.second->getObservation(*this, obs)) {
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
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->getObservationSpace(space);
    }
  }
  return false;
}

int Simulator::getAgentObservationSpaces(
    const int agentId,
    std::map<std::string, sensor::ObservationSpace>& spaces) {
  spaces.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    const std::map<std::string, sensor::Sensor::ptr>& sensors =
        ag->getSensorSuite().getSensors();
    for (std::pair<std::string, sensor::Sensor::ptr> s : sensors) {
      sensor::ObservationSpace space;
      if (s.second->getObservationSpace(space)) {
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
    gfx::setLightSetupForSubTree(physicsManager_->getObjectSceneNode(objectID),
                                 lightSetupKey);
  }
}

}  // namespace sim
}  // namespace esp
