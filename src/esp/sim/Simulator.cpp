// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <string>

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>

#include "esp/assets/Attributes.h"
#include "esp/core/esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/PinholeCamera.h"

namespace Cr = Corrade;

namespace esp {
namespace sim {

Simulator::Simulator(const SimulatorConfiguration& cfg) {
  // initalize members according to cfg
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

Simulator::~Simulator() {
  LOG(INFO) << "Deconstructing Simulator";
}

void Simulator::reconfigure(const SimulatorConfiguration& cfg) {
  // if configuration is unchanged, just reset and return
  if (cfg == config_) {
    reset();
    return;
  }
  // otherwise set current configuration and initialize
  // TODO can optimize to do partial re-initialization instead of from-scratch
  config_ = cfg;

  // load scene
  std::string sceneFilename = cfg.scene.id;
  if (cfg.scene.filepaths.count("mesh")) {
    sceneFilename = cfg.scene.filepaths.at("mesh");
  }

  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  std::string navmeshFilename = io::changeExtension(sceneFilename, ".navmesh");
  if (cfg.scene.filepaths.count("navmesh")) {
    navmeshFilename = cfg.scene.filepaths.at("navmesh");
  }
  if (io::exists(navmeshFilename)) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);
    LOG(INFO) << "Loaded.";
  } else {
    LOG(WARNING) << "Navmesh file not found, checked at " << navmeshFilename;
  }

  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (!io::exists(houseFilename)) {
    houseFilename = io::changeExtension(sceneFilename, ".scn");
  }
  if (cfg.scene.filepaths.count("house")) {
    houseFilename = cfg.scene.filepaths.at("house");
  }

  if (!io::exists(houseFilename)) {
    houseFilename = io::changeExtension(sceneFilename, ".scn");
  }

  assets::AssetInfo sceneInfo = assets::AssetInfo::fromPath(sceneFilename);
  sceneInfo.requiresLighting =
      cfg.sceneLightSetup != assets::ResourceManager::NO_LIGHT_KEY;

  // initalize scene graph
  // CAREFUL!
  // previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when doing reconfigure, shall we delete all of the previous scene graphs
  activeSceneID_ = sceneManager_.initSceneGraph();

  // LOG(INFO) << "Active scene graph ID = " << activeSceneID_;
  sceneID_.push_back(activeSceneID_);

  if (cfg.createRenderer) {
    if (!context_) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    // reinitalize members
    if (!renderer_) {
      renderer_ = gfx::Renderer::create();
    }

    auto& sceneGraph = sceneManager_.getSceneGraph(activeSceneID_);

    auto& rootNode = sceneGraph.getRootNode();
    auto& drawables = sceneGraph.getDrawables();
    resourceManager_.compressTextures(cfg.compressTextures);

    bool loadSuccess = false;
    if (config_.enablePhysics) {
      loadSuccess = resourceManager_.loadScene(
          sceneInfo, physicsManager_, &rootNode, &drawables,
          cfg.sceneLightSetup, cfg.physicsConfigFile);
    } else {
      loadSuccess = resourceManager_.loadScene(sceneInfo, &rootNode, &drawables,
                                               cfg.sceneLightSetup);
    }
    if (!loadSuccess) {
      LOG(ERROR) << "cannot load " << sceneFilename;
      // Pass the error to the python through pybind11 allowing graceful exit
      throw std::invalid_argument("Cannot load: " + sceneFilename);
    }
    const Magnum::Range3D& sceneBB = rootNode.computeCumulativeBB();
    resourceManager_.setLightSetup(gfx::getLightsAtBoxCorners(sceneBB));

    if (io::exists(houseFilename)) {
      LOG(INFO) << "Loading house from " << houseFilename;
      // if semantic mesh exists, load it as well
      // TODO: remove hardcoded filename change and use SceneConfiguration
      const std::string semanticMeshFilename =
          io::removeExtension(houseFilename) + "_semantic.ply";
      if (io::exists(semanticMeshFilename)) {
        LOG(INFO) << "Loading semantic mesh " << semanticMeshFilename;
        activeSemanticSceneID_ = sceneManager_.initSceneGraph();
        sceneID_.push_back(activeSemanticSceneID_);
        auto& semanticSceneGraph =
            sceneManager_.getSceneGraph(activeSemanticSceneID_);
        auto& semanticRootNode = semanticSceneGraph.getRootNode();
        auto& semanticDrawables = semanticSceneGraph.getDrawables();
        const assets::AssetInfo semanticSceneInfo =
            assets::AssetInfo::fromPath(semanticMeshFilename);
        resourceManager_.loadScene(
            semanticSceneInfo, &semanticRootNode, &semanticDrawables,
            assets::ResourceManager::NO_LIGHT_KEY, cfg.frustumCulling);
      }
      LOG(INFO) << "Loaded.";
    } else {
      activeSemanticSceneID_ = activeSceneID_;
      // instance meshes and suncg houses contain their semantic annotations
      // empty scene has none to worry about
      if (!(sceneInfo.type == assets::AssetType::SUNCG_SCENE ||
            sceneInfo.type == assets::AssetType::INSTANCE_MESH ||
            sceneFilename.compare(assets::EMPTY_SCENE) == 0)) {
        // TODO: programmatic generation of semantic meshes when no annotations
        // are provided.
        LOG(WARNING) << ":\n---\n The active scene does not contain semantic "
                        "annotations. \n---";
      }
    }
  }

  semanticScene_ = nullptr;
  semanticScene_ = scene::SemanticScene::create();
  switch (sceneInfo.type) {
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
      scene::SemanticScene::loadSuncgHouse(sceneFilename, *semanticScene_);
      break;
    default:
      break;
  }

  reset();
}

void Simulator::reset() {
  if (physicsManager_ != nullptr) {
    // TODO: this does nothing yet... desired reset behavior?
    physicsManager_->reset();
  }

  for (auto& agent : agents_) {
    agent->reset();
  }
  const Magnum::Range3D& sceneBB =
      getActiveSceneGraph().getRootNode().computeCumulativeBB();
  resourceManager_.setLightSetup(gfx::getLightsAtBoxCorners(sceneBB));
}

void Simulator::seed(uint32_t newSeed) {
  random_.seed(newSeed);
  pathfinder_->seed(newSeed);
}

std::shared_ptr<gfx::Renderer> Simulator::getRenderer() {
  return renderer_;
}

std::shared_ptr<physics::PhysicsManager> Simulator::getPhysicsManager() {
  return physicsManager_;
}

std::shared_ptr<scene::SemanticScene> Simulator::getSemanticScene() {
  return semanticScene_;
}

scene::SceneGraph& Simulator::getActiveSceneGraph() {
  CHECK_GE(activeSceneID_, 0);
  CHECK_LT(activeSceneID_, sceneID_.size());
  return sceneManager_.getSceneGraph(activeSceneID_);
}

//! return the semantic scene's SceneGraph for rendering
scene::SceneGraph& Simulator::getActiveSemanticSceneGraph() {
  CHECK_GE(activeSemanticSceneID_, 0);
  CHECK_LT(activeSemanticSceneID_, sceneID_.size());
  return sceneManager_.getSceneGraph(activeSemanticSceneID_);
}

bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.scene == b.scene && a.defaultAgentId == b.defaultAgentId &&
         a.defaultCameraUuid == b.defaultCameraUuid &&
         a.compressTextures == b.compressTextures &&
         a.createRenderer == b.createRenderer &&
         a.enablePhysics == b.enablePhysics &&
         a.physicsConfigFile.compare(b.physicsConfigFile) == 0;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

// === Physics Simulator Functions ===

int Simulator::addObject(int objectLibIndex,
                         scene::SceneNode* attachmentNode,
                         const std::string& lightSetupKey,
                         int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    // TODO: change implementation to support multi-world and physics worlds to
    // own reference to a sceneGraph to avoid this.
    auto& sceneGraph_ = sceneManager_.getSceneGraph(activeSceneID_);
    auto& drawables = sceneGraph_.getDrawables();
    return physicsManager_->addObject(objectLibIndex, &drawables,
                                      attachmentNode, lightSetupKey);
  }
  return ID_UNDEFINED;
}

// return the current size of the physics object library (objects [0,size) can
// be instanced)
int Simulator::getPhysicsObjectLibrarySize() {
  return resourceManager_.getNumLibraryObjects();
}

assets::PhysicsObjectAttributes& Simulator::getObjectTemplate(int templateId) {
  return resourceManager_.getPhysicsObjectAttributes(templateId);
}

std::vector<int> Simulator::loadObjectConfigs(const std::string& path) {
  std::vector<int> templateIndices;
  std::vector<std::string> validConfigPaths =
      resourceManager_.getObjectConfigPaths(path);
  for (auto& validPath : validConfigPaths) {
    templateIndices.push_back(
        resourceManager_.parseAndLoadPhysObjTemplate(validPath));
  }
  return templateIndices;
}

int Simulator::loadObjectTemplate(
    assets::PhysicsObjectAttributes& objectTemplate,
    const std::string& objectTemplateHandle) {
  // check for duplicate keys
  if (resourceManager_.getObjectTemplateID(objectTemplateHandle) !=
      ID_UNDEFINED) {
    return ID_UNDEFINED;
  }

  return resourceManager_.loadObjectTemplate(objectTemplate,
                                             objectTemplateHandle);
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
  return esp::physics::MotionType::ERROR_MOTIONTYPE;
}

bool Simulator::setObjectMotionType(const esp::physics::MotionType& motionType,
                                    const int objectID,
                                    const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->setObjectMotionType(objectID, motionType);
  }
  return false;
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

bool Simulator::contactTest(const int objectID, const int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    return physicsManager_->contactTest(objectID);
  }
  return false;
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

bool Simulator::recomputeNavMesh(nav::PathFinder& pathfinder,
                                 const nav::NavMeshSettings& navMeshSettings,
                                 bool includeStaticObjects) {
  CORRADE_ASSERT(
      config_.createRenderer,
      "Simulator::recomputeNavMesh: SimulatorConfiguration::createRenderer is "
      "false. Scene geometry is required to recompute navmesh. No geometry is "
      "loaded without renderer initialization.",
      false);

  assets::MeshData::uptr joinedMesh =
      resourceManager_.createJoinedCollisionMesh(config_.scene.id);

  // add STATIC collision objects
  if (includeStaticObjects) {
    for (auto objectID : physicsManager_->getExistingObjectIDs()) {
      if (physicsManager_->getObjectMotionType(objectID) ==
          physics::MotionType::STATIC) {
        auto objectTransform = Magnum::EigenIntegration::cast<
            Eigen::Transform<float, 3, Eigen::Affine> >(
            physicsManager_->getObjectVisualSceneNode(objectID)
                .absoluteTransformationMatrix());
        const assets::PhysicsObjectAttributes& initializationTemplate =
            physicsManager_->getInitializationAttributes(objectID);
        objectTransform.scale(Magnum::EigenIntegration::cast<vec3f>(
            initializationTemplate.getScale()));
        std::string meshHandle =
            initializationTemplate.getCollisionMeshHandle();
        if (meshHandle.empty()) {
          meshHandle = initializationTemplate.getRenderMeshHandle();
        }
        assets::MeshData::uptr joinedObjectMesh =
            resourceManager_.createJoinedCollisionMesh(meshHandle);
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

  LOG(INFO) << "reconstruct navmesh successful";
  return true;
}

// Agents
void Simulator::sampleRandomAgentState(agent::AgentState& agentState) {
  if (pathfinder_->isLoaded()) {
    agentState.position = pathfinder_->getRandomNavigablePoint();
    const float randomAngleRad = random_.uniform_float_01() * M_PI;
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

agent::Agent::ptr Simulator::getAgent(int agentId) {
  ASSERT(0 <= agentId && agentId < agents_.size());
  return agents_[agentId];
}

nav::PathFinder::ptr Simulator::getPathFinder() {
  return pathfinder_;
}

bool Simulator::displayObservation(int agentId, const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->displayObservation(*this);
    }
  }
  return false;
}

bool Simulator::getAgentObservation(int agentId,
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
    int agentId,
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

bool Simulator::getAgentObservationSpace(int agentId,
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
    int agentId,
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
  resourceManager_.setLightSetup(std::move(setup), key);
}

gfx::LightSetup Simulator::getLightSetup(const std::string& key) {
  return *resourceManager_.getLightSetup(key);
}

void Simulator::setObjectLightSetup(int objectID,
                                    const std::string& lightSetupKey,
                                    int sceneID) {
  if (sceneHasPhysics(sceneID)) {
    gfx::setLightSetupForSubTree(physicsManager_->getObjectSceneNode(objectID),
                                 lightSetupKey);
  }
}

}  // namespace sim
}  // namespace esp
