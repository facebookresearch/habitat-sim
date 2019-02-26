// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <Corrade/Containers/Pointer.h>
#include <Corrade/Utility/String.h>

#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/AbstractImageConverter.h>

#include "Drawable.h"

#include "esp/agent/Agent.h"
#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"
#include "esp/nav/ActionSpacePath.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/PinholeCamera.h"

using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

namespace esp {
namespace gfx {

Simulator::Simulator(const SimulatorConfiguration& cfg)
    : context_(cfg.gpuDeviceId) {
  // initalize members according to cfg
  reconfigure(cfg);
}

Simulator::~Simulator() {
  LOG(INFO) << "Deconstructing Simulator";
}

void Simulator::sampleRandomAgentState(agent::AgentState::ptr agentState) {
  agentState->position = pathfinder_->getRandomNavigablePoint();
  const float randomAngleRad = random_.uniform_float_01() * M_PI;
  quatf rotation(Eigen::AngleAxisf(randomAngleRad, vec3f::UnitY()));
  agentState->rotation = rotation.coeffs();
  // TODO: any other AgentState members should be randomized?
}

std::shared_ptr<agent::Agent> Simulator::addAgent(
    const agent::AgentConfiguration& agentConfig,
    scene::SceneNode& agentParentNode) {
  // initialize the agent, as well as all the sensors on it.

  // attach each agent, each sensor to a scene node, set the local
  // transformation of the sensor w.r.t. the agent (done internally in the
  // constructor of Agent)

  auto& agentNode = agentParentNode.createChild();
  std::shared_ptr<agent::Agent> agent =
      std::make_shared<agent::Agent>(agentConfig, agentNode);
  agents_.push_back(agent);

  return agent;
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

  // check that at least one agent with at least one sensor exists
  const int numAgents = cfg.agents.size();
  if (numAgents == 0) {
    LOG(ERROR) << "No agent configurations";
    return;
  }
  if (cfg.defaultAgentId >= numAgents) {
    LOG(ERROR) << "Selected default agent configuration not available";
    return;
  }
  const agent::AgentConfiguration& agentConfig = *cfg.agents[0];
  const int numSensors = agentConfig.sensorSpecifications.size();
  if (numSensors == 0) {
    LOG(ERROR) << "No sensor specifications";
    return;
  }

  // Get first agent's first sensor by default
  const sensor::SensorSpec::ptr firstSpec = agentConfig.sensorSpecifications[0];
  const int height = firstSpec->resolution[0];
  const int width = firstSpec->resolution[1];

  // reinitalize members
  if (renderer_) {
    renderer_.reset();
  }
  renderer_ = Renderer::create(width, height);

  // initalize scene graph
  // CAREFUL!
  // previous scene graph is not deleted!
  // TODO:
  // We need to make a design decision here:
  // when doing reconfigure, shall we delete all of the previous scene graphs
  activeSceneID_ = sceneManager_.initSceneGraph();
  // LOG(INFO) << "Active scene graph ID = " << activeSceneID_;
  sceneID_.push_back(activeSceneID_);
  auto& sceneGraph = sceneManager_.getSceneGraph(activeSceneID_);

  // load scene
  std::string sceneFilename = cfg.scene.id;
  if (cfg.scene.filepaths.count("mesh")) {
    sceneFilename = cfg.scene.filepaths.at("mesh");
  }
  auto& rootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();
  const assets::AssetInfo sceneInfo =
      assets::AssetInfo::fromPath(sceneFilename);
  resourceManager_.compressTextures(cfg.compressTextures);
  if (!resourceManager_.loadScene(sceneInfo, &rootNode, &drawables)) {
    LOG(ERROR) << "cannot load " << sceneFilename;
    // Pass the error to the python through pybind11 allowing graceful exit
    throw std::invalid_argument("Cannot load: " + sceneFilename);
  }

  // create agents, sensors
  agents_.clear();
  // TODO: in the future, it should create multiple agents
  auto& agentParentNode = sceneGraph.getRootNode();
  addAgent(agentConfig, agentParentNode);
  ASSERT(0 <= cfg.defaultAgentId && cfg.defaultAgentId < agents_.size());

  // create pathfinder and load navmesh if available
  if (pathfinder_) {
    pathfinder_.reset();
  }
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

  // load semantic annotations if available
  if (semanticScene_) {
    semanticScene_.reset();
  }
  semanticScene_ = scene::SemanticScene::create();
  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (cfg.scene.filepaths.count("house")) {
    houseFilename = cfg.scene.filepaths.at("house");
  }
  if (io::exists(houseFilename)) {
    LOG(INFO) << "Loading house from " << houseFilename;
    scene::SemanticScene::loadMp3dHouse(houseFilename, *semanticScene_);
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
      resourceManager_.loadScene(semanticSceneInfo, &semanticRootNode,
                                 &semanticDrawables);
    }
    LOG(INFO) << "Loaded.";
  }
  // frl instance meshes and suncg houses contain their semantic annotations
  if (sceneInfo.type == assets::AssetType::FRL_INSTANCE_MESH ||
      sceneInfo.type == assets::AssetType::SUNCG_SCENE) {
    activeSemanticSceneID_ = activeSceneID_;
  }
  // also load SemanticScene for SUNCG house file
  if (sceneInfo.type == assets::AssetType::SUNCG_SCENE) {
    scene::SemanticScene::loadSuncgHouse(sceneFilename, *semanticScene_);
  }

  // connect controls to navmesh if loaded
  if (pathfinder_->isLoaded()) {
    agents_[cfg.defaultAgentId]->getControls()->setMoveFilterFunction(
        [&](const vec3f& start, const vec3f& end) {
          return pathfinder_->tryStep(start, end);
        });
  }

  // now reset to sample agent state
  reset();
}

void Simulator::reset() {
  for (int iAgent = 0; iAgent < agents_.size(); ++iAgent) {
    auto& agent = agents_[iAgent];
    agent::AgentState::ptr state = agent::AgentState::create();
    sampleRandomAgentState(state);
    agent->setState(*state);
    LOG(INFO) << "Reset agent i=" << iAgent
              << " position=" << state->position.transpose()
              << " rotation=" << state->rotation.transpose();
  }
}

void Simulator::seed(uint32_t newSeed) {
  random_.seed(newSeed);
  pathfinder_->seed(newSeed);
}

std::shared_ptr<Renderer> Simulator::getRenderer() {
  return renderer_;
}

std::shared_ptr<scene::SemanticScene> Simulator::getSemanticScene() {
  return semanticScene_;
}

std::shared_ptr<nav::PathFinder> Simulator::getPathFinder() {
  return pathfinder_;
}

std::shared_ptr<agent::Agent> Simulator::getAgent(int agentId) {
  ASSERT(0 <= agentId && agentId < agents_.size());
  return agents_[agentId];
}

scene::SceneGraph& Simulator::getActiveSceneGraph() {
  ASSERT(0 <= activeSceneID_ && activeSceneID_ < sceneID_.size());
  return sceneManager_.getSceneGraph(activeSceneID_);
}

//! return the semantic scene's SceneGraph for rendering
scene::SceneGraph& Simulator::getActiveSemanticSceneGraph() {
  ASSERT(0 <= activeSemanticSceneID_ &&
         activeSemanticSceneID_ < sceneID_.size());
  return sceneManager_.getSceneGraph(activeSemanticSceneID_);
}

std::shared_ptr<nav::ActionSpacePathFinder> Simulator::makeActionPathfinder(
    int agentId) {
  auto agent = getAgent(agentId);

  agent::AgentState::ptr state = agent::AgentState::create();
  agent->getState(state);
  return std::make_shared<nav::ActionSpacePathFinder>(
      pathfinder_, agent->getConfig(), *agent->getControls(), state->rotation);
}

void Simulator::saveFrame(const std::string& filename) {
  LOG(INFO) << "Saving frame to " << filename;
  const auto& agent = agents_[config_.defaultAgentId];

  ASSERT(0 <= activeSceneID_ && activeSceneID_ < sceneID_.size());
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_[activeSceneID_]);

  // a smart pointer
  auto pinholeCamera = agent->getSensorSuite().get(config_.defaultCameraUuid);

  // render all the drawables in the scene graph by the sensor
  renderer_->draw(*pinholeCamera.get(), sceneGraph);

  const vec3i dims = renderer_->getSize();
  std::vector<uint8_t> frameBytes(dims[0] * dims[1] * dims[2]);
  renderer_->readFrameRgba(frameBytes.data());

  Containers::ArrayView<uint8_t> view{frameBytes.data(), frameBytes.size()};
  Magnum::ImageView2D img{PixelFormat::RGBA8Unorm, {dims[0], dims[1]}, view};

  PluginManager::Manager<Trade::AbstractImageConverter> converterManager;
  Corrade::Containers::Pointer<Magnum::Trade::AbstractImageConverter>
      converter_ = converterManager.loadAndInstantiate("StbPngImageConverter");
  converter_->exportToFile(img, filename);

  LOG(INFO) << "Saved frame.";
}

bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.scene == b.scene && esp::equal(a.agents, b.agents) &&
         a.defaultAgentId == b.defaultAgentId &&
         a.defaultCameraUuid == b.defaultCameraUuid &&
         a.compressTextures == b.compressTextures;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

}  // namespace gfx
}  // namespace esp
