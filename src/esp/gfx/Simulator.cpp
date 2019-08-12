// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Simulator.h"

#include <string>

#include <Corrade/Containers/Pointer.h>
#include <Corrade/Utility/String.h>

#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/AbstractImageConverter.h>

#include "Drawable.h"

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/PinholeCamera.h"

using namespace Magnum;
using namespace Math::Literals;
using namespace Corrade;

namespace esp {
namespace gfx {

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

  const int height = cfg.height;
  const int width = cfg.width;

  // load scene
  std::string sceneFilename = cfg.scene.id;
  if (cfg.scene.filepaths.count("mesh")) {
    sceneFilename = cfg.scene.filepaths.at("mesh");
  }

  std::string houseFilename = io::changeExtension(sceneFilename, ".house");
  if (cfg.scene.filepaths.count("house")) {
    houseFilename = cfg.scene.filepaths.at("house");
  }

  const assets::AssetInfo sceneInfo =
      assets::AssetInfo::fromPath(sceneFilename);

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
      context_ = std::make_unique<gfx::WindowlessContext>(config_.gpuDeviceId);
    }

    // reinitalize members
    renderer_ = nullptr;
    renderer_ = Renderer::create(width, height);

    auto& sceneGraph = sceneManager_.getSceneGraph(activeSceneID_);

    auto& rootNode = sceneGraph.getRootNode();
    auto& drawables = sceneGraph.getDrawables();
    resourceManager_.compressTextures(cfg.compressTextures);

    bool loadSuccess = false;
    if (config_.enablePhysics) {
      loadSuccess = resourceManager_.loadScene(sceneInfo, physicsManager_,
                                               &rootNode, &drawables);
    } else {
      loadSuccess =
          resourceManager_.loadScene(sceneInfo, &rootNode, &drawables);
    }
    if (!loadSuccess) {
      LOG(ERROR) << "cannot load " << sceneFilename;
      // Pass the error to the python through pybind11 allowing graceful exit
      throw std::invalid_argument("Cannot load: " + sceneFilename);
    }

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
        resourceManager_.loadScene(semanticSceneInfo, &semanticRootNode,
                                   &semanticDrawables);
      }
      LOG(INFO) << "Loaded.";
    }

    // instance meshes and suncg houses contain their semantic annotations
    if (sceneInfo.type == assets::AssetType::FRL_INSTANCE_MESH ||
        sceneInfo.type == assets::AssetType::SUNCG_SCENE ||
        sceneInfo.type == assets::AssetType::INSTANCE_MESH) {
      activeSemanticSceneID_ = activeSceneID_;
    }
  }

  semanticScene_ = nullptr;
  semanticScene_ = scene::SemanticScene::create();
  if (io::exists(houseFilename)) {
    scene::SemanticScene::loadMp3dHouse(houseFilename, *semanticScene_);
  }

  // also load SemanticScene for SUNCG house file
  if (sceneInfo.type == assets::AssetType::SUNCG_SCENE) {
    scene::SemanticScene::loadSuncgHouse(sceneFilename, *semanticScene_);
  }

  // now reset to sample agent state
  reset();
}

void Simulator::reset() {
  if (physicsManager_ != nullptr)
    physicsManager_.reset();  // TODO: does nothing yet
}

void Simulator::seed(uint32_t newSeed) {
  random_.seed(newSeed);
}

std::shared_ptr<Renderer> Simulator::getRenderer() {
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

}  // namespace gfx
}  // namespace esp
