// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "WindowlessContext.h"
#include "esp/core/esp.h"
#include "esp/core/random.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"

#include "esp/assets/ResourceManager.h"

namespace esp {
namespace nav {
class PathFinder;
class ActionSpacePathFinder;
}  // namespace nav
namespace scene {
class SemanticScene;
}  // namespace scene
namespace gfx {

// forward declarations
class Renderer;

struct SimulatorConfiguration {
  scene::SceneConfiguration scene;
  int defaultAgentId = 0;
  int gpuDeviceId = 0;
  std::string defaultCameraUuid = "rgba_camera";
  bool compressTextures = false;
  int width = 256, height = 256;

  ESP_SMART_POINTERS(SimulatorConfiguration)
};
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);
bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

class Simulator {
 public:
  explicit Simulator(const SimulatorConfiguration& cfg);
  virtual ~Simulator();

  void reconfigure(const SimulatorConfiguration& cfg);

  void reset();

  void seed(uint32_t newSeed);

  std::shared_ptr<Renderer> getRenderer();
  std::shared_ptr<scene::SemanticScene> getSemanticScene();

  scene::SceneGraph& getActiveSceneGraph();
  scene::SceneGraph& getActiveSemanticSceneGraph();

  void saveFrame(const std::string& filename);

 protected:
  WindowlessContext context_;
  std::shared_ptr<Renderer> renderer_ = nullptr;
  // CANNOT make the specification of resourceManager_ above the context_!
  // Because when deconstructing the resourceManager_, it needs
  // the GL::Context
  // If you switch the order, you will have the error:
  // GL::Context::current(): no current context from Magnum
  // during the deconstruction
  assets::ResourceManager resourceManager_;
  scene::SceneManager sceneManager_;
  int activeSceneID_ = ID_UNDEFINED;
  int activeSemanticSceneID_ = ID_UNDEFINED;
  std::vector<int> sceneID_;

  std::shared_ptr<scene::SemanticScene> semanticScene_ = nullptr;

  core::Random random_;
  SimulatorConfiguration config_;

  ESP_SMART_POINTERS(Simulator)
};

}  // namespace gfx
}  // namespace esp
