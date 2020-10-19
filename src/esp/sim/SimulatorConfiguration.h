// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_SIMULATORCONFIGURATION_H_
#define ESP_SIM_SIMULATORCONFIGURATION_H_

#include "esp/assets/ResourceManager.h"
#include "esp/physics/configure.h"
#include "esp/scene/SceneConfiguration.h"

namespace esp {
namespace sim {

struct SimulatorConfiguration {
  scene::SceneConfiguration scene;
  int defaultAgentId = 0;
  int gpuDeviceId = 0;
  unsigned int randomSeed = 0;
  std::string defaultCameraUuid = "rgba_camera";
  bool compressTextures = false;
  bool createRenderer = true;
  // Whether or not the agent can slide on collisions
  bool allowSliding = true;
  // enable or disable the frustum culling
  bool frustumCulling = true;
  /**
   * @brief This flags specifies whether or not dynamics is supported by the
   * simulation, if a suitable library (i.e. Bullet) has been installed.
   */
  bool enablePhysics = false;
  /**
   * @brief Whether or not to load the semantic mesh
   */
  bool loadSemanticMesh = true;
  /**
   * @brief Whether or not to load textures for the meshes. This MUST be true
   * for RGB rendering
   */
  bool requiresTextures = true;
  std::string physicsConfigFile =
      ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH;  // should we instead link a
                                            // PhysicsManagerConfiguration
                                            // object here?
  /** @brief Light setup key for scene */
  std::string sceneLightSetup = assets::ResourceManager::NO_LIGHT_KEY;

  ESP_SMART_POINTERS(SimulatorConfiguration)
};
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

}  // namespace sim
}  // namespace esp

#endif  // ESP_SIM_SIMULATORCONFIGURATION_H_
