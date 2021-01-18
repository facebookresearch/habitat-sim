// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_SIMULATORCONFIGURATION_H_
#define ESP_SIM_SIMULATORCONFIGURATION_H_

#include <string>

#include "esp/core/esp.h"
#include "esp/physics/configure.h"

namespace esp {

namespace sim {
struct SimulatorConfiguration {
  /**
   * @brief Name of scene or stage config or asset to load
   */
  std::string activeSceneName;
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
   * @brief todo
   */
  bool enableGfxReplaySave = false;
  /**
   * @brief Whether or not to load the semantic mesh
   */
  bool loadSemanticMesh = true;
  /**
   * Force creation of a separate semantic scene graph, even when no semantic
   * mesh is loaded for the stage. Required to support playback of any replay
   * that includes a semantic-only render asset instance. Set to false
   * otherwise.
   */
  bool forceSeparateSemanticSceneGraph = false;
  /**
   * @brief Whether or not to load textures for the meshes. This MUST be true
   * for RGB rendering
   */
  bool requiresTextures = true;
  std::string physicsConfigFile = ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH;

  /**
   * @brief File location for initial scene dataset to use.
   */
  std::string sceneDatasetConfigFile = "default";

  /**
   * @brief allows for overriding any scene lighting setup specified in a scene
   * instance file with the value specified below.
   */
  bool overrideSceneLightDefaults = false;

  /** @brief Light setup key for scene */
  std::string sceneLightSetup = esp::NO_LIGHT_KEY;

  ESP_SMART_POINTERS(SimulatorConfiguration)
};
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

}  // namespace sim
}  // namespace esp

#endif  // ESP_SIM_SIMULATORCONFIGURATION_H_
