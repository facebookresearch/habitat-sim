// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_SIMULATORCONFIGURATION_H_
#define ESP_SIM_SIMULATORCONFIGURATION_H_

#include <string>

#include "esp/core/Esp.h"
#include "esp/gfx/configure.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/configure.h"

namespace Cr = Corrade;

namespace esp {

namespace sim {
struct SimulatorConfiguration {
  //! Name of scene or stage config or asset to load
  std::string activeSceneName;
  //! The default agent id used during initialization and functionally whenever
  //! alternative agent ids are not provided.
  int defaultAgentId = 0;
  //! The system GPU device to use for rendering.
  int gpuDeviceId = 0;
  //! The Simulator and Pathfinder random seed. Set during scene initialization.
  unsigned int randomSeed = 0;
  //! Optimisation for non-visual simulation. If false, no renderer will be
  //! created and no materials or textures loaded.
  bool createRenderer = true;
  //! Whether or not the agent can slide on NavMesh collisions.
  bool allowSliding = true;
  //! Enable or disable the frustum culling optimisation
  bool frustumCulling = true;
  /**
   * @brief This flags specifies whether or not dynamics is supported by the
   * simulation, if a suitable library (i.e. Bullet) has been installed.
   */
  bool enablePhysics = false;
  /**
   * @brief Enable the recording of render keyframes during simulation.
   * These keyframes can be used later to replay the graphics of a simulation.
   */
  bool enableGfxReplaySave = false;
  /**
   * @brief Whether or not to load the semantic mesh
   */
  bool loadSemanticMesh = true;
  /**
   * @brief Force creation of a separate semantic scene graph, even when no
   * semantic mesh is loaded for the stage. Required to support playback of any
   * replay that includes a semantic-only render asset instance. Set to false
   * otherwise.
   */
  bool forceSeparateSemanticSceneGraph = false;
  /**
   * @brief Whether or not to load textures for the meshes. This MUST be true
   * for RGB rendering
   */
  bool requiresTextures = true;

  /**
   * @brief Leave the context with the background thread after finishing draw
   * jobs. This will improve performance as transferring the OpenGL context back
   * and forth takes time but will require the user to manually transfer the
   * context back to the main thread before adding or removing objects.
   */
  bool leaveContextWithBackgroundRenderer = false;

  //! Path to the physics parameter config file.
  std::string physicsConfigFile = ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH;

  /**
   * @brief File location for initial scene dataset to use.
   */
  std::string sceneDatasetConfigFile = "default";

  /**
   * @brief Allows for overriding any scene lighting setup specified in a scene
   * instance file with the value specified below.
   */
  bool overrideSceneLightDefaults = false;

  /** @brief Light setup key for scene */
  std::string sceneLightSetupKey = esp::NO_LIGHT_KEY;

  /**
   * @brief Setup the image based lighting for pbr rendering. @deprecated use
   * configs to enable/disable IBL.
   */
  bool pbrImageBasedLighting = false;

  /**
   * @brief Use texture-based semantics if the specified asset/dataset support
   * them.
   */
  bool useSemanticTexturesIfFound = true;

  /**
   * @brief Optionally provide a pre-configured NavMeshSettings. If provided,
   * the NavMesh will be recomputed with the provided settings if A. no NavMesh
   * was loaded, or B. the loaded NavMesh's settings differ from the configured
   * settings. If not provided, no NavMesh recompute will be done automatically.
   */
  nav::NavMeshSettings::ptr navMeshSettings = nullptr;

  ESP_SMART_POINTERS(SimulatorConfiguration)
};
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

}  // namespace sim
}  // namespace esp

#endif  // ESP_SIM_SIMULATORCONFIGURATION_H_
