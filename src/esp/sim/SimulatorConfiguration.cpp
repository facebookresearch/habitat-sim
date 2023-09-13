// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SimulatorConfiguration.h"

namespace esp {
namespace sim {
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.activeSceneName == b.activeSceneName &&
         a.defaultAgentId == b.defaultAgentId &&
         a.gpuDeviceId == b.gpuDeviceId && a.randomSeed == b.randomSeed &&
         a.createRenderer == b.createRenderer &&
         a.allowSliding == b.allowSliding &&
         a.frustumCulling == b.frustumCulling &&
         a.enablePhysics == b.enablePhysics &&
         a.enableGfxReplaySave == b.enableGfxReplaySave &&
         a.loadSemanticMesh == b.loadSemanticMesh &&
         a.forceSeparateSemanticSceneGraph ==
             b.forceSeparateSemanticSceneGraph &&
         a.requiresTextures == b.requiresTextures &&
         a.leaveContextWithBackgroundRenderer ==
             b.leaveContextWithBackgroundRenderer &&
         a.useSemanticTexturesIfFound == b.useSemanticTexturesIfFound &&
         a.sceneDatasetConfigFile == b.sceneDatasetConfigFile &&
         a.physicsConfigFile == b.physicsConfigFile &&
         a.overrideSceneLightDefaults == b.overrideSceneLightDefaults &&
         a.sceneLightSetupKey == b.sceneLightSetupKey &&
         a.enableHBAO == b.enableHBAO && a.navMeshSettings == b.navMeshSettings;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

}  // namespace sim
}  // namespace esp
