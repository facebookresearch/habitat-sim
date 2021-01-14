// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SimulatorConfiguration.h"

namespace esp {
namespace sim {
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.activeSceneName.compare(b.activeSceneName) == 0 &&
         a.defaultAgentId == b.defaultAgentId &&
         a.gpuDeviceId == b.gpuDeviceId && a.randomSeed == b.randomSeed &&
         a.defaultCameraUuid.compare(b.defaultCameraUuid) == 0 &&
         a.compressTextures == b.compressTextures &&
         a.createRenderer == b.createRenderer &&
         a.allowSliding == b.allowSliding &&
         a.frustumCulling == b.frustumCulling &&
         a.enablePhysics == b.enablePhysics &&
         a.enableGfxReplaySave == b.enableGfxReplaySave &&
         a.loadSemanticMesh == b.loadSemanticMesh &&
         a.forceSeparateSemanticSceneGraph ==
             b.forceSeparateSemanticSceneGraph &&
         a.requiresTextures == b.requiresTextures &&
         a.sceneDatasetConfigFile.compare(b.sceneDatasetConfigFile) == 0 &&
         a.physicsConfigFile.compare(b.physicsConfigFile) == 0 &&
         a.overrideSceneLightDefaults == b.overrideSceneLightDefaults &&
         a.sceneLightSetup.compare(b.sceneLightSetup) == 0;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

}  // namespace sim
}  // namespace esp
