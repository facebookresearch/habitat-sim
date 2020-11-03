// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SimulatorConfiguration.h"

namespace esp {
namespace sim {
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return a.scene == b.scene && a.defaultAgentId == b.defaultAgentId &&
         a.defaultCameraUuid == b.defaultCameraUuid &&
         a.compressTextures == b.compressTextures &&
         a.createRenderer == b.createRenderer &&
         a.enablePhysics == b.enablePhysics &&
         a.physicsConfigFile.compare(b.physicsConfigFile) == 0 &&
         a.loadSemanticMesh == b.loadSemanticMesh &&
         a.sceneLightSetup.compare(b.sceneLightSetup) == 0;
}

bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b) {
  return !(a == b);
}

}  // namespace sim
}  // namespace esp
