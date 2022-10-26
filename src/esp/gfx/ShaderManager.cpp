// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ShaderManager.h"

#include "esp/core/Esp.h"
#include "esp/gfx/Drawable.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

void setLightSetupForSubTree(scene::SceneNode& root,
                             const Magnum::ResourceKey& lightSetup) {
  scene::preOrderFeatureTraversalWithCallback<Drawable>(
      root, [&lightSetup](Drawable& drawable) {
        drawable.setLightSetup(lightSetup);
      });
}

}  // namespace gfx
}  // namespace esp
