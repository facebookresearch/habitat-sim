// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_SHADERMANAGER_H_
#define ESP_GFX_SHADERMANAGER_H_

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/ResourceManager.h>
#include <Magnum/Trade/MaterialData.h>

#include "esp/gfx/LightSetup.h"

namespace esp {
namespace gfx {

using ShaderManager = Magnum::ResourceManager<Magnum::GL::AbstractShaderProgram,
                                              gfx::LightSetup,
                                              Magnum::Trade::MaterialData>;

/**
 * @brief Set the light setup for a subtree
 *
 * All drawables in the subtree starting at root will have the new lightSetup
 *
 * @param root Subtree root
 * @param lightSetup @ref LightSetup key in the ShaderManager
 */
void setLightSetupForSubTree(scene::SceneNode& root,
                             const Magnum::ResourceKey& lightSetup);

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_SHADERMANAGER_H_
