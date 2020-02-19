// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/ResourceManager.h>

#include "esp/gfx/LightSetup.h"
#include "esp/gfx/MaterialData.h"

namespace esp {
namespace gfx {

using ShaderManager = Magnum::ResourceManager<Magnum::GL::AbstractShaderProgram,
                                              gfx::LightSetup,
                                              gfx::MaterialData>;

}  // namespace gfx
}  // namespace esp
