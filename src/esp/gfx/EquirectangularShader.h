// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_EQUIRECTANGULARSHADER_H_
#define ESP_GFX_EQUIRECTANGULARSHADER_H_

#include "CubeMapShaderBase.h"

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Macros.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include "Magnum/GL/Mesh.h"
#include "esp/core/Esp.h"
#include "esp/gfx/CubeMap.h"

namespace esp {
namespace gfx {
class EquirectangularShader : public CubeMapShaderBase {
 public:
  /**
   * @brief constructor
   * @param[in] flags equirectangular shader flags
   */
  explicit EquirectangularShader(Flags flags = {Flag::ColorTexture});

  ~EquirectangularShader() override = default;

  /**
   * @brief Set ViewportSize for calculations in vertex shader
   * @param[in] viewportSize the size of the viewport
   * @return itself for method chaining
   */
  EquirectangularShader& setViewportSize(esp::vec2i viewportSize);

 protected:
  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  int viewportHeightUniform_ = ID_UNDEFINED;
  int viewportWidthUniform_ = ID_UNDEFINED;
};
}  // namespace gfx
}  // namespace esp
#endif
