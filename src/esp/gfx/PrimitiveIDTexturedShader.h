// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <vector>

#include <Corrade/Containers/EnumSet.h>

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class PrimitiveIDTexturedShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /**
   * @brief Constructor
   */
  explicit PrimitiveIDTexturedShader();

  //! @brief vertex positions
  typedef Magnum::GL::Attribute<0, Magnum::Vector4> Position;
  //! @brief vertex colors
  typedef Magnum::GL::Attribute<3, Magnum::Vector3> Color;

  //! Color attachment location per output type
  enum : uint8_t {
    //! color output
    ColorOutput = 0,
    //! object id output
    ObjectIdOutput = 1
  };

  /**
   * @brief Set transformation and projection matrix
   * @return Reference to self (for method chaining)
   */
  PrimitiveIDTexturedShader& setTransformationProjectionMatrix(
      const Magnum::Matrix4& matrix) {
    setUniform(uniformLocation("transformationProjectionMatrix"), matrix);
    return *this;
  }

  /**
   * @brief Bind a color texture
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::Textured enabled.
   * @see @ref setColor()
   */
  PrimitiveIDTexturedShader& bindTexture(Magnum::GL::Texture2D& texture);
};

}  // namespace gfx
}  // namespace esp
