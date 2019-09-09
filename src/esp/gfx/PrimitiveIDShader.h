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
#include <Magnum/Shaders/Generic.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class PrimitiveIDShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /**
   * @brief Constructor
   */
  explicit PrimitiveIDShader();

  //! @brief vertex positions
  typedef Magnum::Shaders::Generic3D::Position Position;
  //! @brief vertex colors
  typedef Magnum::Shaders::Generic3D::Color3 Color3;
  //! @brief object ids
  typedef Magnum::GL::Attribute<5, Magnum::UnsignedInt> ObjectId;

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
  PrimitiveIDShader& setTransformationProjectionMatrix(
      const Magnum::Matrix4& matrix) {
    setUniform(transformationProjectionMatrixUniform_, matrix);
    return *this;
  }

 private:
  int transformationProjectionMatrixUniform_;
};

}  // namespace gfx
}  // namespace esp
