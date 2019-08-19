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

  enum : int {
    /**
     * Hardcoded width of the primitive ID texture, since we wouldn't be able
     * to fit it all into a 1D texture. Picking a power-of-two size as that
     * nicely fits caches etc. Conservatively choosing 4K as 8K might not be
     * supported on some platforms (mobile/WebGL, possibly). A 4096x4096
     * texture fits 16M primitives, which should be enough. Smaller meshes will
     * have the height much smaller while larger meshes are of course allowed
     * to go beyond that -- e.g. 4096x6000 is not expected to be a problem.
     */
    PrimitiveIDTextureWidth = 4096
  };

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
    setUniform(transformationProjectionMatrixUniform_, matrix);
    return *this;
  }

  PrimitiveIDTexturedShader& bindTexture(Magnum::GL::Texture2D& texture);

 private:
  int transformationProjectionMatrixUniform_;
};

}  // namespace gfx
}  // namespace esp
