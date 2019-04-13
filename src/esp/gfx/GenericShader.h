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

class GenericShader : public Magnum::GL::AbstractShaderProgram {
 public:
  //! Configuration flag
  enum class Flag : uint8_t {
    //! Multiply color with a texture
    Textured = 1 << 0,
    //! Use per-vertex colors
    VertexColored = 1 << 1,
    //! Use per-vertex ids encoded in vertex position[3]
    PerVertexIds = 1 << 2,
    //! Indexes a texture with the primitive id
    PrimitiveIDTextured = 1 << 3,
  };

  //! Set of configuration flags
  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  /**
   * @brief Constructor
   * @param flags Flags
   */
  explicit GenericShader(Flags flags = {});

  //! @brief vertex positions
  typedef Magnum::GL::Attribute<0, Magnum::Vector4> Position;
  //! @brief texture coordinates
  typedef Magnum::GL::Attribute<1, Magnum::Vector2> TextureCoordinates;
  //! @brief vertex normals
  typedef Magnum::GL::Attribute<2, Magnum::Vector3> Normal;
  //! @brief vertex colors
  typedef Magnum::GL::Attribute<3, Magnum::Vector3> Color;

  //! Color attachment location per output type
  enum : uint8_t {
    //! color output
    ColorOutput = 0,
    //! depth frame output
    DepthOutput = 1,
    //! object id output
    ObjectIdOutput = 2
  };

  /**
   * @return Configuration flags
   */
  Flags flags() const { return flags_; }

  /**
   * @brief Set transformation and projection matrix
   * @return Reference to self (for method chaining)
   */
  GenericShader& setTransformationProjectionMatrix(
      const Magnum::Matrix4& matrix) {
    setUniform(uniformLocation("transformationProjectionMatrix"), matrix);
    return *this;
  }

  /**
   * @brief Set projection matrix
   * @return Reference to self (for method chaining)
   */
  GenericShader& setProjectionMatrix(const Magnum::Matrix4& matrix) {
    setUniform(uniformLocation("projectionMatrix"), matrix);
    return *this;
  }

  /**
   * @brief Set color
   * @return Reference to self (for method chaining)
   *
   * If @ref Flag::Textured is set, initial value is @cpp 0xffffffff_rgbaf @ce
   * and the color will be multiplied with texture.
   * @see @ref bindTexture()
   */
  GenericShader& setColor(const Magnum::Color4& color) {
    setUniform(uniformLocation("colorUniform"), color);
    return *this;
  }

  /**
   * @brief Set object id
   * @return Reference to self (for method chaining)
   */
  GenericShader& setObjectId(int objectId) {
    setUniform(uniformLocation("objectIdUniform"), objectId);
    return *this;
  }

  /**
   * @brief Bind a color texture
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::Textured enabled.
   * @see @ref setColor()
   */
  GenericShader& bindTexture(Magnum::GL::Texture2D& texture);

 protected:
  Flags flags_;
};

CORRADE_ENUMSET_OPERATORS(GenericShader::Flags)

}  // namespace gfx
}  // namespace esp
