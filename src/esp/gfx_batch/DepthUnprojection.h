// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DEPTHUNPROJECTION_H_
#define ESP_GFX_DEPTHUNPROJECTION_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>

namespace esp {
namespace gfx_batch {

/**
@brief Depth-only shader

Outputs depth values without projection applied. Can also unproject existing
depth buffer if @ref Flag::UnprojectExistingDepth is enabled.
@see @ref calculateDepthUnprojection(), @ref unprojectDepth()
*/
class DepthShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /** @brief Flag */
  enum class Flag {
    /**
     * Perform unprojection of an existing depth buffer. If enabled, instead
     * of rendering particular polygons bind a depth texture with
     * @ref bindDepthTexture(). The shader will then render a full-screen
     * triangle and unprojects the depth using an inverse of the matrix
     * passed in @ref setProjectionMatrix().
     */
    UnprojectExistingDepth = 1 << 0,

    /**
     * By default, depth values on the far plane are patched to have a value of
     * @cpp 0.0f @ce (instead of @cpp 100.0f @ce or whatever the far plane is
     * set to). This might have some performance penalty and can be turned off
     * with this flag.
     */
    NoFarPlanePatching = 1 << 1
  };

  /** @brief Flags */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /** @brief Constructor */
  explicit DepthShader(Flags flags = {});

  /**
   * @brief Set transformation and projection matrix
   * @return Reference to self (for method chaining)
   *
   * Expects that @ref Flag::UnprojectExistingDepth is not set.
   */
  DepthShader& setTransformationMatrix(const Magnum::Matrix4& matrix);

  /**
   * @brief Set the depth unprojection parameters directly
   * @return Reference to self (for method chaining)
   */
  DepthShader& setDepthUnprojection(const Magnum::Vector2& depthUnprojection);

  /**
   * @brief Set projection matrix for unprojection
   * @return Reference to self (for method chaining)
   */
  DepthShader& setProjectionMatrix(const Magnum::Matrix4& matrix);

  /**
   * @brief Bind depth texture
   * @return Reference to self (for method chaining)
   *
   * Expects that @ref Flag::UnprojectExistingDepth is set.
   */
  DepthShader& bindDepthTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief The flags passed to the Constructor
   */
  Flags flags() const { return flags_; }

 private:
  const Flags flags_;
  int transformationMatrixUniform_, projectionMatrixOrDepthUnprojectionUniform_;
};

CORRADE_ENUMSET_OPERATORS(DepthShader::Flags)

/**
@brief Calculate depth unprojection coefficients for @ref unprojectDepth()

Given a projection transformation of vector @f$ \boldsymbol{v} @f$ using a
matrix @f$ \boldsymbol{P} @f$ where @f$ x @f$ and @f$ y @f$ are arbitrary, the
transformation can be reduced to a 2x2 matrix multiplication: @f[
  \begin{array}{rcl}
    \boldsymbol{P} \boldsymbol{v} & = & \begin{pmatrix}
      p & 0 & 0 & 0 \\
      0 & q & 0 & 0 \\
      0 & 0 & a & b \\
      0 & 0 & -1 & 0
    \end{pmatrix} \begin{pmatrix}
      x \\ y \\ z \\ 1
    \end{pmatrix} \\[2.5em]

    \boldsymbol{P'} \boldsymbol{v} & = & \begin{pmatrix}
      a & b \\
      -1 & 0
    \end{pmatrix} \begin{pmatrix}
      z \\ 1
    \end{pmatrix} = \begin{pmatrix}
      az + b \\
      -z
    \end{pmatrix}
  \end{array}
@f]

In the OpenGL depth buffer, the depth values @f$ d @f$ are in range
@f$ [ 0 ; 1 ] @f$ and the final depth value is after a perspective divide. The
output has Z going forward, not backward, so we're looking for the value of
@f$ -z @f$: @f[
  \begin{array}{rcl}
    2d - 1 & = & \cfrac{az + b}{-z} \\[1.25em]
    -z & = & \cfrac{b}{2d + a - 1}
  \end{array}
@f]

Finally, to reduce the amount of operations in @ref unprojectDepth(), we
integrate the constants into @f$ a @f$ and @f$ b @f$, returning @f$ a' @f$ and
@f$ b' @f$: @f[
  \begin{array}{rcl}
    -z & = & \cfrac{b}{2d + (a - 1)} \\[1.25em]
    -z & = & \cfrac{\frac{1}{2}b}{d + \frac{1}{2}(a - 1)} \\[1.25em]
    -z & = & \cfrac{b'}{d + a'} ; ~~~~ a' = \frac{1}{2}(a - 1),
                                  ~~~~ b' = \frac{1}{2}b \end{array}
@f]
*/
Magnum::Vector2 calculateDepthUnprojection(
    const Magnum::Matrix4& projectionMatrix);

/**
@brief Unproject depth values
@param[in] unprojection Unprojection coefficients from
    @ref calculateDepthUnprojection()
@param[in,out] depth    Depth values in range @f$ [ 0 ; 1 ] @f$

See @ref calculateDepthUnprojection() for the full algorithm explanation.
Additionally to applying that calculation, if the input depth is at the far
plane (of value @cpp 1.0f @ce), it's set to @cpp 0.0f @ce on output as
consumers expect zeros for things that are too far.
*/
void unprojectDepth(
    const Magnum::Vector2& unprojection,
    const Corrade::Containers::StridedArrayView2D<Magnum::Float>& depth);

}  // namespace gfx_batch
}  // namespace esp

#endif  // ESP_GFX_DEPTHUNPROJECTION_H_
