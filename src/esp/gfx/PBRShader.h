// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBRSHADER_H_
#define ESP_GFX_PBRSHADER_H_

#include <memory>
#include <vector>

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Matrix4.h>
#include "esp/core/esp.h"

namespace esp {

namespace gfx {

class PBRShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /**
   * @brief vertex positions
   */
  typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;

  /**
   * @brief normal direction
   */
  typedef Magnum::GL::Attribute<1, Magnum::Vector3> Normal;

  /**
   * @brief 2D texture coordinates
   *
   * Used only if at least one of
   * @ref Flag::AlbedoTexture, @ref Flag::NormalTexture and
   * @ref Flag::RoughnessTexture @ref Flag::MetallicTexture is set.
   */
  typedef Magnum::GL::Attribute<2, Magnum::Vector2> TextureCoordinates;

  /**
   * @brief Tangent direction with the fourth component indicating the handness.
   *
   * T = Tangent, B = BiTangent, N = Normal
   *
   * 1.0 means T, B, N form a right-handed coordinate;
   * -1.0 means T, B, N form a left-handed coordinate;
   *
   * Used only if @ref Flag::NormalTexture is set.
   */
  typedef Magnum::GL::Attribute<3, Magnum::Vector4> Tangent4;

  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = 0,

#ifndef MAGNUM_TARGET_GLES2
    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     * @requires_gl30 Extension @gl_extension{EXT,texture_integer}
     * @requires_gles30 Object ID output requires integer support in
     *      shaders, which is not available in OpenGL ES 2.0 or WebGL
     *      1.0.
     */
    ObjectIdOutput = 1,
#endif
  };

  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * Multiply ambient color with a texture.
     * @see @ref setAmbientColor(), @ref bindAmbientTexture()
     */
    AlbedoTexture = 1 << 0,

    /**
     * @see @ref setRoughness(), @ref bindRoughnessTexture()
     */
    RoughnessTexture = 1 << 1,

    /**
     * @see @ref setMetallic(), @ref bindMetallicTexture()
     */
    MetallicTexture = 1 << 2,

    /**
     * Modify normals according to a texture. Requires the
     * @ref Tangent4 attribute to be present.
     */
    NormalTexture = 1 << 3,

    /**
     * TODO: Do we need VertexColor?
     * Multiply diffuse color with a vertex color. Requires either
     * the @ref Color3 or @ref Color4 attribute to be present.
     */

    /**
     * Enable object ID output.
     * @requires_gl30 Extension @gl_extension{EXT,gpu_shader4}
     * @requires_gles30 Object ID output requires integer support in
     *      shaders, which is not available in OpenGL ES 2.0 or WebGL
     *      1.0.
     */
    ObjectId = 1 << 5,
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief Constructor
   * @param flags         Flags
   * @param lightCount    Count of light sources
   */
  explicit PBRShader(Flags flags = {}, Magnum::UnsignedInt lightCount = 1);

  /** @brief Copying is not allowed */
  PBRShader(const PBRShader&) = delete;

  /** @brief Move constructor */
  PBRShader(PBRShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PBRShader& operator=(const PBRShader&) = delete;

  /** @brief Move assignment */
  PBRShader& operator=(PBRShader&&) noexcept = default;

  /** @brief Flags */
  Flags flags() const { return flags_; }

  // ======== texture binding ========
  /**
   * @brief Bind the albedo texture
   * @return Reference to self (for method chaining)
   */
  PBRShader& bindAlbedoTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the roughness texture
   * @return Reference to self (for method chaining)
   */
  PBRShader& bindRoughnessTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the metallic texture
   * @return Reference to self (for method chaining)
   */
  PBRShader& bindMetallicTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the normal texture
   * @return Reference to self (for method chaining)
   */
  PBRShader& bindNormalTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the albedo, roughness, metallic, normal textures
   * @return Reference to self (for method chaining)
   */
  PBRShader& bindTextures(Magnum::GL::Texture2D* albedo,
                          Magnum::GL::Texture2D* roughness,
                          Magnum::GL::Texture2D* metallic,
                          Magnum::GL::Texture2D* normal = nullptr);

  // ======== set uniforms ===========
  /**
   *  @brief Set modelview and projection matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PBRShader& setMVPMatrix(const Magnum::Matrix4& matrix);

  // -------- materials ---------------
  /**
   *  @brief Set base color to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PBRShader& setBaseColor(const Magnum::Color4& color);
  /**
   *  @brief Set roughness to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PBRShader& setRoughness(float roughness);
  /**
   *  @brief Set metallic to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PBRShader& setMetallic(float metallic);

  /**
   *  @brief Set object id to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PBRShader& setObjectId(unsigned int objectId);

 protected:
  Flags flags_;
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int modelviewMatrixUniform_ = ID_UNDEFINED;
  int normalMatrixUniform_ = ID_UNDEFINED;
  int mvpMatrixUniform_ = ID_UNDEFINED;
  int baseColorUniform_ = ID_UNDEFINED;  // diffuse color
  int roughnessUniform_ = ID_UNDEFINED;  // roughness of a surface
  int metallicUniform_ = ID_UNDEFINED;
  int albedoTextureUniform_ = ID_UNDEFINED;
  int roughnessTextureUniform_ = ID_UNDEFINED;
  int metallicTextureUniform_ = ID_UNDEFINED;
  int normalTextureUniform_ = ID_UNDEFINED;
  int objectIdUniform_ = ID_UNDEFINED;
};

CORRADE_ENUMSET_OPERATORS(PBRShader::Flags);

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRSHADER_H_
