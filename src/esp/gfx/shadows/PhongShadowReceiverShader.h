#pragma once
/*
    Originally written by Vladimir Vondrus as part of the Magnum Library

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019
              Vladimír Vondruš <mosra@centrum.cz>
*/

#include "Magnum/GL/AbstractShaderProgram.h"
#include "Magnum/Shaders/Generic.h"
#include "Magnum/Shaders/visibility.h"

namespace esp {
namespace gfx {

class PhongShadowReceiverShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /**
   * @brief Vertex position
   *
   * @ref shaders-generic "Generic attribute",
   * @ref Magnum::Vector3 "Vector3".
   */
  typedef Magnum::Shaders::Generic3D::Position Position;

  /**
   * @brief Normal direction
   *
   * @ref shaders-generic "Generic attribute",
   * @ref Magnum::Vector3 "Vector3".
   */
  typedef Magnum::Shaders::Generic3D::Normal Normal;

  /**
   * @brief Tangent direction
   * @m_since{2019,10}
   *
   * @ref shaders-generic "Generic attribute",
   * @ref Magnum::Vector3 "Vector3", used only if
   * @ref Flag::NormalTexture is set.
   */
  typedef Magnum::Shaders::Generic3D::Tangent Tangent;

  /**
   * @brief 2D texture coordinates
   *
   * @ref shaders-generic "Generic attribute",
   * @ref Magnum::Vector2 "Vector2", used only if at least one of
   * @ref Flag::AmbientTexture, @ref Flag::DiffuseTexture and
   * @ref Flag::SpecularTexture is set.
   */
  typedef Magnum::Shaders::Generic3D::TextureCoordinates TextureCoordinates;

  /**
   * @brief Three-component vertex color
   * @m_since{2019,10}
   *
   * @ref shaders-generic "Generic attribute", @ref Magnum::Color3. Use
   * either this or the @ref Color4 attribute. Used only if
   * @ref Flag::VertexColor is set.
   */
  typedef Magnum::Shaders::Generic3D::Color3 Color3;

  /**
   * @brief Four-component vertex color
   * @m_since{2019,10}
   *
   * @ref shaders-generic "Generic attribute", @ref Magnum::Color4. Use
   * either this or the @ref Color3 attribute. Used only if
   * @ref Flag::VertexColor is set.
   */
  typedef Magnum::Shaders::Generic3D::Color4 Color4;

  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     * @m_since{2019,10}
     */
    ColorOutput = Magnum::Shaders::Generic3D::ColorOutput,

    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there, see
     * @ref Shaders-Phong-usage-object-id for more information.
     * @requires_gles30 Object ID output requires integer buffer
     *      attachments, which are not available in OpenGL ES 2.0 or
     *      WebGL 1.0.
     * @m_since{2019,10}
     */
    ObjectIdOutput = Magnum::Shaders::Generic3D::ObjectIdOutput

  };

  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedByte {
    /**
     * Multiply ambient color with a texture.
     * @see @ref setAmbientColor(), @ref bindAmbientTexture()
     */
    AmbientTexture = 1 << 0,

    /**
     * Multiply diffuse color with a texture.
     * @see @ref setDiffuseColor(), @ref bindDiffuseTexture()
     */
    DiffuseTexture = 1 << 1,

    /**
     * Multiply specular color with a texture.
     * @see @ref setSpecularColor(), @ref bindSpecularTexture()
     */
    SpecularTexture = 1 << 2,

    /**
     * Modify normals according to a texture. Requires the
     * @ref Tangent attribute to be present.
     * @m_since{2019,10}
     */
    NormalTexture = 1 << 4,

    /**
     * Enable alpha masking. If the combined fragment color has an
     * alpha less than the value specified with @ref setAlphaMask(),
     * given fragment is discarded.
     *
     * This uses the @glsl discard @ce operation which is known to have
     * considerable performance impact on some platforms. While useful
     * for cheap alpha masking that doesn't require depth sorting,
     * with proper depth sorting and blending you'll usually get much
     * better performance and output quality.
     */
    AlphaMask = 1 << 3,

    /**
     * Multiply diffuse color with a vertex color. Requires either
     * the @ref Color3 or @ref Color4 attribute to be present.
     * @m_since{2019,10}
     */
    VertexColor = 1 << 5,

    /**
     * Enable object ID output. See @ref Shaders-Phong-usage-object-id
     * for more information.
     * @requires_gles30 Object ID output requires integer buffer
     *      attachments, which are not available in OpenGL ES 2.0 or
     *      WebGL 1.0.
     * @m_since{2019,10}
     */
    ObjectId = 1 << 6

  };

  /**
   * @brief Flags
   *
   * @see @ref flags()
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief Constructor
   * @param flags         Flags
   * @param lightCount    Count of light sources
   */
  explicit PhongShadowReceiverShader(
      Flags flags = {},
      Magnum::UnsignedInt lightCount = 1,
      const Magnum::UnsignedInt numShadowMapLayers = 1);

  /**
   * @brief Construct without creating the underlying OpenGL object
   *
   * The constructed instance is equivalent to a moved-from state. Useful
   * in cases where you will overwrite the instance later anyway. Move
   * another object over it to make it useful.
   *
   * This function can be safely used for constructing (and later
   * destructing) objects even without any OpenGL context being active.
   * However note that this is a low-level and a potentially dangerous
   * API, see the documentation of @ref NoCreate for alternatives.
   */
  explicit PhongShadowReceiverShader(Magnum::NoCreateT) noexcept
      : Magnum::GL::AbstractShaderProgram{Magnum::NoCreate} {}

  /** @brief Copying is not allowed */
  PhongShadowReceiverShader(const PhongShadowReceiverShader&) = delete;

  /** @brief Move constructor */
  PhongShadowReceiverShader(PhongShadowReceiverShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PhongShadowReceiverShader& operator=(const PhongShadowReceiverShader&) =
      delete;

  /** @brief Move assignment */
  PhongShadowReceiverShader& operator=(PhongShadowReceiverShader&&) noexcept =
      default;

  /** @brief Flags */
  Flags flags() const { return _flags; }

  /** @brief Light count */
  Magnum::UnsignedInt lightCount() const { return _lightCount; }

  /**
   * @brief Set ambient color
   * @return Reference to self (for method chaining)
   *
   * If @ref Flag::AmbientTexture is set, default value is
   * @cpp 0xffffffff_rgbaf @ce and the color will be multiplied with
   * ambient texture, otherwise default value is @cpp 0x00000000_rgbaf @ce.
   * @see @ref bindAmbientTexture()
   */
  PhongShadowReceiverShader& setAmbientColor(const Magnum::Color4& color);

  /**
   * @brief Bind an ambient texture
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::AmbientTexture
   * enabled.
   * @see @ref bindTextures(), @ref setAmbientColor()
   */
  PhongShadowReceiverShader& bindAmbientTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Set diffuse color
   * @return Reference to self (for method chaining)
   *
   * Initial value is @cpp 0xffffffff_rgbaf @ce. If @ref lightCount() is
   * zero, this function is a no-op, as diffuse color doesn't contribute
   * to the output in that case.
   * @see @ref bindDiffuseTexture()
   */
  PhongShadowReceiverShader& setDiffuseColor(const Magnum::Color4& color);

  /**
   * @brief Bind a diffuse texture
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::DiffuseTexture
   * enabled. If @ref lightCount() is zero, this function is a no-op, as
   * diffuse color doesn't contribute to the output in that case.
   * @see @ref bindTextures(), @ref setDiffuseColor()
   */
  PhongShadowReceiverShader& bindDiffuseTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind a normal texture
   * @return Reference to self (for method chaining)
   * @m_since{2019,10}
   *
   * Expects that the shader was created with @ref Flag::NormalTexture
   * enabled and the @ref Tangent attribute was supplied. If
   * @ref lightCount() is zero, this function is a no-op, as normals
   * dosn't contribute to the output in that case.
   * @see @ref bindTextures()
   */
  PhongShadowReceiverShader& bindNormalTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Set specular color
   * @return Reference to self (for method chaining)
   *
   * Initial value is @cpp 0xffffffff_rgbaf @ce. Color will be multiplied
   * with specular texture if @ref Flag::SpecularTexture is set. If you
   * want to have a fully diffuse material, set specular color to
   * @cpp 0x000000ff_rgbaf @ce. If @ref lightCount() is zero, this
   * function is a no-op, as specular color doesn't contribute to the
   * output in that case.
   * @see @ref bindSpecularTexture()
   */
  PhongShadowReceiverShader& setSpecularColor(const Magnum::Color4& color);

  /**
   * @brief Bind a specular texture
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::SpecularTexture
   * enabled. If @ref lightCount() is zero, this function is a no-op, as
   * specular color doesn't contribute to the output in that case.
   * @see @ref bindTextures(), @ref setSpecularColor()
   */
  PhongShadowReceiverShader& bindSpecularTexture(
      Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind textures
   * @return Reference to self (for method chaining)
   *
   * A particular texture has effect only if particular texture flag from
   * @ref Phong::Flag "Flag" is set, you can use @cpp nullptr @ce for the
   * rest. Expects that the shader was created with at least one of
   * @ref Flag::AmbientTexture, @ref Flag::DiffuseTexture,
   * @ref Flag::SpecularTexture or @ref Flag::NormalTexture enabled. More
   * efficient than setting each texture separately.
   * @see @ref bindAmbientTexture(), @ref bindDiffuseTexture(),
   *      @ref bindSpecularTexture(), @ref bindNormalTexture()
   */
  PhongShadowReceiverShader& bindTextures(Magnum::GL::Texture2D* ambient,
                                          Magnum::GL::Texture2D* diffuse,
                                          Magnum::GL::Texture2D* specular,
                                          Magnum::GL::Texture2D* normal);

  /**
   * @brief Set shininess
   * @return Reference to self (for method chaining)
   *
   * The larger value, the harder surface (smaller specular highlight).
   * Initial value is @cpp 80.0f @ce. If @ref lightCount() is zero, this
   * function is a no-op, as specular color doesn't contribute to the
   * output in that case.
   */
  PhongShadowReceiverShader& setShininess(Magnum::Float shininess);

  /**
   * @brief Set alpha mask value
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::AlphaMask
   * enabled. Fragments with alpha values smaller than the mask value
   * will be discarded. Initial value is @cpp 0.5f @ce. See the flag
   * documentation for further information.
   */
  PhongShadowReceiverShader& setAlphaMask(Magnum::Float mask);

  /**
   * @brief Set object ID
   * @return Reference to self (for method chaining)
   *
   * Expects that the shader was created with @ref Flag::ObjectId
   * enabled. Value set here is written to the @ref ObjectIdOutput, see
   * @ref Shaders-Phong-usage-object-id for more information. Default is
   * @cpp 0 @ce.
   * @requires_gles30 Object ID output requires integer buffer
   *      attachments, which are not available in OpenGL ES 2.0 or WebGL
   *      1.0.
   */
  PhongShadowReceiverShader& setObjectId(Magnum::UnsignedInt id);

  /**
   * @brief Set transformation matrix
   * @return Reference to self (for method chaining)
   *
   * You need to set also @ref setNormalMatrix() with a corresponding
   * value. Initial value is an identity matrix.
   */
  PhongShadowReceiverShader& setTransformationMatrix(
      const Magnum::Matrix4& matrix);

  /**
   * @brief Set normal matrix
   * @return Reference to self (for method chaining)
   *
   * The matrix doesn't need to be normalized, as the renormalization
   * must be done in the shader anyway. You need to set also
   * @ref setTransformationMatrix() with a corresponding value. Initial
   * value is an identity matrix. If @ref lightCount() is zero, this
   * function is a no-op, as normals don't contribute to the output in
   * that case.
   * @see @ref Math::Matrix4::normalMatrix()
   */
  PhongShadowReceiverShader& setNormalMatrix(const Magnum::Matrix3x3& matrix);

  /**
   * @brief Set projection matrix
   * @return Reference to self (for method chaining)
   *
   * Initial value is an identity matrix (i.e., an orthographic
   * projection of the default @f$ [ -\boldsymbol{1} ; \boldsymbol{1} ] @f$
   * cube).
   */
  PhongShadowReceiverShader& setProjectionMatrix(const Magnum::Matrix4& matrix);

  /**
   * @brief Set light positions
   * @return Reference to self (for method chaining)
   *
   * Initial values are zero vectors --- that will in most cases cause
   * the object to be rendered black (or in the ambient color), as the
   * lights are is inside of it. Expects that the size of the @p lights
   * array is the same as @ref lightCount().
   * @see @ref setLightPosition(UnsignedInt, const Vector3&),
   *      @ref setLightPosition(const Vector3&)
   */
  PhongShadowReceiverShader& setLightPositions(
      Corrade::Containers::ArrayView<const Magnum::Vector3> lights);

  /** @overload */
  PhongShadowReceiverShader& setLightPositions(
      std::initializer_list<Magnum::Vector3> lights);

  /**
   * @brief Set position for given light
   * @return Reference to self (for method chaining)
   *
   * Unlike @ref setLightPosition() updates just a single light position.
   * Expects that @p id is less than @ref lightCount().
   * @see @ref setLightPosition(const Vector3&)
   */
  PhongShadowReceiverShader& setLightPosition(Magnum::UnsignedInt id,
                                              const Magnum::Vector3& position);

  /**
   * @brief Set light position
   * @return Reference to self (for method chaining)
   *
   * Convenience alternative to @ref setLightPositions() when there is
   * just one light.
   * @see @ref setLightPosition(Magnum::UnsignedInt, const Vector3&)
   */
  PhongShadowReceiverShader& setLightPosition(const Magnum::Vector3& position) {
    return setLightPositions({&position, 1});
  }

  /**
   * @brief Set light colors
   * @return Reference to self (for method chaining)
   *
   * Initial values are @cpp 0xffffffff_rgbaf @ce. Expects that the size
   * of the @p colors array is the same as @ref lightCount().
   */
  PhongShadowReceiverShader& setLightColors(
      Corrade::Containers::ArrayView<const Magnum::Color4> colors);

  /** @overload */
  PhongShadowReceiverShader& setLightColors(
      std::initializer_list<Magnum::Color4> colors);

  /**
   * @brief Set position for given light
   * @return Reference to self (for method chaining)
   *
   * Unlike @ref setLightColors() updates just a single light color.
   * Expects that @p id is less than @ref lightCount().
   * @see @ref setLightColor(const Magnum::Color4&)
   */
  PhongShadowReceiverShader& setLightColor(Magnum::UnsignedInt id,
                                           const Magnum::Color4& color);

  /**
   * @brief Set light color
   * @return Reference to self (for method chaining)
   *
   * Convenience alternative to @ref setLightColors() when there is just
   * one light.
   * @see @ref setLightColor(Magnum::UnsignedInt, const Magnum::Color4&)
   */
  PhongShadowReceiverShader& setLightColor(const Magnum::Color4& color) {
    return setLightColors({&color, 1});
  }

  PhongShadowReceiverShader& setTransformationProjectionMatrix(
      const Magnum::Matrix4& matrix);

  /**
   * @brief Set model matrix
   *
   * Matrix that transforms from local model space -> world space (used
   * for lighting).
   */
  PhongShadowReceiverShader& setModelMatrix(const Magnum::Matrix4& matrix);

  /**
   * @brief Set shadowmap matrices
   *
   * Matrix that transforms from world space -> shadow texture space.
   */
  PhongShadowReceiverShader& setShadowmapMatrices(
      Corrade::Containers::ArrayView<const Magnum::Matrix4> matrices);

  /** @brief Set world-space direction to the light source */
  PhongShadowReceiverShader& setShadowLightDirection(
      const Magnum::Vector3& vector3);

  /** @brief Set shadow map texture array */
  PhongShadowReceiverShader& setShadowmapTexture(
      Magnum::GL::Texture2DArray& texture);

  /** @brief Set shadow bias */
  PhongShadowReceiverShader& setShadowBias(const Magnum::Float bias);

  PhongShadowReceiverShader& setShadeFacesFacingAwayFromLight(bool shadeFaces);

  Magnum::UnsignedInt getNumLayers() const { return numShadowMapLayers_; }

 private:
  Flags _flags;
  Magnum::UnsignedInt _lightCount;
  Magnum::UnsignedInt numShadowMapLayers_;
  Magnum::Int _transformationMatrixUniform{0}, _projectionMatrixUniform{1},
      _normalMatrixUniform{2}, shadeFacesFacingAwayFromLight_{3},
      _ambientColorUniform{4}, _diffuseColorUniform{5},
      _specularColorUniform{6}, _shininessUniform{7}, _alphaMaskUniform{8};
  Magnum::Int _objectIdUniform{9};
  Magnum::Int modelMatrixUniform_{10};
  Magnum::Int shadowLightDirectionUniform_{11};
  Magnum::Int shadowBiasUniform_{12};
  Magnum::Int shadowMapMatrixUniform_{13};
  Magnum::Int _lightPositionsUniform{14},
      _lightColorsUniform; /* 14 + lightCount, set in the constructor */
};

CORRADE_ENUMSET_OPERATORS(PhongShadowReceiverShader::Flags)

}  // namespace gfx
}  // namespace esp
