// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBRSHADER_H_
#define ESP_GFX_PBRSHADER_H_

#include <initializer_list>

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/GL.h>  // header with all forward declarations for the Mn::GL namespace
#include <Magnum/Shaders/GenericGL.h>

#include "esp/core/Esp.h"

namespace esp {
namespace gfx {

class PbrShader : public Magnum::GL::AbstractShaderProgram {
 public:
  // ==== Attribute definitions ====
  /**
   * @brief vertex positions
   */
  typedef Magnum::Shaders::GenericGL3D::Position Position;

  /**
   * @brief normal direction
   */
  typedef Magnum::Shaders::GenericGL3D::Normal Normal;

  /**
   * @brief 2D texture coordinates
   *
   * Used only if at least one of
   * @ref Flag::BaseColorTexture, @ref Flag::NormalTexture and
   * @ref Flag::RoughnessTexture @ref Flag::MetallicTexture is set.
   */
  typedef Magnum::Shaders::GenericGL3D::TextureCoordinates TextureCoordinates;

  /**
   * @brief Tangent direction with the fourth component indicating the
   * handedness.
   *
   * T = Tangent, B = BiTangent, N = Normal
   *
   * 1.0 means T, B, N form a right-handed coordinate;
   * -1.0 means T, B, N form a left-handed coordinate;
   *
   * Used only if @ref Flag::NormalTexture is set.
   */
  typedef Magnum::Shaders::GenericGL3D::Tangent4 Tangent4;

  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,

    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     */
    ObjectIdOutput = Magnum::Shaders::GenericGL3D::ObjectIdOutput,
  };

  /**
   * @brief Flags enums describing various features present in shader
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedInt {
    /**
     * Multiply base color with the baseColor texture.
     * @see @ref setBaseColor(), @ref bindBaseColorTexture()
     */
    BaseColorTexture = 1 << 0,

    /**
     * This flag term means the NoneRoughnessMetallic texture is present, with
     * the Roughness in G channel and metalness in B channel (R and Alpha
     * channels are not used).
     * @see @ref setMetallic(), @ref bindMetallicTexture()
     */
    NoneRoughnessMetallicTexture = 1 << 1,

    /*
     * The occlusion map texture is present.
     * The occlusion, Roughness and Metalness are packed together in one
     * texture, with Occlusion in R channel, Roughness in G channel and
     * metalness in B channel (Alpha channels is not used).
     */
    OcclusionTexture = 1 << 2,

    /**
     * Modify normals according to a texture.
     */
    NormalTexture = 1 << 3,

    /**
     * emissive texture
     */
    EmissiveTexture = 1 << 4,

    /**
     * Enable texture coordinate transformation. If this flag is set,
     * the shader expects that at least one of
     * @ref Flag::BaseColorTexture, @ref Flag::RoughnessTexture,
     * @ref Flag::MetallicTexture, @ref Flag::NormalTexture,
     * @ref Flag::EmissiveTexture
     * @ref Flag::NoneRoughnessMetallicTexture or
     * @ref Flag::OcclusionRoughnessMetallicTexture is enabled as well.
     * @see @ref setTextureMatrix()
     */
    TextureTransformation = 1 << 5,

    /**
     * TODO: Do we need instanced object? (instanced texture, instanced id etc.)
     */

    /**
     * TODO: Do we need VertexColor?
     * Multiply diffuse color with a vertex color. Requires either
     * the @ref Color3 or @ref Color4 attribute to be present.
     */

    /*
     * Precomputed tangent as the vertex attribute
     * Otherwise, it will be computed in the fragment shader dynamically
     * see PBR fragment shader code for more details
     * Requires the @ref Tangent4 attribute to be present.
     */
    PrecomputedTangent = 1 << 6,

    /**
     * Enable object ID output for this shader.
     */
    ObjectId = 1 << 7,

    /**
     * Support Instanced object ID. Retrieves a per-instance / per-vertex
     * object ID from the @ref ObjectId attribute. If this is false, the shader
     * will use the node's semantic ID
     */
    InstancedObjectId = (1 << 8) | ObjectId,

    /**
     * Has ClearCoat layer.
     */
    ClearCoatLayer = 1 << 9,
    /**
     * Has ClearCoat Texture in ClearCoat layer
     */
    ClearCoatTexture = (1 << 10) | ClearCoatLayer,
    /**
     * Has Roughness Texture in ClearCoat layer
     */
    ClearCoatRoughnessTexture = (1 << 11) | ClearCoatLayer,
    /**
     * Has Normal Texture in ClearCoat layer
     */
    ClearCoatNormalTexture = (1 << 12) | ClearCoatLayer,

    /**
     * Has KHR_materials_specular layer
     */
    SpecularLayer = 1 << 13,
    /**
     * Has Specular Texture in KHR_materials_specular layer
     */
    SpecularLayerTexture = (1 << 14) | SpecularLayer,

    /**
     * Has Specular Color Texture in KHR_materials_specular layer
     */
    SpecularLayerColorTexture = (1 << 15) | SpecularLayer,

    /**
     * Has KHR_materials_anisotropy layer
     */
    AnisotropyLayer = 1 << 16,

    /**
     * Has Anisotropy Texture in KHR_materials_anisotropy layer
     */
    AnisotropyLayerTexture = (1 << 17) | AnisotropyLayer,

    /**
     * Has KHR_materials_transmission layer
     */
    TransmissionLayer = 1 << 18,
    /**
     * Has transmission texture in KHR_materials_transmission layer
     */
    TransmissionLayerTexture = (1 << 19) | TransmissionLayer,

    /**
     * Has KHR_materials_volume layer
     */
    VolumeLayer = 1 << 20,

    /**
     * Has Thickness texture in  KHR_materials_volume layer
     */
    VolumeLayerThicknessTexture = (1 << 21) | VolumeLayer,

    /**
     * Enable double-sided rendering.
     * (Temporarily STOP supporting this functionality. See comments in
     * the PbrDrawable::draw() function)
     */
    DoubleSided = 1 << 22,

    /**
     * Enable image based lighting
     */
    ImageBasedLighting = 1 << 23,

    /**
     * Enable shader debug mode. Then developer can set the uniform
     * PbrDebugDisplay in the fragment shader for debugging
     */
    DebugDisplay = 1 << 24,
    /*
     * TODO: alphaMask
     */
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief Constructor
   * @param flags         Flags
   * @param lightCount    Count of light sources
   *
   * By default,
   *
   * the shader provides a single directional "fill" light, coming
   * from the center of the camera. Using the @p lightCount parameter in
   * constructor, you can specify how many lights you want, and then control
   * light parameters using @ref setLightVectors(), @ref setLightColors(),
   * and @ref setLightRanges(). Light positions (directions)
   * are specified as four-component vectors, the last component distinguishing
   * between directional (w == 0) and point lights (w == 1.0).
   *
   * the shader renders the mesh with a white color in an identity
   * transformation.
   *
   * the light range is set to Magnum::Constants::inf()
   */
  explicit PbrShader(Flags flags = {}, unsigned int lightCount = 1);

  /** @brief Copying is not allowed */
  PbrShader(const PbrShader&) = delete;

  /** @brief Move constructor */
  PbrShader(PbrShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PbrShader& operator=(const PbrShader&) = delete;

  /** @brief Move assignment */
  PbrShader& operator=(PbrShader&&) noexcept = default;

  /**
   * @brief Get number of lights
   */
  unsigned int lightCount() const { return lightCount_; }

  /** @brief whether this shader has any lighting enabled, either direct or
   * indirect/IBL.*/
  bool lightingIsEnabled() const { return lightingIsEnabled_; }

  /** @brief whether any textures are present in this shader.*/
  bool isTextured() const { return isTextured_; }

  /** @brief Flags */
  Flags flags() const { return flags_; }

  // ======== texture binding ========
  /**
   * @brief Bind the BaseColor texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindBaseColorTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the metallic-roughness texture
   * NOTE that though MetallicRoughnessTexture exists, it does not mean both
   * metallic texture and roughness texture exist.
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindMetallicRoughnessTexture(Magnum::GL::Texture2D& texture);
  /**
   * @brief Bind the normal texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindNormalTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the emissive texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindEmissiveTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the clearcoat factor texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindClearCoatFactorTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the clearcoat roughness texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindClearCoatRoughnessTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the clearcoat normal texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindClearCoatNormalTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the specular layer texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindSpecularLayerTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the specular layer color texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindSpecularLayerColorTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the anisotropy layer texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindAnisotropyLayerTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the irradiance cubemap texture
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindIrradianceCubeMap(Magnum::GL::CubeMapTexture& texture);

  /**
   * @brief Bind the BRDF LUT texture
   * NOTE: requires Flag::ImageBasedLighting is set
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindBrdfLUT(Magnum::GL::Texture2D& texture);

  /**
   * @brief Bind the prefiltered environment map (cubemap texture)
   * NOTE: requires Flag::ImageBasedLighting is set
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindPrefilteredMap(Magnum::GL::CubeMapTexture& texture);

  // ======== set uniforms ===========
  /**
   * @brief set the texture transformation matrix
   * @return Reference to self (for method chaining)
   */
  PbrShader& setTextureMatrix(const Magnum::Matrix3& matrix);

  /**
   *  @brief Set "projection" matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setProjectionMatrix(const Magnum::Matrix4& matrix);

  /**
   *  @brief Set view matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setViewMatrix(const Magnum::Matrix4& matrix);

  /**
   *  @brief Set model matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setModelMatrix(const Magnum::Matrix4& matrix);

  /**
   *  @brief Set normal matrix to the uniform on GPU
   *         normal = inverse transpose of the up-left 3x3 matrix of the
   *         modelview matrix
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setNormalMatrix(const Magnum::Matrix3x3& matrix);

  // -------- materials ---------------
  /**
   *  @brief Set base color to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setBaseColor(const Magnum::Color4& color);
  /**
   *  @brief Set emissive color to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setEmissiveColor(const Magnum::Color3& color);
  /**
   *  @brief Set roughness to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setRoughness(float roughness);
  /**
   *  @brief Set metallic to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setMetallic(float metallic);

  /**
   * @brief Set index of refraction.
   * @return Reference to self (for method chaining)
   */
  PbrShader& setIndexOfRefraction(float ior);

  /**
   * @brief Set clearcoat intensity/factor
   * @return Reference to self (for method chaining)
   */
  PbrShader& setClearCoatFactor(float ccFactor);

  /**
   * @brief Set clearcoat roughness
   * @return Reference to self (for method chaining)
   */
  PbrShader& setClearCoatRoughness(float ccRoughness);

  /**
   * @brief Set clearcoat normal texture scale
   * @return Reference to self (for method chaining)
   */
  PbrShader& setClearCoatNormalTextureScale(float ccTextureScale);

  /**
   * @brief Set specular layer factor
   * @return Reference to self (for method chaining)
   */
  PbrShader& setSpecularLayerFactor(float specLayerFactor);

  /**
   * @brief Set specular layer color factor
   * @return Reference to self (for method chaining)
   */
  PbrShader& setSpecularLayerColorFactor(const Magnum::Color3& color);

  /**
   * @brief Set anisotropy layer factor
   * @return Reference to self (for method chaining)
   */
  PbrShader& setAnisotropyLayerFactor(float anisoLayerFactor);

  /**
   * @brief Set anisotropy layer direction 2d vector
   * @return Reference to self (for method chaining)
   */
  PbrShader& setAnisotropyLayerDirection(
      const Magnum::Vector2& anisoLayerDirection);

  /**
   *  @brief Set object id to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setObjectId(unsigned int objectId);

  /**
   *  @brief Set object id to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setCameraWorldPosition(const Magnum::Vector3& cameraWorldPos);

  /**
   *  @brief Set total mipmap levels of the prefiltered environment map to the
   * uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setPrefilteredMapMipLevels(unsigned int mipLevels);

  /**
   * @brief Set light positions or directions
   * @param vectors an array of the light vectors
   * @return Reference to self (for method chaining)
   *
   * when vec.w == 0, it means vec.xyz is the light direction;
   * when vec.w == 1, it means vec.xyz is the light position;
   * vec is an element in the "vectors" array
   */
  PbrShader& setLightVectors(
      Corrade::Containers::ArrayView<const Magnum::Vector4> vectors);

  /**
   * @overload
   */
  PbrShader& setLightVectors(std::initializer_list<Magnum::Vector4> vectors);

  /**
   *  @brief Set the position or direction of a specific light See @ref vec for
   *  details
   *  @param lightIndex the index of the light, MUST be smaller than
   *                     lightCount_
   *  @param vec the direction (or position) of the light in *camera* space;
   *              when vec.w == 0, it means vec.xyz is the light direction;
   *              when vec.w == 1, it means vec.xyz is the light position;
   *  @return Reference to self (for method chaining)
   *  Note:
   *  If the light was a directional (point) light, it will be overridden as a
   *  point (directional) light
   */
  PbrShader& setLightVector(unsigned int lightIndex,
                            const Magnum::Vector4& vec);

  /**
   *  @brief Set the position of a specific light.
   *  @param lightIndex the index of the light, MUST be smaller than
   * lightCount_
   *  @param pos the position of the light in *camera* space
   *  @return Reference to self (for method chaining)
   *  Note:
   *  If the light was a directional light, it will be overridden as a point
   *  light;
   */
  PbrShader& setLightPosition(unsigned int lightIndex,
                              const Magnum::Vector3& pos);

  /**
   *  @brief Set the direction of a specific light.
   *  @param lightIndex the index of the light, MUST be smaller than
   * lightCount_
   *  @param dir the direction of the light in *camera* space
   *  @return Reference to self (for method chaining)
   *  NOTE:
   *  If the light was a point light, it will be overridden as a direction
   * light;
   */
  PbrShader& setLightDirection(unsigned int lightIndex,
                               const Magnum::Vector3& dir);

  /**
   *  @brief Set the range of a specific light.
   *  @param lightIndex the index of the light, MUST be smaller than
   * lightCount_
   *  @param range the range of the light
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setLightRange(unsigned int lightIndex, float range);

  /**
   *  @brief Set the color of a specific light.
   *  @param lightIndex the index of the light, MUST be smaller than
   * lightCount_
   *  @param color the color of the light
   *  @param intensity the intensity of the light
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setLightColor(unsigned int lightIndex,
                           const Magnum::Vector3& color,
                           float intensity = 1.0f);

  /**
   *  @brief Set the colors of the lights
   *  @param colors the colors of the lights
   *  NOTE: the intensity MUST be included in the color
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setLightColors(
      Corrade::Containers::ArrayView<const Magnum::Color3> colors);

  /**
   * @overload
   */
  PbrShader& setLightColors(std::initializer_list<Magnum::Color3> colors);

  /**
   *  @brief Set the ranges of the lights
   *  @param ranges the ranges of the lights
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setLightRanges(Corrade::Containers::ArrayView<const float> ranges);

  /**
   * @overload
   */
  PbrShader& setLightRanges(std::initializer_list<float> ranges);

  /**
   * @brief Set the global lighting intensity applied equally across all lights
   * for direct lighting.
   *  @param lightIntensity config-driven global intensity knob to easily
   * control the intensity of the entire scene by a single field
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setGlobalLightIntensity(float lightIntensity);

  /**
   * @brief Set the gamma value used for remapping sRGB to linear approximations
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setGamma(float gamma);

  /**
   * @brief Set the IBL exposure value.
   *  @param exposure config-driven exposure value for IBL calculations.
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setTonemapExposure(float exposure);

  /**
   *  @brief Set the scale of the normal texture
   *  @param scale
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setNormalTextureScale(float scale);

  /**
   * Toggles that control contributions from different components - should
   * never be set to 0 or will cause warnings when the shader executes
   */
  struct PbrEquationScales {
    float directDiffuse = 1.0f;
    float directSpecular = 1.0f;
    float iblDiffuse = 1.0f;
    float iblSpecular = 1.0f;
  };

  /**
   *  @brief Set the scales for different components in the pbr equation
   *  @param scales
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setPbrEquationScales(const PbrEquationScales& scales);

  enum class PbrDebugDisplay : uint8_t {
    None = 0,
    // Direct Lighting Diffuse result
    DirectDiffuse = 1,
    // Direct Lighting Specular result
    DirectSpecular = 2,
    // IBL Diffuse result
    IblDiffuse = 3,
    // IBL Specular result
    IblSpecular = 4,
    // Normal vector
    Normal = 5,
  };
  /**
   *@brief debug display visualization
   */
  PbrShader& setDebugDisplay(PbrDebugDisplay index);

 protected:
  Flags flags_;
  unsigned int lightCount_;

  // whether or not this shader uses any textures
  bool isTextured_ = false;
  // whether or not there is lighting - either direct or indirect - used by this
  // shader
  bool lightingIsEnabled_ = false;

  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int viewMatrixUniform_ = ID_UNDEFINED;
  int modelMatrixUniform_ = ID_UNDEFINED;
  int normalMatrixUniform_ = ID_UNDEFINED;
  int projMatrixUniform_ = ID_UNDEFINED;
  int baseColorUniform_ = ID_UNDEFINED;  // diffuse color
  int roughnessUniform_ = ID_UNDEFINED;  // roughness of a surface
  int metallicUniform_ = ID_UNDEFINED;
  int iorUniform_ = ID_UNDEFINED;
  int emissiveColorUniform_ = ID_UNDEFINED;
  int objectIdUniform_ = ID_UNDEFINED;
  int textureMatrixUniform_ = ID_UNDEFINED;
  int normalTextureScaleUniform_ = ID_UNDEFINED;
  int lightColorsUniform_ = ID_UNDEFINED;
  int lightRangesUniform_ = ID_UNDEFINED;
  // In the fragment shader, the "LightDirection" is a vec4.
  // when w == 0, it means .xyz is the light direction;
  // when w == 1, it means it is the light position, NOT the direction;
  int lightDirectionsUniform_ = ID_UNDEFINED;
  // Global, config-driven knob to control lighting intensity
  int globalLightingIntensityUniform_ = ID_UNDEFINED;

  // Global, config-driven knob to control IBL exposure
  int tonemapExposureUniform_ = ID_UNDEFINED;

  // Gamma value for sRGB mapping approx
  int gammaUniform_ = ID_UNDEFINED;

  int cameraWorldPosUniform_ = ID_UNDEFINED;
  int prefilteredMapMipLevelsUniform_ = ID_UNDEFINED;

  // Clearcoat layer
  int clearCoatFactorUniform_ = ID_UNDEFINED;

  int clearCoatTextureScaleUniform_ = ID_UNDEFINED;

  int clearCoatRoughnessUniform_ = ID_UNDEFINED;

  // Specular Layer
  int specularLayerFactorUniform_ = ID_UNDEFINED;

  int specularLayerColorFactorUniform_ = ID_UNDEFINED;

  int anisotropyLayerFactorUniform_ = ID_UNDEFINED;

  int anisotropyLayerDirectionUniform_ = ID_UNDEFINED;

  // scales
  int componentScalesUniform_ = ID_UNDEFINED;

  // pbr debug info
  int pbrDebugDisplayUniform_ = ID_UNDEFINED;
};

CORRADE_ENUMSET_OPERATORS(PbrShader::Flags)

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRSHADER_H_
