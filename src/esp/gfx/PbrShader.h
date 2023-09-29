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
  enum class Flag : Magnum::UnsignedLong {
    /**
     * Multiply base color with the baseColor texture.
     * @see @ref setBaseColor(), @ref bindBaseColorTexture()
     */
    BaseColorTexture = 1ULL << 0,

    /**
     * This flag term means the NoneRoughnessMetallic texture is present, with
     * the Roughness in G channel and metalness in B channel (R and Alpha
     * channels are not used).
     * @see @ref setMetallic(), @ref bindMetallicTexture()
     */
    NoneRoughnessMetallicTexture = 1ULL << 1,

    /*
     * The occlusion map texture is present.
     * The occlusion, Roughness and Metalness are packed together in one
     * texture, with Occlusion in R channel, Roughness in G channel and
     * metalness in B channel (Alpha channels is not used).
     */
    OcclusionTexture = 1ULL << 2,

    /**
     * Modify normals according to a texture.
     */
    NormalTexture = 1ULL << 3,

    /**
     * emissive texture
     */
    EmissiveTexture = 1ULL << 4,

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
    TextureTransformation = 1ULL << 5,

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
    PrecomputedTangent = 1ULL << 6,

    /**
     * Enable object ID output for this shader.
     */
    ObjectId = 1ULL << 7,

    /**
     * Support Instanced object ID. Retrieves a per-instance / per-vertex
     * object ID from the @ref ObjectId attribute. If this is false, the shader
     * will use the node's semantic ID
     */
    InstancedObjectId = (1ULL << 8) | ObjectId,

    /**
     * Has ClearCoat layer.
     */
    ClearCoatLayer = 1ULL << 9,
    /**
     * Has ClearCoat Texture in ClearCoat layer
     */
    ClearCoatTexture = (1ULL << 10) | ClearCoatLayer,
    /**
     * Has Roughness Texture in ClearCoat layer
     */
    ClearCoatRoughnessTexture = (1ULL << 11) | ClearCoatLayer,
    /**
     * Has Normal Texture in ClearCoat layer
     */
    ClearCoatNormalTexture = (1ULL << 12) | ClearCoatLayer,

    /**
     * Has KHR_materials_specular layer
     */
    SpecularLayer = 1ULL << 13,
    /**
     * Has Specular Texture in KHR_materials_specular layer
     */
    SpecularLayerTexture = (1ULL << 14) | SpecularLayer,

    /**
     * Has Specular Color Texture in KHR_materials_specular layer
     */
    SpecularLayerColorTexture = (1ULL << 15) | SpecularLayer,

    /**
     * Has KHR_materials_anisotropy layer
     */
    AnisotropyLayer = 1ULL << 16,

    /**
     * Has Anisotropy Texture in KHR_materials_anisotropy layer
     */
    AnisotropyLayerTexture = (1ULL << 17) | AnisotropyLayer,

    /**
     * Has KHR_materials_transmission layer
     */
    TransmissionLayer = 1ULL << 18,
    /**
     * Has transmission texture in KHR_materials_transmission layer
     */
    TransmissionLayerTexture = (1ULL << 19) | TransmissionLayer,

    /**
     * Has KHR_materials_volume layer
     */
    VolumeLayer = 1ULL << 20,

    /**
     * Has Thickness texture in  KHR_materials_volume layer
     */
    VolumeLayerThicknessTexture = (1ULL << 21) | VolumeLayer,

    /**
     * Enable double-sided rendering.
     * (Temporarily STOP supporting this functionality. See comments in
     * the PbrDrawable::draw() function)
     */
    DoubleSided = 1ULL << 22,

    ///////////////////////////////
    // PbrShaderAttributes provides these values to configure the shader

    /**
     * If not set, disable direct lighting regardless of presence of lights.
     * Ignored if no direct lights present.
     */
    DirectLighting = 1ULL << 23,

    /**
     * Enable image based lighting
     */
    ImageBasedLighting = 1ULL << 24,

    /**
     * Whether or not the direct lighting diffuse calculation should use the
     * Disney/Burley algorithm or the lambertian calculation. If set, the PBR
     * shader uses a calc based on modified to be more energy conserving.
     * https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
     * Lambertian is simpler and quicker to calculate but may not look as 'nice'
     */
    UseBurleyDiffuse = 1ULL << 25,

    /**
     * If set, skip TBN frame calculation in fragment shader. This calculation
     * enables normal textures and anisotropy when no precomputed tangents are
     * provided.
     * TODO : implement in shader.
     */
    SkipMissingTBNCalc = 1ULL << 26,

    /**
     * Use the Mikkelsen algorithm to calculate TBN, as per
     * https://jcgt.org/published/0009/03/04/paper.pdf. If not set,
     * a simplified, faster method will be used to calculate the TBN frame,
     * based on
     * https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag,
     * which empirically seems to give equivalent results.
     */
    UseMikkelsenTBN = 1ULL << 27,

    /**
     * Whether we should use shader-based srgb->linear approx remapping of
     * applicable material color textures in PBR rendering for direct lighting
     * and IBL. This field should be removed/ignored when Magnum fully supports
     * sRGB texture conversion on load.
     */
    MapMatTxtrToLinear = 1ULL << 28,

    /**
     * Whether we should use shader-based srgb->linear approx remapping of
     * applicable IBL environment textures in PBR rendering for IBL
     * calculations. This field should be removed/ignored when Magnum fully
     * supports sRGB texture conversion on load.
     */
    MapIBLTxtrToLinear = 1ULL << 29,

    /**
     * Whether we should use shader-based linear->srgb approx remapping of
     * color output in PBR rendering for direct lighting and IBL results. This
     * field should be removed/ignored when an appropriate framebuffer is used
     * for output to handle this conversion.
     */
    MapOutputToSRGB = 1ULL << 30,

    /**
     * Whether or not to use tonemappping for direct lighting.
     */
    UseDirectLightTonemap = 1ULL << 31,

    /**
     * Whether or not to use tonemappping for image-based lighting.
     */
    UseIBLTonemap = 1ULL << 32,

    ////////////////
    // Testing and debugging
    /**
     * Whether we should skip all clearcoat layer calcs. Values will still be
     * sent to the shader, but no actual calcuations will be performed if this
     * is set.
     */
    SkipClearCoatLayer = 1ULL << 33,
    /**
     * Whether we should skip all specular layer calcs. Values will still be
     * sent to the shader, but no actual calcuations will be performed if this
     * is set.
     */
    SkipSpecularLayer = 1ULL << 34,
    /**
     * Whether we should skip all anisotropy layer calcs. Values will still be
     * sent to the shader, but no actual calcuations will be performed if this
     * is set.
     */
    SkipAnisotropyLayer = 1ULL << 35,

    /**
     * Enable shader debug mode. Then developer can set the uniform
     * PbrDebugDisplay in the fragment shader for debugging
     */
    DebugDisplay = 1ULL << 36,
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
  explicit PbrShader(Flags flags = {},
                     Magnum::UnsignedInt lightCount = 1,
                     Magnum::UnsignedInt jointCount = 0,
                     Magnum::UnsignedInt perVertexJointCount = 0);

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
  Magnum::UnsignedInt lightCount() const { return lightCount_; }

  /** @brief whether this shader as direct lighting enabled and there are lights
   * defined.*/
  bool directLightingIsEnabled() const { return directLightingIsEnabled_; }

  /** @brief whether this shader has any lighting enabled, either direct or
   * indirect/IBL.*/
  bool lightingIsEnabled() const { return lightingIsEnabled_; }

  /** @brief whether this shader has both direct lighting and IBL enabled.*/
  bool directAndIBLIsEnabled() const { return directAndIBLisEnabled_; }

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
  PbrShader& setLightPositions(
      Corrade::Containers::ArrayView<const Magnum::Vector4> vectors);

  /**
   * @overload
   */
  PbrShader& setLightPositions(std::initializer_list<Magnum::Vector4> vectors);

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
  PbrShader& setDirectLightIntensity(float lightIntensity);

  /**
   * @brief Set the gamma value used for remapping sRGB to linear approximations
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setGamma(const Magnum::Vector3& gamma);

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
   * @brief Set joint matrices
   * @return Reference to self (for method chaining)
   */
  PbrShader& setJointMatrices(
      Corrade::Containers::ArrayView<const Magnum::Matrix4> matrices);

  /**
   * @overload
   * @m_since_latest
   */
  PbrShader& setJointMatrices(std::initializer_list<Magnum::Matrix4> matrices);

  /**
   * Toggles that control contributions from different components - should
   * never be set to 0 or will cause warnings when the shader executes
   */
  struct PbrEquationScales {
    float directDiffuse = 0.5f;
    float directSpecular = 0.5f;
    float iblDiffuse = 0.5f;
    float iblSpecular = 0.5f;
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
  Magnum::UnsignedInt lightCount_;

  Magnum::UnsignedInt jointCount_;
  Magnum::UnsignedInt perVertexJointCount_;

  // whether or not this shader uses any textures
  bool isTextured_ = false;
  // Whether or not there is lighting - either direct or indirect - used by this
  // shader
  bool lightingIsEnabled_ = false;

  // Whether direct lighting is enabled and there are direct lights defined
  bool directLightingIsEnabled_ = false;

  // Whether direct _AND_ indirect lighting is available
  bool directAndIBLisEnabled_ = false;

  // Whether the any incoming textures should be remapped from sRGB to
  // linear for calculations. This will determine whether or not uGamma is
  // populated
  bool mapInputToLinear_ = false;

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

  int jointMatricesUniform_ = ID_UNDEFINED;
  // In the fragment shader, the "LightDirection" is a vec4.
  // when w == 0, it means .xyz is the light direction;
  // when w == 1, it means it is the light position, NOT the direction;
  int lightDirectionsUniform_ = ID_UNDEFINED;
  // Global, config-driven knob to control direct lighting intensity
  int directLightingIntensityUniform_ = ID_UNDEFINED;

  // Global, config-driven knob to control IBL exposure
  int tonemapExposureUniform_ = ID_UNDEFINED;

  // Gamma value for sRGB->linear mapping approx
  int gammaUniform_ = ID_UNDEFINED;
  // invGamma value for linear->sRGB mapping approx
  int invGammaUniform_ = ID_UNDEFINED;

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
