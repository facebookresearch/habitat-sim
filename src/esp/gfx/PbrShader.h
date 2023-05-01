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
   * @brief Tangent direction with the fourth component indicating the handness.
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
   * @brief Flag
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
     * Multiply roughness with the roughness texture.
     * This flag term means the roughness texture is independent, and
     * "roughness" is stored in the R channel of it.
     * NOTE:
     * if NoneRoughnessMetallicTexture or OcclusionRoughnessMetallicTexture are
     * presented, this texture will be ignored.
     * @see @ref setRoughness(), @ref bindRoughnessTexture()
     */
    RoughnessTexture = 1 << 1,

    /**
     * Multiply metallic with the metallic texture.
     * This flag term means the metallic texture is independent, and "metalness"
     * is stored in the B channel of it.
     * NOTE:
     * if NoneRoughnessMetallicTexture or OcclusionRoughnessMetallicTexture are
     * presented, this texture will be ignored.
     * @see @ref setMetallic(), @ref bindMetallicTexture()
     */
    MetallicTexture = 1 << 2,

    /**
     * This flag term means the NoneRoughnessMetallic texture is present, with
     * the Roughness in G channel and metalness in B channel (R and Alpha
     * channels are not used).
     * @see @ref setMetallic(), @ref bindMetallicTexture()
     */
    NoneRoughnessMetallicTexture = 1 << 3,

    /*
     * The occlusion map texture.
     * The occlusion, Roughness and Metalness are packed together in one
     * texture, with Occlusion in R channel, Roughness in G channel and
     * metalness in B channel (Alpha channels is not used).
     */
    PackedOcclusionTexture = 1 << 4,

    /*
     * The occlusion map texture.
     * The occlusion map texture is separate from the metallicRoughness texture.
     * The values are sampled from the R channel.
     */
    SeparateOcclusionTexture = 1 << 5,

    /**
     * Modify normals according to a texture.
     */
    NormalTexture = 1 << 6,

    /**
     * Enable normal texture scale
     * the shader expects that
     * @ref Flag::NormalTexture is enabled as well.
     * @see @ref setNormalTextureScale
     */
    NormalTextureScale = 1 << 7,

    /**
     * emissive texture
     */
    EmissiveTexture = 1 << 8,

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
    TextureTransformation = 1 << 9,

    /**
     * TODO: Do we need instanced object? (instanced texture, istanced id etc.)
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
    PrecomputedTangent = 1 << 10,

    /**
     * Enable object ID output for this shader.
     */
    ObjectId = 1 << 11,

    /**
     * Support Instanced object ID. Retrieves a per-instance / per-vertex
     * object ID from the @ref ObjectId attribute. If this is false, the shader
     * will use the node's semantic ID
     */
    InstancedObjectId = (1 << 12) | ObjectId,

    /**
     * Has ClearCoat layer.
     */
    ClearCoatLayer = 1 << 13,

    /**
     * Has ClearCoat Texture in ClearCoat layer
     */
    CCLayer_CCTexture = (1 << 14) | ClearCoatLayer,
    /**
     * Has Roughness Texture in ClearCoat layer
     */
    CCLayer_RoughnessTexture = (1 << 15) | ClearCoatLayer,
    /**
     * Has Normal Texture in ClearCoat layer
     */
    CCLayer_NormalTexture = (1 << 16) | ClearCoatLayer,

    CCLayer_NormalTextureScale = (1 << 17) | ClearCoatLayer,

    /**
     * Has KHR_materials_specular layer
     */
    SpecularLayer = 1 << 18,

    /**
     * Has Specular Texture in KHR_materials_specular layer
     */
    SpecLayer_SpecTexture = (1 << 19) | SpecularLayer,

    /**
     * Has Specular Color Texture in KHR_materials_specular layer
     */
    SpecLayer_SpecColorTexture = (1 << 20) | SpecularLayer,

    /**
     * Has KHR_materials_transmission layer
     */
    TransmissionLayer = 1 << 21,

    /**
     * Has transmission texture in KHR_materials_transmission layer
     */
    TransLayer_TransmissionTexture = (1 << 22) | TransmissionLayer,

    /**
     * Has KHR_materials_volume layer
     */
    VolumeLayer = 1 << 23,

    /**
     * Has Thickness texture in  KHR_materials_volume layer
     */
    VolLayer_ThicknessTexture = (1 << 24) | VolumeLayer,

    /**
     * Enable double-sided rendering.
     * (Temporarily STOP supporting this functionality. See comments in
     * the PbrDrawable::draw() function)
     */
    DoubleSided = 1 << 25,

    /**
     * Enable image based lighting
     */
    ImageBasedLighting = 1 << 26,

    /**
     * render point light shadows using variance shadow map (VSM)
     */
    ShadowsVSM = 1 << 27,

    /**
     * Enable shader debug mode. Then developer can set the uniform
     * PbrDebugDisplay in the fragment shader for debugging
     */
    DebugDisplay = 1 << 28,
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

  /**
   * @brief Bind the point shadow map (cubemap texture)
   * @param[in] idx, the index of the shadow map, can be 0, 1, or 2. (We allow
   * at most 3 shadow maps.)
   * NOTE: requires Flag::ShadowsPCF or Flag::ShadowsVSM is set
   * @return Reference to self (for method chaining)
   */
  PbrShader& bindPointShadowMap(int idx, Magnum::GL::CubeMapTexture& texture);

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
   *  If the light was a directional (point) light, it will be overrided as a
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
   *  If the light was a directional light, it will be overrided as a point
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
   *  If the light was a point light, it will be overrided as a direction
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
                           float intensity = 1.0);

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
   *  @brief Set the scale of the normal texture
   *  @param scale
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setNormalTextureScale(float scale);

  /**
   * Toggles that control contributions from different components
   */
  struct PbrEquationScales {
    float directDiffuse = 1.0f;
    float directSpecular = 1.0f;
    float iblDiffuse = 1.0f;
    float iblSpecular = 1.0f;
  };

  /**
   *  @brief Set the scales for differenct components in the pbr equation
   *  @param scales
   *  @return Reference to self (for method chaining)
   */
  PbrShader& setPbrEquationScales(const PbrEquationScales& scales);

  enum class PbrDebugDisplay : uint8_t {
    None = 0,
    DirectDiffuse = 1,
    DirectSpecular = 2,
    IblDiffuse = 3,
    IblSpecular = 4,
    Normal = 5,
    Shadow0 = 6,
  };
  /**
   *@brief debug display visualization
   */
  PbrShader& setDebugDisplay(PbrDebugDisplay index);

 protected:
  Flags flags_;
  unsigned int lightCount_;

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

  int cameraWorldPosUniform_ = ID_UNDEFINED;
  int prefilteredMapMipLevelsUniform_ = ID_UNDEFINED;

  // scales
  int componentScalesUniform_ = ID_UNDEFINED;

  // pbr debug info
  int pbrDebugDisplayUniform_ = ID_UNDEFINED;

  /** @brief return true if direct lights or image based lighting is enabled. */
  inline bool lightingIsEnabled() const;
};

CORRADE_ENUMSET_OPERATORS(PbrShader::Flags)

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRSHADER_H_
