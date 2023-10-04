// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBRDRAWABLE_H_
#define ESP_GFX_PBRDRAWABLE_H_

#include <Corrade/Containers/Optional.h>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/DrawableConfiguration.h"
#include "esp/gfx/PbrIBLHelper.h"
#include "esp/gfx/PbrShader.h"
#include "esp/gfx/ShaderManager.h"
namespace esp {
namespace gfx {

class PbrDrawable : public Drawable {
 public:
  /**
   * This cache holds all the material quantities and attributes from the
   * Magnum MaterialData to speed up access in draw.
   */
  struct PBRMaterialCache {
    ////////////////
    // Base layer
    Mn::Color4 baseColor{1.0f};
    float roughness = 1.0f;
    float metalness = 1.0f;
    Mn::Color3 emissiveColor{};
    Mn::Matrix3 textureMatrix{};

    Mn::GL::Texture2D* baseColorTexture = nullptr;

    /**
     * Currently we only support a single noneRoughnessMetallicTexture texture
     * for both metalness and roughness.
     */
    Mn::GL::Texture2D* noneRoughnessMetallicTexture = nullptr;

    Mn::GL::Texture2D* emissiveTexture = nullptr;

    Mn::GL::Texture2D* normalTexture = nullptr;

    float normalTextureScale = 1.0f;

    ////////////////
    // ClearCoat layer

    /**
     * Structure holding clearcoat layer values
     */
    struct ClearCoat {
      /**
       * Clear coat layer intensity
       */
      float factor = 0.0f;

      /**
       * Texture defining clearcoat intensity in R channel. Multiplied by scalar
       * if both are present.
       */
      Mn::GL::Texture2D* texture = nullptr;

      /**
       * Clearcoat layer roughness.
       */
      float roughnessFactor = 0.0f;

      /**
       * Texture describing clear coat roughness in G channel. Multiplied by
       * scalar if both are present.
       */
      Mn::GL::Texture2D* roughnessTexture = nullptr;

      /**
       * Scale value for clearcoat normal texture
       */
      float normalTextureScale = 1.0f;

      /**
       * Clearcoat Normal map texture, in RGB channels.
       */
      Mn::GL::Texture2D* normalTexture = nullptr;

    } clearCoat;

    ////////////////
    // KHR_materials_ior

    /**
     * Index of refraction of material. Generally between 1-2, although values
     * higher than 2 are possible. Defaults to 1.5.
     *
     * dielectricSpecular = ((ior - 1)/(ior + 1))^2
     * default ior value evaluates to dielectricSpecular = 0.04
     */
    float ior_Index = 1.5;

    ////////////////
    // KHR_materials_specular layer

    /**
     * Structure holding specular layer values
     */
    struct SpecularLayer {
      /**
       * The strength of the specular reflection.
       */
      float factor = 1.0f;

      /**
       * A texture that defines the strength of the specular reflection, stored
       * in the alpha (A) channel. This will be multiplied by specularFactor.
       */
      Mn::GL::Texture2D* texture = nullptr;

      /**
       * The F0 color of the specular reflection (linear RGB).
       */
      Mn::Color3 colorFactor{1.0f};

      /**
       * A texture that defines the F0 color of the specular reflection,
       * stored in the RGB channels and encoded in sRGB. This texture will be
       * multiplied by specularColorFactor.
       */
      Mn::GL::Texture2D* colorTexture = nullptr;

    } specularLayer;

    ///////////////
    // KHR_materials_anisotropy layer
    /**
     * Structure holding anisotropy layer values
     */
    struct AnisotropyLayer {
      /**
       * The anisotropy strength. When anisotropyTexture is present, this value
       * is multiplied by the blue channel.
       */
      float factor = 0.0f;

      /**
       * [cos(rotation), sin(rotation)] : Built from the rotation of the
       * anisotropy in tangent, bitangent space, measured in radians
       * counter-clockwise from the tangent. When anisotropyTexture is present,
       * anisotropyRotation provides additional rotation to the vectors in the
       * texture.
       */
      Mn::Vector2 direction{1.0f, 0.0f};

      /**
       * A texture that defines the strength and orientation of the anisotropy
       * of the material. Red and green channels represent the anisotropy
       * direction in [-1, 1] tangent, bitangent space, to be rotated by
       * anisotropyRotation. The blue channel contains strength as [0, 1] to be
       * multiplied by anisotropyStrength
       */
      Mn::GL::Texture2D* texture = nullptr;
    } anisotropyLayer;

    ////////////////
    // KHR_materials_transmission

    /**
     * Structure holding transmission layer values
     */
    struct TransmissionLayer {
      /**
       * The base percentage of light that is transmitted through the surface.
       */
      float factor = 0.0f;
      /**
       * A texture that defines the transmission percentage of the surface,
       * stored in the R channel. This will be multiplied by transmissionFactor.
       */
      Mn::GL::Texture2D* texture = nullptr;
    } transmissionLayer;

    ////////////////
    // KHR_materials_volume

    /**
     * Structure holding volume-layer values
     */
    struct VolumeLayer {
      /**
       * The thickness of the volume beneath the surface. The value is given in
       * the coordinate space of the mesh. If the value is 0 the material is
       * thin-walled. Otherwise the material is a volume boundary. The
       * doubleSided property has no effect on volume boundaries. Range is [0,
       * +inf).
       */
      float thicknessFactor = 0.0f;

      /**
       * A texture that defines the thickness, stored in the G channel. This
       * will be multiplied by thicknessFactor. Range is [0, 1]
       */
      Mn::GL::Texture2D* thicknessTexture = nullptr;

      /**
       * Density of the medium given as the average distance that light travels
       * in the medium before interacting with a particle. The value is given in
       * world space. Range is (0, +inf). Default is inf (treat -1).
       */
      float attenuationDist = -1.0f;

      /**
       * The color that white light turns into due to absorption when reaching
       * the attenuation distance.
       */

      Mn::Color3 attenuationColor{1.0f};

    } volumeLayer;
  };  // struct PBRMaterialCache

  /**
   * This struct holds configuration values to be passed as uniforms to the PBR
   * shader. These values are beyond what the material data can provide, are
   * user provided, and can control, for example, light balance between direct
   * and IBL lighting, or the intensity of the direct light.
   */
  struct PBRShaderConfig {
    /**
     * Control the direct lighting intensity.
     */
    float directLightingIntensity = 1.0f;
    /**
     * Controls the exposure level for the tonemapping function. Only used if
     * tonemapping is enabled.
     */
    float tonemapExposure = 4.5f;
    /**
     * Controls the gamma value used in the sRGB<->linear approximation
     * calculations. Only used if sRGB remapping is enabled.
     */
    Magnum::Vector3 gamma = {2.2f, 2.2f, 2.2f};

    /**
     * Scales the contributions for each of the 4 given values - direct lighting
     * diffuse and specular and ibl diffuse and specular. Only used both direct
     * lighting and IBL are enabled, ignored otherwise.
     */
    PbrShader::PbrEquationScales eqScales{0.5f,   // directDiffuse
                                          0.5f,   // directSpecular
                                          0.5f,   // iblDiffuse
                                          0.5f};  // iblSpecular
  };                                              // struct PBRShaderConfig

  /**
   * @brief Constructor, to create a PbrDrawable for the given object using
   * shader and mesh. Adds drawable to given group and uses provided texture,
   * and color for textured buffer and color shader output respectively
   */
  explicit PbrDrawable(scene::SceneNode& node,
                       Mn::GL::Mesh* mesh,
                       gfx::Drawable::Flags& meshAttributeFlags,
                       ShaderManager& shaderManager,
                       DrawableConfiguration& cfg);

  /**
   *  @brief Set the light info
   *  @param lightSetupKey the key value for the light resource
   */
  void setLightSetup(const Mn::ResourceKey& lightSetupKey) override;

  /**
   * @brief Set the @ref ref::metadata::attributes::PbrShaderAttributes,
   * which controls the program flow and functionality of the shader. NOTE :
   * changing boolean values will require the shader to be rebuilt before
   * rendering can occur.
   */
  void setShaderAttributesValues(
      const std::shared_ptr<metadata::attributes::PbrShaderAttributes>&
          _pbrShaderConfig);

 private:
  /**
   * @brief Internal implementation of material setting, so that it can be
   * called from constructor without virtual dispatch issues
   */
  void setMaterialValuesInternal(
      const Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData>&
          material,
      bool reset) override;

 protected:
  /**
   * @brief overload draw function, see here for more details:
   * https://doc.magnum.graphics/magnum/classMagnum_1_1SceneGraph_1_1Drawable.html#aca0d0a219aa4d7712316de55d67f2134
   * @param transformationMatrix the transformation of the object (to which
   * the drawable is attached) relative to camera
   * @param camera the camera that views and renders the world
   */
  void draw(const Mn::Matrix4& transformationMatrix,
            Mn::SceneGraph::Camera3D& camera) override;

  /**
   *  @brief Update the shader so it can correctly handle the current material,
   *         light setup, etc.
   */
  void updateShader();

  // shader parameters
  PbrShader::Flags flags_;
  ShaderManager& shaderManager_;
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrShader> shader_;
  std::shared_ptr<PbrIBLHelper> pbrIbl_ = nullptr;

  /**
   * Local cache of material quantities to speed up access in draw
   */
  PBRMaterialCache matCache{};

  /**
   * Local cache of shader control values
   */
  PBRShaderConfig shaderConfig_{};
  /**
   * Creation attributes of this drawable
   */
  const gfx::Drawable::Flags meshAttributeFlags_;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRDRAWABLE_H_
