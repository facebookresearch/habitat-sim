// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBRDRAWABLE_H_
#define ESP_GFX_PBRDRAWABLE_H_

#include <Corrade/Containers/Optional.h>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/PbrImageBasedLighting.h"
#include "esp/gfx/PbrShader.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/gfx/ShadowMapManager.h"
namespace esp {
namespace gfx {

class PbrDrawable : public Drawable {
 public:
  /**
   * This cache holds all the material quantities and attributes from the
   * Magnum MaterialData to speed up access in draw.
   */
  struct PBRMaterialCache {
    Mn::Color4 baseColor{1.0f};
    float roughness = 1.0f;
    float metalness = 1.0f;
    Mn::Color3 emissiveColor{};
    Mn::Matrix3 textureMatrix{};

    Mn::GL::Texture2D* baseColorTexture = nullptr;

    bool hasAnyMetallicRoughnessTexture = false;

    Mn::GL::Texture2D* useMetallicRoughnessTexture = nullptr;
    /**
     * Currently we only support a single NoneMetalnessRoughness texture for
     * both metalness and roughness.  Separate textures will be stored, but only
     * the useMetallicRoughnessTexture should be sent to the shader
     */
    Mn::GL::Texture2D* noneRoughnessMetallicTexture = nullptr;
    Mn::GL::Texture2D* roughnessTexture = nullptr;
    Mn::GL::Texture2D* metallicTexture = nullptr;

    Mn::GL::Texture2D* normalTexture = nullptr;
    Mn::GL::Texture2D* emissiveTexture = nullptr;
  };

  /**
   * @brief Constructor, to create a PbrDrawable for the given object using
   * shader and mesh. Adds drawable to given group and uses provided texture,
   * and color for textured buffer and color shader output respectively
   */
  explicit PbrDrawable(scene::SceneNode& node,
                       Mn::GL::Mesh* mesh,
                       gfx::Drawable::Flags& meshAttributeFlags,
                       ShaderManager& shaderManager,
                       const Mn::ResourceKey& lightSetupKey,
                       const Mn::ResourceKey& materialDataKey,
                       DrawableGroup* group = nullptr,
                       PbrImageBasedLighting* pbrIbl = nullptr);

  /**
   *  @brief Set the light info
   *  @param lightSetupKey the key value for the light resource
   */
  void setLightSetup(const Mn::ResourceKey& lightSetupKey) override;

  /**
   * @brief Set the shadow map info
   * @param[in] manager, stores the shadow maps
   * @param[in] keys, keys to retrieve the shadow maps
   * @param[in] shadowFlag, can only be either ShadowsPCF or ShadowsVSM
   */
  void setShadowData(ShadowMapManager& manager,
                     ShadowMapKeys& keys,
                     PbrShader::Flag shadowFlag);

  static constexpr const char* SHADER_KEY_TEMPLATE = "PBR-lights={}-flags={}";

  /**
   * Set or change this drawable's @ref Magnum::Trade::MaterialData values from passed material.
   * This is only pertinent for material-equipped drawables.
   * @param material
   */
  void setMaterialValues(
      const Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData>&
          material) override {
    setMaterialValuesInternal(material);
  }

 private:
  /**
   * @brief Internal implementation of material setting, so that it can be
   * called from constructor without virtual dispatch issues
   */
  void setMaterialValuesInternal(
      const Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData>&
          material);

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
   *  @brief Update the shader so it can correcly handle the current material,
   *         light setup
   *  @return Reference to self (for method chaining)
   */
  PbrDrawable& updateShader();

  /**
   *  @brief Update every light's color, intensity, range etc.
   *  @return Reference to self (for method chaining)
   */
  PbrDrawable& updateShaderLightParameters();

  /**
   *  @brief Update light direction (or position) in *camera* space to the
   * shader
   *  @param transformationMatrix describes a tansformation from object
   * (model) space to camera space
   *  @param camera the camera, which views and renders the world
   *  @return Reference to self (for method chaining)
   */
  PbrDrawable& updateShaderLightDirectionParameters(
      const Mn::Matrix4& transformationMatrix,
      Mn::SceneGraph::Camera3D& camera);

  /**
   * @brief get the key for the shader
   * @param lightCount the number of the lights;
   * @param flags flags that defines the shader features
   */
  Mn::ResourceKey getShaderKey(Mn::UnsignedInt lightCount,
                               PbrShader::Flags flags) const;

  // shader parameters
  PbrShader::Flags flags_;
  ShaderManager& shaderManager_;
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrShader> shader_;
  Mn::Resource<LightSetup> lightSetup_;
  PbrImageBasedLighting* pbrIbl_ = nullptr;

  /**
   * Local cache of material quantities to speed up access in draw
   */
  PBRMaterialCache matCache{};
  /**
   * Creation attributes of this drawable
   */
  const gfx::Drawable::Flags meshAttributeFlags_;
  /**
   * Material to use to render this PBR drawawble
   */
  Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData> materialData_;
  ShadowMapManager* shadowMapManger_ = nullptr;
  ShadowMapKeys* shadowMapKeys_ = nullptr;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRDRAWABLE_H_
