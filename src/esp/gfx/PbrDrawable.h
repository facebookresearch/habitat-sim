// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBRDRAWABLE_H_
#define ESP_GFX_PBRDRAWABLE_H_

#include "esp/gfx/Drawable.h"
#include "esp/gfx/PbrShader.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

class PbrDrawable : public Drawable {
 public:
  /**
   * @brief Constructor, to create a PbrDrawable for the given object using
   * shader and mesh. Adds drawable to given group and uses provided texture,
   * and color for textured buffer and color shader output respectively
   */
  explicit PbrDrawable(scene::SceneNode& node,
                       Magnum::GL::Mesh& mesh,
                       gfx::Drawable::Flags& meshAttributeFlags,
                       ShaderManager& shaderManager,
                       const Magnum::ResourceKey& lightSetupKey,
                       const Magnum::ResourceKey& materialDataKey,
                       DrawableGroup* group = nullptr);

  /**
   *  @brief Set the light info
   *  @param lightSetupKey, the key value for the light resource
   *  @param color, the color of the light
   */
  void setLightSetup(const Magnum::ResourceKey& lightSetupkey) override;

  static constexpr const char* SHADER_KEY_TEMPLATE = "PBR-lights={}-flags={}";

 protected:
  /**
   * @brief overload draw function, see here for more details:
   * https://doc.magnum.graphics/magnum/classMagnum_1_1SceneGraph_1_1Drawable.html#aca0d0a219aa4d7712316de55d67f2134
   * @param transformationMatrix, the transformation of the object (to which the
   *        drawable is attached) relative to camera
   * @param camera, the camera that views and renders the world
   */
  void draw(const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& camera) override;

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
   *  @param transformationMatrix, describes a tansformation from object (model)
   *         space to camera space
   *  @param camera, the camera, which views and renders the world
   *  @return Reference to self (for method chaining)
   */
  PbrDrawable& updateShaderLightDirectionParameters(
      const Magnum::Matrix4& transformationMatrix,
      Magnum::SceneGraph::Camera3D& camera);

  /**
   * @brief get the key for the shader
   * @param lightCount, the number of the lights;
   * @param flags, flags that defines the shader features
   */
  Magnum::ResourceKey getShaderKey(Magnum::UnsignedInt lightCount,
                                   PbrShader::Flags flags) const;

  // shader parameters
  PbrShader::Flags flags_;
  ShaderManager& shaderManager_;
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, PbrShader> shader_;
  Magnum::Resource<MaterialData, PbrMaterialData> materialData_;
  Magnum::Resource<LightSetup> lightSetup_;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PBRDRAWABLE_H_
