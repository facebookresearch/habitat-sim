// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_GENERICDRAWABLE_H_
#define ESP_GFX_GENERICDRAWABLE_H_

#include <Magnum/Shaders/PhongGL.h>
#include <memory>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/DrawableConfiguration.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {
struct InstanceSkinData;

class GenericDrawable : public Drawable {
 public:
  /**
   * This config holds all the material quantities and attributes from the
   * Magnum MaterialData to speed up access in draw.
   */
  struct GenericMaterialCache {
    Mn::Color4 ambientColor{};
    Mn::Color4 diffuseColor{};
    Mn::Color4 specularColor{};
    float shininess = 80.0f;
    Mn::Matrix3 textureMatrix{};
    Mn::GL::Texture2D* ambientTexture = nullptr;
    Mn::GL::Texture2D* diffuseTexture = nullptr;
    Mn::GL::Texture2D* specularTexture = nullptr;
    Mn::GL::Texture2D* normalTexture = nullptr;
    Mn::GL::Texture2D* objectIdTexture = nullptr;

  };  // GenericMaterialCache

  //! Create a GenericDrawable for the given object using shader and mesh.
  //! Adds drawable to given group and uses provided texture, and
  //! color for textured buffer and color shader output respectively
  explicit GenericDrawable(scene::SceneNode& node,
                           Mn::GL::Mesh* mesh,
                           Drawable::Flags& meshAttributeFlags,
                           ShaderManager& shaderManager,
                           DrawableConfiguration& cfg);

  void setLightSetup(const Mn::ResourceKey& lightSetupKey) override;
  static constexpr const char* SHADER_KEY_TEMPLATE =
      "Phong-lights={}-flags={}-joints={}";

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
  void draw(const Mn::Matrix4& transformationMatrix,
            Mn::SceneGraph::Camera3D& camera) override;

  void updateShader();
  void updateShaderLightingParameters(const Mn::Matrix4& transformationMatrix,
                                      Mn::SceneGraph::Camera3D& camera);

  Mn::ResourceKey getShaderKey(Mn::UnsignedInt lightCount,
                               Mn::Shaders::PhongGL::Flags flags,
                               Mn::UnsignedInt jointCount) const;

  // shader parameters

  Mn::Shaders::PhongGL::Flags flags_;
  ShaderManager& shaderManager_;
  Mn::Resource<Mn::GL::AbstractShaderProgram, Mn::Shaders::PhongGL> shader_;
  Mn::Resource<LightSetup> lightSetup_;
  std::shared_ptr<InstanceSkinData> skinData_;
  Cr::Containers::Array<Mn::Matrix4> jointTransformations_;

  /**
   * Local cache of material quantities to speed up access in draw
   */
  GenericMaterialCache matCache{};
  /**
   * Creation attributes of this drawable
   */
  const gfx::Drawable::Flags meshAttributeFlags_;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_GENERICDRAWABLE_H_
