// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_GENERICDRAWABLE_H_
#define ESP_GFX_GENERICDRAWABLE_H_

#include <Magnum/Shaders/PhongGL.h>
#include <memory>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {
struct InstanceSkinData;

class GenericDrawable : public Drawable {
 public:
  //! Create a GenericDrawable for the given object using shader and mesh.
  //! Adds drawable to given group and uses provided texture, and
  //! color for textured buffer and color shader output respectively
  explicit GenericDrawable(
      scene::SceneNode& node,
      Mn::GL::Mesh* mesh,
      Drawable::Flags& meshAttributeFlags,
      ShaderManager& shaderManager,
      const Mn::ResourceKey& lightSetupKey,
      const Mn::ResourceKey& materialDataKey,
      DrawableGroup* group = nullptr,
      const std::shared_ptr<InstanceSkinData>& skinData = nullptr);

  void setLightSetup(const Mn::ResourceKey& lightSetupKey) override;
  static constexpr const char* SHADER_KEY_TEMPLATE =
      "Phong-lights={}-flags={}-joints={}";

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
  ShaderManager& shaderManager_;
  Mn::Resource<Mn::GL::AbstractShaderProgram, Mn::Shaders::PhongGL> shader_;
  Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::PhongMaterialData>
      materialData_;
  Mn::Resource<LightSetup> lightSetup_;
  Mn::Shaders::PhongGL::Flags flags_;
  std::shared_ptr<InstanceSkinData> skinData_;
  Cr::Containers::Array<Mn::Matrix4> jointTransformations_;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_GENERICDRAWABLE_H_
