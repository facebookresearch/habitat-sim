// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Shaders/Phong.h>

#include "esp/gfx/Drawable.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

class GenericDrawable : public Drawable {
 public:
  //! Create a GenericDrawable for the given object using shader and mesh.
  //! Adds drawable to given group and uses provided texture, and
  //! color for textured buffer and color shader output respectively
  explicit GenericDrawable(scene::SceneNode& node,
                           Magnum::GL::Mesh& mesh,
                           ShaderManager& shaderManager,
                           const Magnum::ResourceKey& lightSetup,
                           const Magnum::ResourceKey& materialData,
                           DrawableGroup* group = nullptr);

  void setLightSetup(const Magnum::ResourceKey& lightSetup) override;
  static constexpr const char* SHADER_KEY_TEMPLATE = "Phong-lights={}-flags={}";

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  void updateShader();

  Magnum::ResourceKey getShaderKey(Magnum::UnsignedInt lightCount,
                                   Magnum::Shaders::Phong::Flags flags) const;

  Magnum::GL::Texture2D* texture_;
  Magnum::Color4 color_;

  // shader parameters
  ShaderManager& shaderManager_;
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, Magnum::Shaders::Phong>
      shader_;
  Magnum::Resource<MaterialData, PhongMaterialData> materialData_;
  Magnum::Resource<LightSetup> lightSetup_;
};

}  // namespace gfx
}  // namespace esp
