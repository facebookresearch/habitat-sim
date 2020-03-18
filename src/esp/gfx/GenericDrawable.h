// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/gfx/Drawable.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/gfx/shadows/PhongShadowReceiverShader.h"

namespace esp {
namespace gfx {

class GenericDrawable : public Drawable {
 public:
  //! Create a GenericDrawable for the given object using shader and mesh.
  //! Adds drawable to given group and uses provided texture, objectId, and
  //! color for textured, object id buffer and color shader output respectively
  explicit GenericDrawable(
      scene::SceneNode& node,
      Magnum::GL::Mesh& mesh,
      ShaderManager& shaderManager,
      const Magnum::ResourceKey& lightSetup,
      const Magnum::ResourceKey& materialData,
      DrawableGroup* group = nullptr,
      int objectId = ID_UNDEFINED,
      scene::SceneGraph::ShadowMapRegistry* shadowMapRegistry = nullptr,
      bool shadeFacesFacingAwayFromLight = false);

  void setLightSetup(const Magnum::ResourceKey& lightSetup) override;

  static constexpr const char* SHADER_KEY_TEMPLATE =
      "Phong-lights={}-flags={}-layers={}";

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  void updateShader();

  Magnum::ResourceKey getShaderKey(Magnum::UnsignedInt lightCount,
                                   PhongShadowReceiverShader::Flags flags,
                                   Magnum::UnsignedInt layerCount) const;

  Magnum::GL::Texture2D* texture_;
  int objectId_;
  Magnum::Color4 color_;

  // shader parameters
  ShaderManager& shaderManager_;
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, PhongShadowReceiverShader>
      shader_;
  Magnum::Resource<MaterialData, PhongMaterialData> materialData_;
  Magnum::Resource<LightSetup> lightSetup_;
  Magnum::Resource<scene::SceneGraph::LightSetupShadowMaps>
      lightSetupShadowMaps_;
  scene::SceneGraph::ShadowMapRegistry* shadowMapRegistry_ = nullptr;
  bool receivesShadow_;
  bool shadeFacesFacingAwayFromLight_;
};

}  // namespace gfx
}  // namespace esp
