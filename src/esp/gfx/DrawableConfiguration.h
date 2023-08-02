// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DRAWABLECONFIGURATION_H_
#define ESP_GFX_DRAWABLECONFIGURATION_H_

#include <Magnum/Resource.h>
#include "esp/gfx/PbrIBLHelper.h"
#include "esp/gfx/SkinData.h"
#include "esp/metadata/attributes/PbrShaderAttributes.h"
namespace esp {
namespace gfx {
class DrawableGroup;

/**
 * This class will hold configuration values and utilities for Drawables and the
 * shaders that they own.
 */
class DrawableConfiguration {
 public:
  DrawableConfiguration(
      const Mn::ResourceKey& lightSetupKey,
      const Mn::ResourceKey& materialKey,
      esp::metadata::attributes::ObjectInstanceShaderType materialDataType,
      DrawableGroup* group,
      const std::shared_ptr<gfx::InstanceSkinData>& skinData,
      const std::shared_ptr<PbrIBLHelper>& pbrIblData,
      const std::shared_ptr<metadata::attributes::PbrShaderAttributes>&
          pbrShaderConfig);

  std::shared_ptr<InstanceSkinData> getSkinData() const { return skinData_; }
  std::shared_ptr<PbrIBLHelper> getPbrIblData() const { return pbrIblData_; }
  void setPbrIblData(std::shared_ptr<PbrIBLHelper> pbrIblData) {
    pbrIblData_ = std::move(pbrIblData);
  }

  std::shared_ptr<metadata::attributes::PbrShaderAttributes>
  getPbrShaderConfig() const {
    return pbrShaderConfig_;
  }

  void setPbrShaderConfig(
      std::shared_ptr<metadata::attributes::PbrShaderAttributes>
          pbrShaderConfig) {
    pbrShaderConfig_ = std::move(pbrShaderConfig);
  }

  /**
   * Lighting key for this drawable
   */
  const Magnum::ResourceKey lightSetupKey_;

  /**
   * Lighting key for this drawable
   */
  const Magnum::ResourceKey materialDataKey_;

  /**
   * Shader being used.
   */
  const esp::metadata::attributes::ObjectInstanceShaderType materialDataType_;

  /**
   * Optional @ref DrawableGroup with which to render the @ref
   * gfx::Drawable.
   */
  esp::gfx::DrawableGroup* group_ = nullptr;

 protected:
  /**
   * Skin data for use with this drawable, if appropriate.
   */
  std::shared_ptr<InstanceSkinData> skinData_ = nullptr;

  /**
   * Configuration values for PBR shader to be used by this object. Only used
   * for PbrDrawables, ignored otherwise.
   */
  std::shared_ptr<PbrIBLHelper> pbrIblData_ = nullptr;

  /**
   * The attributes configuration to configure the PBR shader
   */
  std::shared_ptr<metadata::attributes::PbrShaderAttributes> pbrShaderConfig_ =
      nullptr;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DRAWABLECONFIGURATION_H_
