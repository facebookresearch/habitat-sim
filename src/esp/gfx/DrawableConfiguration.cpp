// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DrawableConfiguration.h"
#include "esp/gfx/PbrIBLHelper.h"
#include "esp/gfx/SkinData.h"
#include "esp/metadata/attributes/AttributesEnumMaps.h"
namespace esp {
namespace gfx {

DrawableConfiguration::DrawableConfiguration(
    const Mn::ResourceKey& lightSetupKey,
    const Mn::ResourceKey& materialDataKey,
    esp::metadata::attributes::ObjectInstanceShaderType materialDataType,
    DrawableGroup* group,
    const std::shared_ptr<gfx::InstanceSkinData>& skinData,
    const std::shared_ptr<PbrIBLHelper>& pbrIblData,
    const std::shared_ptr<metadata::attributes::PbrShaderAttributes>&
        pbrShaderConfig)
    : lightSetupKey_{lightSetupKey},
      materialDataKey_{materialDataKey},
      materialDataType_{materialDataType},
      group_{group},
      skinData_{skinData},
      pbrIblData_{pbrIblData},
      pbrShaderConfig_{pbrShaderConfig} {}

}  // namespace gfx
}  // namespace esp
