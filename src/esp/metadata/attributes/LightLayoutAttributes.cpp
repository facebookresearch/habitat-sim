// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributes.h"

#include <utility>
using Magnum::Math::Literals::operator""_radf;
using Magnum::Math::Literals::operator""_degf;

namespace esp {
namespace metadata {
namespace attributes {
LightInstanceAttributes::LightInstanceAttributes(const std::string& handle)
    : AbstractAttributes("LightInstanceAttributes", handle) {
  init("position", Mn::Vector3{0.0, 0.0, 0.0});
  init("direction", Mn::Vector3{0.0, -1.0, 0.0});
  init("color", Mn::Vector3{1.0, 1.0, 1.0});
  init("intensity", 1.0);
  initTranslated("type", getLightTypeName(gfx::LightType::Point));
  initTranslated("position_model",
                 getLightPositionModelName(gfx::LightPositionModel::Global));
  // ignored for all but spot lights
  std::shared_ptr<Configuration> spotGrp = editSubconfig<Configuration>("spot");
  spotGrp->init("innerConeAngle", 0.0_radf);
  spotGrp->init("outerConeAngle", Magnum::Rad{90.0_degf});
}  // ctor

void LightInstanceAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  gfx::LightType lightType = getType();
  if (lightType == gfx::LightType::Directional) {
    writeValueToJson("direction", jsonObj, allocator);
  } else {
    writeValueToJson("position", jsonObj, allocator);
  };
  writeValueToJson("type", jsonObj, allocator);
  writeValueToJson("color", jsonObj, allocator);
  writeValueToJson("intensity", jsonObj, allocator);
  writeValueToJson("position_model", jsonObj, allocator);
}  // LightInstanceAttributes::writeValuesToJson

LightLayoutAttributes::LightLayoutAttributes(const std::string& handle)
    : AbstractAttributes("LightLayoutAttributes", handle) {
  // set default scaling for positive and negative intensities to 1.0
  init("positive_intensity_scale", 1.0);
  init("negative_intensity_scale", 1.0);
  // get ref to internal subconfig for light instances
  lightInstConfig_ = editSubconfig<Configuration>("lights");
}
LightLayoutAttributes::LightLayoutAttributes(const LightLayoutAttributes& otr)
    : AbstractAttributes(otr), availableLightIDs_(otr.availableLightIDs_) {
  lightInstConfig_ = editSubconfig<Configuration>("lights");

  copySubconfigIntoMe<LightInstanceAttributes>(otr.lightInstConfig_,
                                               lightInstConfig_);
}
LightLayoutAttributes::LightLayoutAttributes(
    LightLayoutAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes&&>(otr))),
      availableLightIDs_(std::move(otr.availableLightIDs_)) {
  lightInstConfig_ = editSubconfig<Configuration>("lights");
}

void LightLayoutAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("positive_intensity_scale", jsonObj, allocator);
  writeValueToJson("negative_intensity_scale", jsonObj, allocator);
}  // LightLayoutAttributes::writeValuesToJson

LightLayoutAttributes& LightLayoutAttributes::operator=(
    const LightLayoutAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
    // point to our own light instance config and available ids
    availableLightIDs_ = otr.availableLightIDs_;
    lightInstConfig_ = editSubconfig<Configuration>("lights");
    copySubconfigIntoMe<LightInstanceAttributes>(otr.lightInstConfig_,
                                                 lightInstConfig_);
  }
  return *this;
}
LightLayoutAttributes& LightLayoutAttributes::operator=(
    LightLayoutAttributes&& otr) noexcept {
  // point to our own light instance config and available ids
  availableLightIDs_ = std::move(otr.availableLightIDs_);
  this->AbstractAttributes::operator=(static_cast<AbstractAttributes&&>(otr));
  lightInstConfig_ = editSubconfig<Configuration>("lights");
  return *this;
}

std::string LightLayoutAttributes::getObjectInfoInternal() const {
  std::string res = "\n";
  int iter = 0;
  auto lightInstances = getLightInstances();
  for (const auto& lightInst : lightInstances) {
    if (iter == 0) {
      ++iter;
      Cr::Utility::formatInto(res, res.size(), ",{}\n",
                              lightInst->getObjectInfoHeader());
    }
    Cr::Utility::formatInto(res, res.size(), ",{}\n",
                            lightInst->getObjectInfo());
  }
  return res;
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
