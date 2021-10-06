// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributes.h"
using Magnum::Math::Literals::operator""_radf;
using Magnum::Math::Literals::operator""_degf;

namespace esp {
namespace metadata {
namespace attributes {
LightInstanceAttributes::LightInstanceAttributes(const std::string& handle)
    : AbstractAttributes("LightInstanceAttributes", handle) {
  setPosition({0.0, 0.0, 0.0});
  setDirection({0.0, -1.0, 0.0});
  setColor({1.0, 1.0, 1.0});
  setIntensity(1.0);
  setType(getLightTypeName(gfx::LightType::Point));
  setPositionModel(getLightPositionModelName(gfx::LightPositionModel::Global));
  // ignored for all but spot lights
  setInnerConeAngle(0.0_radf);
  setOuterConeAngle(90.0_degf);
}  // ctor

LightLayoutAttributes::LightLayoutAttributes(const std::string& handle)
    : AbstractAttributes("LightLayoutAttributes", handle) {
  // set default scaling for positive and negative intensities to 1.0
  setPositiveIntensityScale(1.0);
  setNegativeIntensityScale(1.0);
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
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))),
      availableLightIDs_(std::move(otr.availableLightIDs_)) {
  lightInstConfig_ = editSubconfig<Configuration>("lights");
}

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
  this->AbstractAttributes::operator=(
      std::move(static_cast<AbstractAttributes>(otr)));
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
