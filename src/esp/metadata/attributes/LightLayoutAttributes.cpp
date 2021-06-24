// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightLayoutAttributes.h"
using Magnum::Math::Literals::operator""_radf;
using Magnum::Math::Literals::operator""_degf;

namespace esp {
namespace metadata {
namespace attributes {
const std::map<std::string, esp::gfx::LightType>
    LightInstanceAttributes::LightTypeNamesMap = {
        {"point", esp::gfx::LightType::Point},
        {"directional", esp::gfx::LightType::Directional},
        {"spot", esp::gfx::LightType::Spot}};

const std::map<std::string, esp::gfx::LightPositionModel>
    LightInstanceAttributes::LightPositionNamesMap = {
        {"global", esp::gfx::LightPositionModel::Global},
        {"camera", esp::gfx::LightPositionModel::Camera},
        {"object", esp::gfx::LightPositionModel::Object}};

int LightInstanceAttributes::_count{0};
LightInstanceAttributes::LightInstanceAttributes(const std::string& handle)
    : AbstractAttributes("LightInstanceAttributes", handle) {
  setID(_count++);
  setPosition({0.0, 0.0, 0.0});
  setDirection({0.0, -1.0, 0.0});
  setColor({1.0, 1.0, 1.0});
  setIntensity(1.0);
  setType(static_cast<int>(esp::gfx::LightType::Point));
  setPositionModel(static_cast<int>(esp::gfx::LightPositionModel::Global));
  // ignored for all but spot lights
  setInnerConeAngle(0.0_radf);
  setOuterConeAngle(90.0_degf);
}  // ctor

LightLayoutAttributes::LightLayoutAttributes(const std::string& handle)
    : AbstractAttributes("LightLayoutAttributes", handle) {
  // set default scaling for positive and negative intensities to 1.0
  setPositiveIntensityScale(1.0);
  setNegativeIntensityScale(1.0);
}

std::string LightLayoutAttributes::getObjectInfoInternal() const {
  std::string res = "\n";
  int iter = 0;
  for (const auto& lightInst : lightInstances_) {
    if (iter == 0) {
      iter++;
      res.append(1, ',')
          .append(lightInst.second->getObjectInfoHeader())
          .append(1, '\n');
    }
    res.append(1, ',')
        .append(lightInst.second->getObjectInfo())
        .append(1, '\n');
  }
  return res;
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
