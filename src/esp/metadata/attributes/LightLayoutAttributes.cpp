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

LightInstanceAttributes::LightInstanceAttributes(const std::string& handle)
    : AbstractAttributes("LightInstanceAttributes", handle) {
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
    : AbstractAttributes("LightLayoutAttributes", handle) {}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
