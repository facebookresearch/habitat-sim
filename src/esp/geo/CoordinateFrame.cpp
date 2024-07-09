// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CoordinateFrame.h"
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Math/Quaternion.h>

#include "esp/core/Utility.h"
#include "esp/geo/Geo.h"

namespace esp {
namespace geo {

namespace Mn = Magnum;
CoordinateFrame::CoordinateFrame(
    const Mn::Vector3& up /* = ESP_UP */,
    const Mn::Vector3& front /* = ESP_FRONT */,
    const Mn::Vector3& origin /* = Mn::Vector3(0, 0, 0) */)
    : up_(up), front_(front), origin_(origin) {
  CORRADE_INTERNAL_ASSERT(std::abs(Mn::Math::dot(up_, front_)) <
                          Mn::Math::TypeTraits<float>::epsilon());
}

CoordinateFrame::CoordinateFrame(
    const Mn::Quaternion& rotation,
    const Mn::Vector3& origin /* = Mn::Vector3(0, 0, 0) */)
    : CoordinateFrame(rotation.transformVectorNormalized(ESP_UP),
                      rotation.transformVectorNormalized(ESP_FRONT),
                      origin) {}

Mn::Quaternion CoordinateFrame::rotationWorldToFrame() const {
  const Mn::Quaternion R_frameUp_worldUp =
      Mn::Quaternion::rotation(ESP_UP, up_);
  return (Mn::Quaternion::rotation(
              R_frameUp_worldUp.transformVectorNormalized(ESP_FRONT), front_) *
          R_frameUp_worldUp)
      .normalized();
}

Mn::Quaternion CoordinateFrame::rotationFrameToWorld() const {
  return rotationWorldToFrame().invertedNormalized();
}

std::string CoordinateFrame::toString() const {
  return Cr::Utility::formatString(
      "\"up\":[{},{},{}],\"front\":[{},{},{}],\"origin\":[{},{},{}]", up().x(),
      up().y(), up().z(), front().x(), front().y(), front().z(), origin().x(),
      origin().y(), origin().z());
}

bool operator==(const CoordinateFrame& a, const CoordinateFrame& b) {
  return a.up() == b.up() && a.front() == b.front() && a.origin() == b.origin();
}
bool operator!=(const CoordinateFrame& a, const CoordinateFrame& b) {
  return !(a == b);
}

}  // namespace geo
}  // namespace esp
