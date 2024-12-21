// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CoordinateFrame.h"

#include "esp/geo/Geo.h"

#include <Magnum/Math/Matrix4.h>

namespace esp {
namespace geo {

CoordinateFrame::CoordinateFrame(const vec3f& up /* = ESP_UP */,
                                 const vec3f& front /* = ESP_FRONT */,
                                 const vec3f& origin /* = vec3f(0, 0, 0) */)
    : up_(up), front_(front), origin_(origin) {
  CORRADE_INTERNAL_ASSERT(up_.isOrthogonal(front_));
}

CoordinateFrame::CoordinateFrame(const quatf& rotation,
                                 const vec3f& origin /* = vec3f(0, 0, 0) */)
    : CoordinateFrame(rotation * ESP_UP, rotation * ESP_FRONT, origin) {}

quatf CoordinateFrame::rotationWorldToFrame() const {
  auto front = (Mn::Vector3(front_)).normalized();
  auto right = Mn::Math::cross(Mn::Vector3(up_), front).normalized();
  auto realUp = Mn::Math::cross(front, right);
  auto myMat = Mn::Matrix4::from({front, right, realUp}, Mn::Vector3());
  return Magnum::EigenIntegration::cast<quatf>(
      Mn::Quaternion::fromMatrix(myMat.rotation()));
}

quatf CoordinateFrame::rotationFrameToWorld() const {
  return rotationWorldToFrame().inverse();
}

std::string CoordinateFrame::toString() const {
  std::stringstream ss;
  ss << "{\"up\":" << up() << ",\"front\":" << front()
     << ",\"origin\":" << origin() << "}";
  return ss.str();
}

bool operator==(const CoordinateFrame& a, const CoordinateFrame& b) {
  return a.up().isApprox(b.up()) && a.front().isApprox(b.front()) &&
         a.origin().isApprox(b.origin());
}
bool operator!=(const CoordinateFrame& a, const CoordinateFrame& b) {
  return !(a == b);
}

}  // namespace geo
}  // namespace esp
