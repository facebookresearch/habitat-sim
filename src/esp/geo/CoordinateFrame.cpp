// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CoordinateFrame.h"

#include "esp/geo/geo.h"
#include "esp/io/json.h"

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

CoordinateFrame::CoordinateFrame(const std::string& json) {
  fromJson(json);
}

quatf CoordinateFrame::rotationWorldToFrame() const {
  const quatf R_frameUp_worldUp = quatf::FromTwoVectors(ESP_UP, up_);
  return quatf::FromTwoVectors(R_frameUp_worldUp * ESP_FRONT, front_) *
         R_frameUp_worldUp;
}

quatf CoordinateFrame::rotationFrameToWorld() const {
  return rotationWorldToFrame().inverse();
}

std::string CoordinateFrame::toJson() const {
  std::stringstream ss;
  ss << "{\"up\":" << up() << ",\"front\":" << front()
     << ",\"origin\":" << origin() << "}";
  return ss.str();
}

void CoordinateFrame::fromJson(const std::string& jsonString) {
  const auto json = io::parseJsonString(jsonString);
  const auto up = json["up"].GetArray();
  const auto front = json["front"].GetArray();
  const auto origin = json["origin"].GetArray();
  for (int i = 0; i < 3; ++i) {
    up_[i] = up[i].GetFloat();
    front_[i] = front[i].GetFloat();
    origin_[i] = origin[i].GetFloat();
  }
  CORRADE_INTERNAL_ASSERT(up_.isOrthogonal(front_));
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
