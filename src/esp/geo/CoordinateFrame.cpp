// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CoordinateFrame.h"


#include <utility>


#include "esp/geo/geo.h"
#include "esp/io/json.h"

namespace esp {
namespace geo {

CoordinateFrame::CoordinateFrame(vec3f  up /* = ESP_UP */,
                                 vec3f  front /* = ESP_FRONT */,
                                 vec3f  origin /* = vec3f(0, 0, 0) */)
    : up_(std::move(up)), front_(std::move(front)), origin_(std::move(origin)) {
  ASSERT(up_.isOrthogonal(front_));
}

CoordinateFrame::CoordinateFrame(const quatf& rotation,
                                 const vec3f& origin /* = vec3f(0, 0, 0) */)
    : CoordinateFrame(rotation * ESP_UP, rotation * ESP_FRONT, origin) {}

CoordinateFrame::CoordinateFrame(const std::string& json) {
  fromJson(json);
}

auto CoordinateFrame::rotationWorldToFrame() const -> quatf {
  const quatf R_frameUp_worldUp = quatf::FromTwoVectors(ESP_UP, up_);
  return quatf::FromTwoVectors(R_frameUp_worldUp * ESP_FRONT, front_) *
         R_frameUp_worldUp;
}

auto CoordinateFrame::rotationFrameToWorld() const -> quatf {
  return rotationWorldToFrame().inverse();
}

auto CoordinateFrame::toJson() const -> std::string {
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
  ASSERT(up_.isOrthogonal(front_));
}

auto operator==(const CoordinateFrame& a, const CoordinateFrame& b) -> bool {
  return a.up().isApprox(b.up()) && a.front().isApprox(b.front()) &&
         a.origin().isApprox(b.origin());
}
auto operator!=(const CoordinateFrame& a, const CoordinateFrame& b) -> bool {
  return !(a == b);
}

}  // namespace geo
}  // namespace esp
