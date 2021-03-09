// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_COORDINATEFRAME_H_
#define ESP_GEO_COORDINATEFRAME_H_

#include "esp/core/esp.h"
#include "esp/geo/geo.h"

namespace esp {
namespace geo {

//! Represents a frame of reference defined by an origin and
//! two semantically meaningful directions: "up" and "front", or
//! equivalently "gravity" and "back"
class CoordinateFrame {
 public:
  explicit CoordinateFrame(const vec3f& up = ESP_UP,
                           const vec3f& front = ESP_FRONT,
                           const vec3f& origin = vec3f::Zero());
  explicit CoordinateFrame(const quatf& rotation,
                           const vec3f& origin = vec3f::Zero());
  explicit CoordinateFrame(const std::string& json);

  //! Returns position of origin of this CoordinateFrame relative to parent
  vec3f origin() const { return origin_; }

  //! Returns up orientation
  vec3f up() const { return up_; }

  //! Returns down/gravity orientation
  vec3f gravity() const { return -up_; }

  //! Returns front orientation
  vec3f front() const { return front_; }

  //! Returns front orientation
  vec3f back() const { return -front_; }

  //! Returns quaternion representing the rotation taking direction vectors in
  //! world coordinates to direction vectors in this CoordinateFrame
  quatf rotationWorldToFrame() const;

  //! Returns quaternion representing the rotation taking direction vectors in
  //! this CoordinateFrame to direction vectors in world coordinates
  quatf rotationFrameToWorld() const;

  //! Return Transform from world coordinates to local coordinates
  Transform transformationWorldToFrame() const;

  //! Returns a stringified JSON representation of this CoordinateFrame
  std::string toJson() const;

  //! Read CoordinateFrame from stringified JSON
  void fromJson(const std::string& json);

 protected:
  vec3f up_;
  vec3f front_;
  vec3f origin_;
  ESP_SMART_POINTERS(CoordinateFrame)
};

bool operator==(const CoordinateFrame& a, const CoordinateFrame& b);
bool operator!=(const CoordinateFrame& a, const CoordinateFrame& b);

inline std::ostream& operator<<(std::ostream& os, const CoordinateFrame& c) {
  return os << c.toJson();
}

}  // namespace geo
}  // namespace esp

#endif  // ESP_GEO_COORDINATEFRAME_H_
