// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_COORDINATEFRAME_H_
#define ESP_GEO_COORDINATEFRAME_H_

#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"

namespace esp {
namespace geo {

namespace Mn = Magnum;

//! Represents a frame of reference defined by an origin and
//! two semantically meaningful directions: "up" and "front", or
//! equivalently "gravity" and "back"
class CoordinateFrame {
 public:
  explicit CoordinateFrame(const Mn::Vector3& up = ESP_UP,
                           const Mn::Vector3& front = ESP_FRONT,
                           const Mn::Vector3& origin = {});
  explicit CoordinateFrame(const Mn::Quaternion& rotation,
                           const Mn::Vector3& origin = {});

  //! Returns position of origin of this CoordinateFrame relative to parent
  Mn::Vector3 origin() const { return origin_; }

  //! Returns up orientation
  Mn::Vector3 up() const { return up_; }

  //! Returns down/gravity orientation
  Mn::Vector3 gravity() const { return -up_; }

  //! Returns front orientation
  Mn::Vector3 front() const { return front_; }

  //! Returns front orientation
  Mn::Vector3 back() const { return -front_; }

  //! Returns quaternion representing the rotation taking direction vectors in
  //! world coordinates to direction vectors in this CoordinateFrame
  Mn::Quaternion rotationWorldToFrame() const;

  //! Returns quaternion representing the rotation taking direction vectors in
  //! this CoordinateFrame to direction vectors in world coordinates
  Mn::Quaternion rotationFrameToWorld() const;

  //! Returns a stringified JSON representation of this CoordinateFrame
  std::string toString() const;

 protected:
  Mn::Vector3 up_;
  Mn::Vector3 front_;
  Mn::Vector3 origin_;
  ESP_SMART_POINTERS(CoordinateFrame)
};

bool operator==(const CoordinateFrame& a, const CoordinateFrame& b);
bool operator!=(const CoordinateFrame& a, const CoordinateFrame& b);

inline std::ostream& operator<<(std::ostream& os, const CoordinateFrame& c) {
  return os << c.toString();
}

}  // namespace geo
}  // namespace esp

#endif  // ESP_GEO_COORDINATEFRAME_H_
