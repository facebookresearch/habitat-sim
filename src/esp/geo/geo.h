// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <vector>

#include "esp/core/esp.h"

#include <Magnum/Math/Range.h>
#include "esp/gfx/magnum.h"
namespace Mn = Magnum;

namespace esp {
namespace geo {

//! global/world up direction
static const vec3f ESP_UP = vec3f::UnitY();
//! global/world gravity (down) direction
static const vec3f ESP_GRAVITY = -ESP_UP;
//! global/world front direction
static const vec3f ESP_FRONT = -vec3f::UnitZ();
//! global/world back direction
static const vec3f ESP_BACK = -ESP_FRONT;

typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform;

// compute convex hull of 2D points and return as vector of vertices
std::vector<vec2f> convexHull2D(const std::vector<vec2f>& points);

/**
 * @brief Compute the axis-aligned bounding box which results from applying a
 * transform to an existing bounding box.
 *
 * @param range The initial axis-aligned bounding box.
 * @param xform The desired transform to apply.
 * @return The resulting, transformed axis-aligned bounding box.
 */
Magnum::Range3D getTransformedBB(const Magnum::Range3D& range,
                                 const Magnum::Matrix4& xform);

template <typename T>
T clamp(const T& n, const T& low, const T& high) {
  return std::max(low, std::min(n, high));
}

//! A simple 3D ray defined by an origin point and direction (not necessarily
//! unit length)
struct Ray {
  Mn::Vector3 origin;
  Mn::Vector3 direction;

  Ray(){};

  Ray(Mn::Vector3 _origin, Mn::Vector3 _direction)
      : origin(_origin), direction(_direction){};

  ESP_SMART_POINTERS(Ray)
};

}  // namespace geo
}  // namespace esp
