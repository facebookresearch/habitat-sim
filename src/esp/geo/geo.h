// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <vector>

#include "esp/core/esp.h"

#include <Magnum/Math/Range.h>
#include "esp/gfx/magnum.h"

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

Magnum::Range3D getTransformedBB(const Magnum::Range3D& range,
                                 const Magnum::Matrix4& T);

template <typename T>
T clamp(const T& n, const T& low, const T& high) {
  return std::max(low, std::min(n, high));
}

}  // namespace geo
}  // namespace esp
