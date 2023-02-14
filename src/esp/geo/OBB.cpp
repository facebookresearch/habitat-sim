// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "OBB.h"

#include <array>
#include <vector>

#include "esp/core/Check.h"
#include "esp/geo/Geo.h"

namespace esp {
namespace geo {

OBB::OBB() {
  center_.setZero();
  halfExtents_.setZero();
  rotation_.setIdentity();
}

OBB::OBB(const vec3f& center, const vec3f& dimensions, const quatf& rotation)
    : center_(center), halfExtents_(dimensions * 0.5), rotation_(rotation) {
  recomputeTransforms();
}

OBB::OBB(const box3f& aabb)
    : OBB(aabb.center(), aabb.sizes(), quatf::Identity()) {}

static const vec3f kCorners[8] = {
    vec3f(-1, -1, -1), vec3f(-1, -1, +1), vec3f(-1, +1, -1), vec3f(-1, +1, +1),
    vec3f(+1, -1, -1), vec3f(+1, -1, +1), vec3f(+1, +1, -1), vec3f(+1, +1, +1)};

box3f OBB::toAABB() const {
  box3f bbox;
  for (int i = 0; i < 8; ++i) {
    const vec3f worldPoint =
        center_ + (rotation_ * kCorners[i].cwiseProduct(halfExtents_));
    bbox.extend(worldPoint);
  }
  return bbox;
}

void OBB::recomputeTransforms() {
  ESP_CHECK(center_.allFinite(),
            "Illegal center for OBB. Cannot recompute transformations.");
  ESP_CHECK(halfExtents_.allFinite(),
            "Illegal size values for OBB. Cannot recompute transformations.");
  ESP_CHECK(
      rotation_.coeffs().allFinite(),
      "Illegal rotation quaternion for OBB. Cannot recompute transformations.");

  // TODO(MS): these can be composed more efficiently and directly
  const mat3f R = rotation_.matrix();
  // Local-to-world transform
  for (int i = 0; i < 3; ++i) {
    localToWorld_.linear().col(i) = R.col(i) * halfExtents_[i];
  }
  localToWorld_.translation() = center_;

  // World-to-local transform. Points within OBB are in [0,1]^3
  for (int i = 0; i < 3; ++i) {
    worldToLocal_.linear().row(i) = R.col(i) * (1.0f / halfExtents_[i]);
  }
  worldToLocal_.translation() = -worldToLocal_.linear() * center_;
}

bool OBB::contains(const vec3f& p, float eps /* = 1e-6f */) const {
  const vec3f pLocal = worldToLocal() * p;
  const float bound = 1.0f + eps;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(pLocal[i]) > bound) {
      return false;
    }
  }
  return true;  // Here only if all three coords within bounds
}

float OBB::distance(const vec3f& p) const {
  if (contains(p)) {
    return 0;
  }
  const vec3f closest = closestPoint(p);
  return (p - closest).norm();
}

vec3f OBB::closestPoint(const vec3f& p) const {
  const vec3f d = p - center_;
  vec3f closest = center_;
  const mat3f R = rotation_.matrix();
  for (int i = 0; i < 3; ++i) {
    closest +=
        clamp(R.col(i).dot(d), -halfExtents_[i], halfExtents_[i]) * R.col(i);
  }
  return closest;
}

OBB& OBB::rotate(const quatf& q) {
  rotation_ = q * rotation_;
  recomputeTransforms();
  return *this;
}

// https://geidav.wordpress.com/tag/minimum-obb/
OBB computeGravityAlignedMOBB(const vec3f& gravity,
                              const std::vector<vec3f>& points) {
  const auto align_gravity = quatf::FromTwoVectors(gravity, -vec3f::UnitZ());

  static auto ortho = [](const vec2f& v) { return vec2f(v[1], -v[0]); };
  static auto intersect_lines = [](const vec2f& s0, const vec2f& d0,
                                   const vec2f& s1, const vec2f& d1) {
    const float dd = d0[0] * d1[1] - d0[1] * d1[0];

    const float dx = s1[0] - s0[0];
    const float dy = s1[1] - s0[1];
    const float t = (dx * d1[1] - dy * d1[0]) / dd;

    return s0 + t * d0;
  };
  static auto mobb_area = [](const vec2f& left_start, const vec2f& left_dir,
                             const vec2f& right_start, const vec2f& right_dir,
                             const vec2f& top_start, const vec2f& top_dir,
                             const vec2f& bottom_start,
                             const vec2f& bottom_dir) {
    const vec2f upper_left =
        intersect_lines(left_start, left_dir, top_start, top_dir);
    const vec2f upper_right =
        intersect_lines(right_start, right_dir, top_start, top_dir);
    const vec2f bottom_left =
        intersect_lines(bottom_start, bottom_dir, left_start, left_dir);

    return (upper_left - upper_right).norm() *
           (upper_left - bottom_left).norm();
  };

  std::vector<vec2f> in_plane_points;
  in_plane_points.reserve(points.size());
  for (const auto& pt : points) {
    vec3f aligned_pt = align_gravity * pt;
    in_plane_points.emplace_back(aligned_pt[0], aligned_pt[1]);
  }

  const auto hull = convexHull2D(in_plane_points);
  CORRADE_INTERNAL_ASSERT(hull.size() > 0);

  std::vector<vec2f> edge_dirs;
  edge_dirs.reserve(hull.size());
  for (size_t i = 0; i < hull.size(); ++i) {
    edge_dirs.emplace_back(
        (hull[(i + 1) % hull.size()] - hull[i]).normalized());
  }

  vec2f min_pt = hull[0], max_pt = hull[0];
  int left_idx = 0, right_idx = 0, top_idx = 0, bottom_idx = 0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto& pt = hull[i];
    if (pt[0] < min_pt[0]) {
      min_pt[0] = pt[0];
      left_idx = i;
    }

    if (pt[0] > max_pt[0]) {
      max_pt[0] = pt[0];
      right_idx = i;
    }

    if (pt[1] < min_pt[1]) {
      min_pt[1] = pt[1];
      bottom_idx = i;
    }

    if (pt[1] > max_pt[1]) {
      max_pt[1] = pt[1];
      top_idx = i;
    }
  }

  vec2f left_dir = vec2f(0, -1), right_dir = vec2f(0, 1),
        top_dir = vec2f(-1, 0), bottom_dir = vec2f(1, 0);

  float best_area = 1e10;
  vec2f best_bottom_dir = vec2f(NAN, NAN);
  for (size_t i = 0; i < hull.size(); ++i) {
    const std::array<float, 4> angles(
        {std::acos(left_dir.dot(edge_dirs[left_idx])),
         std::acos(right_dir.dot(edge_dirs[right_idx])),
         std::acos(top_dir.dot(edge_dirs[top_idx])),
         std::acos(bottom_dir.dot(edge_dirs[bottom_idx]))});
    float min_angle = 1e10;
    size_t best_line = 0;
    for (size_t i = 0; i < angles.size(); ++i) {
      if (angles[i] < min_angle) {
        best_line = i;
        min_angle = angles[i];
      }
    }

    switch (best_line) {
      case 0:
        left_dir = edge_dirs[left_idx];
        right_dir = -left_dir;
        top_dir = ortho(left_dir);
        bottom_dir = -top_dir;
        left_idx = (left_idx + 1) % hull.size();
        break;
      case 1:
        right_dir = edge_dirs[right_idx];
        left_dir = -right_dir;
        top_dir = ortho(left_dir);
        bottom_dir = -top_dir;
        right_idx = (right_idx + 1) % hull.size();
        break;
      case 2:
        top_dir = edge_dirs[top_idx];
        bottom_dir = -top_dir;
        left_dir = ortho(bottom_dir);
        right_dir = -left_dir;
        top_idx = (top_idx + 1) % hull.size();
        break;
      case 3:
        bottom_dir = edge_dirs[bottom_idx];
        top_dir = -bottom_dir;
        left_dir = ortho(bottom_dir);
        right_dir = -left_dir;
        bottom_idx = (bottom_idx + 1) % hull.size();
        break;
      default:
        CORRADE_INTERNAL_ASSERT(false);
    }

    const float area =
        mobb_area(hull[left_idx], left_dir, hull[right_idx], right_dir,
                  hull[top_idx], top_dir, hull[bottom_idx], bottom_dir);
    if (area < best_area) {
      best_bottom_dir = bottom_dir;
      best_area = area;
    }
  }
  const auto T_w2b =
      quatf::FromTwoVectors(vec3f(best_bottom_dir[0], best_bottom_dir[1], 0),
                            vec3f::UnitX()) *
      align_gravity;

  box3f aabb;
  aabb.setEmpty();
  for (const auto& pt : points) {
    aabb.extend(T_w2b * pt);
  }

  return OBB{T_w2b.inverse() * aabb.center(), aabb.sizes(), T_w2b.inverse()};
}

}  // namespace geo
}  // namespace esp
