// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "OBB.h"

#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Range.h>

#include <array>
#include <vector>

#include "esp/core/Check.h"
#include "esp/core/Utility.h"
#include "esp/geo/Geo.h"

namespace Mn = Magnum;
namespace esp {
namespace geo {

OBB::OBB(const Mn::Vector3& center,
         const Mn::Vector3& dimensions,
         const Mn::Quaternion& rotation)
    : center_(center), halfExtents_(dimensions * 0.5), rotation_(rotation) {
  recomputeTransforms();
}

OBB::OBB(const Mn::Range3D& aabb)
    : OBB(aabb.center(), aabb.size(), Mn::Quaternion(Mn::Math::IdentityInit)) {}

static const Mn::Vector3 kCorners[8] = {
    Mn::Vector3(-1, -1, -1), Mn::Vector3(-1, -1, +1), Mn::Vector3(-1, +1, -1),
    Mn::Vector3(-1, +1, +1), Mn::Vector3(+1, -1, -1), Mn::Vector3(+1, -1, +1),
    Mn::Vector3(+1, +1, -1), Mn::Vector3(+1, +1, +1)};

Mn::Range3D OBB::toAABB() const {
  Mn::Range3D bbox;
  for (int i = 0; i < 8; ++i) {
    const Mn::Vector3 worldPoint =
        center_ +
        (rotation_.transformVectorNormalized(kCorners[i] * halfExtents_));
    bbox = Mn::Math::join(bbox, worldPoint);
  }
  return bbox;
}

void OBB::recomputeTransforms() {
  const Mn::Matrix3 R = rotation_.toMatrix();
  // Local-to-world transform
  Mn::Matrix3 localToWorldRot;
  for (int i = 0; i < 3; ++i) {
    localToWorldRot[i] = R[i] * halfExtents_[i];
  }

  localToWorld_ = Mn::Matrix4::from(localToWorldRot, center_);

  // World-to-local transform. Points within OBB are in [0,1]^3
  Mn::Matrix3 worldToLocalRotTranspose;
  for (int i = 0; i < 3; ++i) {
    worldToLocalRotTranspose[i] = R[i] * (1.0f / halfExtents_[i]);
  }
  worldToLocal_ =
      Mn::Matrix4::from(worldToLocalRotTranspose.transposed(),
                        (-worldToLocalRotTranspose.transposed() * center_));
}

bool OBB::contains(const Mn::Vector3& p, float eps /* = 1e-6f */) const {
  const Mn::Vector3 pLocal = worldToLocal().transformPoint(p);
  const float bound = 1.0f + eps;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(pLocal[i]) > bound) {
      return false;
    }
  }
  return true;  // Here only if all three coords within bounds
}

float OBB::distance(const Mn::Vector3& p) const {
  if (contains(p)) {
    return 0;
  }
  const Mn::Vector3 closest = closestPoint(p);
  return (p - closest).length();
}

Mn::Vector3 OBB::closestPoint(const Mn::Vector3& p) const {
  const Mn::Vector3 d = p - center_;
  Mn::Vector3 closest = center_;
  const auto R = rotation_.toMatrix();
  for (int i = 0; i < 3; ++i) {
    closest +=
        clamp(Mn::Math::dot(R[i], d), -halfExtents_[i], halfExtents_[i]) * R[i];
  }
  return closest;
}

OBB& OBB::rotate(const Mn::Quaternion& q) {
  rotation_ = q * rotation_;
  recomputeTransforms();
  return *this;
}

// https://geidav.wordpress.com/tag/minimum-obb/
OBB computeGravityAlignedMOBB(const Mn::Vector3& gravity,
                              const std::vector<Mn::Vector3>& points) {
  const auto align_gravity =
      Mn::Quaternion::rotation(gravity, -Mn::Vector3::zAxis());

  static auto ortho = [](const Mn::Vector2& v) {
    return Mn::Vector2(v[1], -v[0]);
  };
  static auto intersect_lines = [](const Mn::Vector2& s0, const Mn::Vector2& d0,
                                   const Mn::Vector2& s1,
                                   const Mn::Vector2& d1) {
    const float dd = d0[0] * d1[1] - d0[1] * d1[0];

    const float dx = s1[0] - s0[0];
    const float dy = s1[1] - s0[1];
    const float t = (dx * d1[1] - dy * d1[0]) / dd;

    return s0 + t * d0;
  };
  static auto mobb_area =
      [](const Mn::Vector2& left_start, const Mn::Vector2& left_dir,
         const Mn::Vector2& right_start, const Mn::Vector2& right_dir,
         const Mn::Vector2& top_start, const Mn::Vector2& top_dir,
         const Mn::Vector2& bottom_start, const Mn::Vector2& bottom_dir) {
        const Mn::Vector2 upper_left =
            intersect_lines(left_start, left_dir, top_start, top_dir);
        const Mn::Vector2 upper_right =
            intersect_lines(right_start, right_dir, top_start, top_dir);
        const Mn::Vector2 bottom_left =
            intersect_lines(bottom_start, bottom_dir, left_start, left_dir);

        return (upper_left - upper_right).length() *
               (upper_left - bottom_left).length();
      };

  std::vector<Mn::Vector2> in_plane_points;
  in_plane_points.reserve(points.size());
  for (const auto& pt : points) {
    Mn::Vector3 aligned_pt = align_gravity.transformVectorNormalized(pt);
    in_plane_points.emplace_back(aligned_pt[0], aligned_pt[1]);
  }

  const auto hull = convexHull2D(in_plane_points);
  CORRADE_INTERNAL_ASSERT(hull.size() > 0);

  std::vector<Mn::Vector2> edge_dirs;
  edge_dirs.reserve(hull.size());
  for (size_t i = 0; i < hull.size(); ++i) {
    edge_dirs.emplace_back(
        (hull[(i + 1) % hull.size()] - hull[i]).normalized());
  }

  Mn::Vector2 min_pt = hull[0], max_pt = hull[0];
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

  Mn::Vector2 left_dir = Mn::Vector2(0, -1), right_dir = Mn::Vector2(0, 1),
              top_dir = Mn::Vector2(-1, 0), bottom_dir = Mn::Vector2(1, 0);

  float best_area = 1e10;
  Mn::Vector2 best_bottom_dir = Mn::Vector2(NAN, NAN);
  for (size_t i = 0; i < hull.size(); ++i) {
    const std::array<float, 4> angles(
        {std::acos(Mn::Math::dot(left_dir, edge_dirs[left_idx])),
         std::acos(Mn::Math::dot(right_dir, edge_dirs[right_idx])),
         std::acos(Mn::Math::dot(top_dir, edge_dirs[top_idx])),
         std::acos(Mn::Math::dot(bottom_dir, edge_dirs[bottom_idx]))});
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
  const auto T_w2b = Mn::Quaternion::rotation(
                         Mn::Vector3(best_bottom_dir[0], best_bottom_dir[1], 0),
                         Mn::Vector3::xAxis()) *
                     align_gravity;

  Mn::Range3D aabb;
  for (const auto& pt : points) {
    const auto transPt = T_w2b.transformVectorNormalized(pt);
    aabb = Mn::Math::join(aabb, pt);
  }
  // Inverted normalized rotation
  const auto inverseT_w2b = T_w2b.inverted().normalized();

  return OBB{inverseT_w2b.transformVectorNormalized(aabb.center()), aabb.size(),
             inverseT_w2b};
}

}  // namespace geo
}  // namespace esp
