// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/geo/geo.h"

#include <Magnum/Math/FunctionsBatch.h>
#include <cmath>
#include <numeric>

namespace Mn = Magnum;

namespace esp {
namespace geo {

std::vector<vec2f> convexHull2D(const std::vector<vec2f>& points) {
  ASSERT(points.size() > 2);

  auto cross = [](const vec2f& o, const vec2f& a, const vec2f& b) {
    return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
  };

  // Sort indices of points lexicographically
  std::vector<size_t> idx(points.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(
      idx.begin(), idx.end(), [&points](const size_t& a, const size_t& b) {
        return points[a](0) < points[b](0) ||
               (points[a](0) == points[b](0) && points[a](1) < points[b](1));
      });

  std::vector<size_t> hullIdx(2 * idx.size());

  // Build lower hull
  int k = 0;
  for (size_t i = 0; i < idx.size(); ++i) {
    while (k >= 2 && cross(points[hullIdx[k - 2]], points[hullIdx[k - 1]],
                           points[idx[i]]) <= 0) {
      k--;
    }

    hullIdx[k++] = idx[i];
  }

  // Build upper hull
  for (size_t i = idx.size() - 1, t = k + 1; i > 0; i--) {
    while (k >= t && cross(points[hullIdx[k - 2]], points[hullIdx[k - 1]],
                           points[idx[i - 1]]) <= 0) {
      k--;
    }
    hullIdx[k++] = idx[i - 1];
  }

  hullIdx.resize(k - 1);

  std::vector<vec2f> hull;
  hull.reserve(hullIdx.size());

  for (auto& ix : hullIdx) {
    hull.emplace_back(points[ix]);
  }

  return hull;
}
/**
 * Assume the aabb is expressed as the center 'c' and the extent 'e'.
 * each corner X is nothing but a combination from
 * (c_x +/- e_x, c_y +/- e_y, c_z +/- e_z)
 *
 * Denote y = (+/- e_x, +/- e_y, +/- e_z)
 *
 * The corner is transformed by:
 * x = R * x0 + t,
 * where x_0, x are the coordinates before and after transformation, t is the
 * translation.
 *
 * x = R * (c0 + y0) + t  = (Rc0 + t) + Ry0    eq(1)
 *
 * Our Goal is to find the x_max and x_min after the transformation.
 *
 * First, determing the x_max:
 *
 * In eq(1), Rc0 + t is a constant, which means max{x} iff max{Ry0}
 *
 * R looks like:
 * [R0, R1, R2]
 * [R3, R4, R5]
 * [R6, R7, R8]
 *
 * We focus on the 1st entry (the 'x' entry) of Ry0:
 * y_x =<(R0, R1, R2), (+/- e_x, +/- e_y, +/- e_z)>
 *     =<(+/- R0, +/- R1, +/- R2), (e_x, e_y, e_z)>
 *
 * Now, note that e_x, e_y, e_z are all positive values for any non-degenerate
 * aabb.
 *
 * So y_x reaches MAX when +/- R0, +/- R1, +/- R2 are all >=0
 * that means max{y_x} = <(|R0|, |R1|, |R2|), (e_x, e_y, e_z)>
 * (|R0| means the absolute value of R0)
 *
 * and likewise for y_y, y_z
 *
 * The derivation for x_min is similar since same logic applies.
 */
Mn::Range3D getTransformedBB(const Mn::Range3D& range,
                             const Mn::Matrix4& xform) {
  // compute the absolute value of the rotationScaling part of the original
  // transformation matrix
  auto absRotationScaling = Mn::Matrix3x3::fromVector(
      Mn::Math::abs(xform.rotationScaling().toVector()));

  const Mn::Vector3 center = range.center();
  const Mn::Vector3 extent = range.size() / 2.0;

  // compute Rc0 + t
  Mn::Vector3 newCenter = xform.transformPoint(center);
  // compute max{Ry0}
  Mn::Vector3 newExtent = absRotationScaling * extent;

  return Mn::Range3D::fromCenter(newCenter, newExtent);
}

}  // namespace geo
}  // namespace esp
