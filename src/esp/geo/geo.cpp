// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/geo/geo.h"

#include <Magnum/Math/FunctionsBatch.h>
#include <numeric>

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
  for (size_t i = idx.size() - 2, t = k + 1; i >= 0; i--) {
    while (k >= t && cross(points[hullIdx[k - 2]], points[hullIdx[k - 1]],
                           points[idx[i]]) <= 0) {
      k--;
    }

    hullIdx[k++] = idx[i];
  }

  hullIdx.resize(k - 1);

  std::vector<vec2f> hull;
  for (auto& ix : hullIdx) {
    hull.emplace_back(points[ix]);
  }

  return hull;
}

Magnum::Range3D getTransformedBB(const Magnum::Range3D& range,
                                 const Magnum::Matrix4& T) {
  std::vector<Magnum::Vector3> corners;
  corners.push_back(T.transformPoint(range.frontBottomLeft()));
  corners.push_back(T.transformPoint(range.frontBottomRight()));
  corners.push_back(T.transformPoint(range.frontTopLeft()));
  corners.push_back(T.transformPoint(range.frontTopRight()));

  corners.push_back(T.transformPoint(range.backTopLeft()));
  corners.push_back(T.transformPoint(range.backTopRight()));
  corners.push_back(T.transformPoint(range.backBottomLeft()));
  corners.push_back(T.transformPoint(range.backBottomRight()));

  Magnum::Range3D transformedBB{Magnum::Math::minmax<Magnum::Vector3>(corners)};

  return transformedBB;
}

}  // namespace geo
}  // namespace esp
