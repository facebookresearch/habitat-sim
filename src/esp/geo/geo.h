// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_GEO_H_
#define ESP_GEO_GEO_H_

#include <vector>

#include "esp/core/esp.h"

#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/Range.h>
#include "esp/gfx/magnum.h"
namespace Mn = Magnum;
namespace Cr = Corrade;

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

/**
 * @brief Build a smooth trajectory of interpolated points from key points
 * along a path using cubic hermitian spline.
 * @param pts The points of the trajectory
 * @param numInterp The number of interpolations between each trajectory point
 * @return interpolated points for trajectory
 */
std::vector<Mn::Vector3> buildSmoothTrajOfPoints(
    const std::vector<Mn::Vector3>& pts,
    int numInterp);

/**
 * @brief Interpolate between two points to find a third.
 * @param a first point
 * @param ta @ref a 's time along trajectory
 * @param b second point
 * @param tb @ref b 's time along trajectory
 * @param t the time along the trajectory for the desired point
 * @return the interpolated point at time @ref t
 */
inline Mn::Vector3 interp2Points(const Mn::Vector3& a,
                                 float ta,
                                 const Mn::Vector3& b,
                                 float tb,
                                 float t) {
  float denom = tb - ta;
  return (((tb - t) / denom) * a) + (((t - ta) / denom) * b);
}
/**
 * @brief Determine the relative knot value for point b as a function of
 * distance from point a, for use in Catmull-Rom spline derivation.
 * @param a a point whose knot value is known
 * @param b a point whose knot value is unkknown
 * @param alpha power to use to derive distance, determines nature of resultant
 * CR spline. 0 yields a standard uniform Catmull-Rom spline, .5 yields
 * centrepital CR spline, 1.0 yields chordal CR spline.  Default is .5.
 * @return b's knot valute relative to a
 */
float calcTForPoints(const Mn::Vector3& a, const Mn::Vector3& b, float alpha);

/**
 * @brief Build a Catmull–Rom spline using 4 sequential points in
 * @ref pts vector, where the resultant spline will extend between the 2nd and
 * 3rd of the 4 points.
 * @param pts The points of the trajectory
 * @param ptKnotVals Precalculated knot values for sequential points, derived
 * from point-to-point distance.  Depending on how these are calculated, the
 * result may be a uniform, centrepital or chordal CR spline.
 * @param trajectory The resultant trajectory to build
 * @param stIdx the index to start building the points
 * @param numInterp The number of interpolations between each trajectory point
 */
void buildCatmullRomTraj4Points(const std::vector<Mn::Vector3>& pts,
                                const std::vector<float>& ptKnotVals,
                                std::vector<Mn::Vector3>& trajectory,
                                int stIdx,
                                int numInterp);

/**
 * @brief Build a mesh representing a tube of given radius around the
 * trajectory given by the passed points.  Will either build cylinders between
 * each pair of points in @ref pts, or will build smoothed trajectory by
 * deriving centripetal Catmull-Rom spline between subsequent sets of 4 points
 * and interoplate
 * @param pts The points of a trajectory, in order
 * @param numSegments The number of segments around the circumference of the
 * tube.
 * @param radius The radius of the tube
 * @param smooth Whether to smooth the points or not
 * @param numInterp The number of interpolations between each trajectory
 * point, if smoothing
 * @return The resultant meshdata for the tube
 */
Mn::Trade::MeshData trajectoryTubeSolid(const std::vector<Mn::Vector3>& pts,
                                        int numSegments,
                                        float radius,
                                        bool smooth,
                                        int numInterp);

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

#endif  // ESP_GEO_GEO_H_
