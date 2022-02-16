// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_GEO_H_
#define ESP_GEO_GEO_H_

#include <set>
#include <vector>

#include "esp/core/Esp.h"

#include <Magnum/Math/CubicHermite.h>
#include <Magnum/Math/Range.h>
#include "esp/gfx/magnum.h"
namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace geo {

/**
 * @brief This enum describes the various colorspaces that colors in Habitat can
 * occupy.  This is currently governed by the supported color space conversions
 * built into the Magnum Color types. This is provided for easy user access to
 * these conversion mechansims.
 */
enum class ColorSpace {
  RGBA,
  sRGBA,
  /**
   * @brief HSV represent Hue, Saturation and Value, where the color space is
   * modeled on a cylinder, hue is an angle, saturation is a radius and value is
   * measured along the height of the cylinder.
   */
  HSV,
  /**
   * @brief CIE XYZ, where Y is luminance, and x-z plane describes chromaticity.
   */
  XYZ

};

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
 * @brief Return a vector of L2/Euclidean distances of points along a
 * trajectory. First point will always be 0, and last point will give length of
 * trajectory.
 * @param pts Vector of points along trajectory.
 */
std::vector<float> getPointDistsAlongTrajectory(
    const std::vector<Mn::Vector3>& pts);

/**
 * @brief Interpolate between two points to find a third at a specified @ref t
 * between them.
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
  return (((tb - t) * a) + ((t - ta) * b)) / denom;
}

/**
 * @brief Determine the weighted distance of point b from point a.  Used to
 * derive relative knot value for point b as a function of distance from point
 * a for Catmull-Rom spline derivation.
 * @param a a point whose knot value is known
 * @param b a point whose knot value is unkknown
 * @param alpha Exponent for distance calculation used to derive time parameter
 * for splines. Determines nature of resultant CR spline. 0 yields a standard
 * uniform Catmull-Rom spline (which may have cusps), .5 yields centrepital CR
 * spline, 1.0 yields chordal CR spline.  Default is .5.  Use alpha == 1.0f for
 * L2 distance calc.
 * @return b's knot valute relative to a / distance from a
 */
float calcWeightedDistance(const Mn::Vector3& a,
                           const Mn::Vector3& b,
                           float alpha = .5f);

/**
 * @brief Build a smooth trajectory of interpolated points from key points
 * along a path using Catmull-Rom Spline of type determined by chosen @ref
 * alpha.
 * @param pts The key points of the trajectory
 * @param numInterp The number of interpolations between each trajectory point
 * @param alpha Exponent for distance calculation used to derive time parameter.
 * Determines nature of resultant CR spline. 0 yields a standard uniform
 * Catmull-Rom spline (which may have cusps), .5 yields centrepital CR
 * spline, 1.0 yields chordal CR spline.  Default is .5.
 * @return interpolated points for trajectory
 */
std::vector<Mn::Vector3> buildCatmullRomTrajOfPoints(
    const std::vector<Mn::Vector3>& pts,
    int numInterp,
    float alpha = .5f);

/**
 * @brief Build a mesh representing a tube of given radius around the
 * trajectory given by the passed points.  Will either build cylinders
 * between each pair of points in @ref pts, or between sequential points in a
 * smoothed trajectory derived using centripetal Catmull-Rom spline between
 * points in @ref pts.
 * @param pts  The key points of the trajectory, in order
 * @param interpColors The list of one or more colors to assign in sequence to
 * vertices along the length of the tube, interpolating if necessary.
 * @param numSegments The number of segments around the circumference of the
 * tube.
 * @param radius The radius of the tube
 * @param smooth Whether to smooth the points or not
 * @param numInterp The number of interpolations between each trajectory
 * point, if smoothing
 * @param clrSpace The color space to interpolate the given colrs in
 * @return The resultant meshdata for the tube
 */
Mn::Trade::MeshData buildTrajectoryTubeSolid(
    const std::vector<Mn::Vector3>& pts,
    const std::vector<Mn::Color3>& interpColors,
    int numSegments,
    float radius,
    bool smooth,
    int numInterp,
    ColorSpace clrSpace = ColorSpace::HSV);

/**
 * @brief Build a connected component recursively on an unconnected graph
 * (i.e. mesh vertices), building from passed @p vIDX from adjecent verts
 * that match the passed @p clr value.
 *
 * @tparam The type of the conditioning variable.
 * @param adjList A reference to the mesh's adjacency list.  IDX of list is
 * src vert index in owning vert list, value is dest vert index in vert
 * list.
 * @param clrVec A reference to the per-vertex identifiers used to condition
 * the CC.
 * @param vIDX The index of the src vertex of this part of the CC.
 * @param visited Per-vertex visitation record.
 * @param clr The CC's identifying "color", to be matched by adjacent verts
 * for membership.
 * @param setOfVerts Aggregation of verts in the CC.
 */
template <class T>
void conditionalDFS(const std::vector<std::set<uint32_t>>& adjList,
                    const std::vector<T>& clrVec,
                    uint32_t vIDX,
                    std::vector<bool>& visited,
                    const T& clr,
                    std::set<uint32_t>& setOfVerts) {
  setOfVerts.insert(vIDX);
  visited[vIDX] = true;
  // for every adjacent vertex
  for (auto it = adjList[vIDX].begin(); it != adjList[vIDX].end(); it++) {
    // make sure not visited and color matches
    if ((!visited[*it]) && (clrVec[*it] == clr)) {
      conditionalDFS(adjList, clrVec, *it, visited, clr, setOfVerts);
    }
  }
}  // conditionalDFS

template <typename T>
T clamp(const T& n, const T& low, const T& high) {
  return std::max(low, std::min(n, high));
}

//! A simple 3D ray defined by an origin point and direction (not necessarily
//! unit length)
struct Ray {
  Mn::Vector3 origin;
  Mn::Vector3 direction;

  Ray() = default;

  Ray(Mn::Vector3 _origin, Mn::Vector3 _direction)
      : origin(_origin), direction(_direction){};

  ESP_SMART_POINTERS(Ray)
};

}  // namespace geo
}  // namespace esp

#endif  // ESP_GEO_GEO_H_
