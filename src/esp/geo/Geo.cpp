// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/geo/Geo.h"

#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Trade/MeshData.h>
#include <cmath>
#include <numeric>

namespace Mn = Magnum;
namespace Cr = Corrade;
using Magnum::Math::Literals::operator""_rgb;
namespace esp {
namespace geo {

std::vector<vec2f> convexHull2D(const std::vector<vec2f>& points) {
  CORRADE_INTERNAL_ASSERT(points.size() > 2);

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
 * First, determining the x_max:
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

float calcWeightedDistance(const Mn::Vector3& a,
                           const Mn::Vector3& b,
                           float alpha) {
  // embed square root from distance calc
  alpha *= .5;
  // calc t value based on L2 norm of displacement between a and b raised to
  // passed alpha power.
  Mn::Vector3 d = b - a;
  float squareDist = dot(d, d);
  return Mn::Math::pow(squareDist, alpha);
}

void buildCatmullRomTraj4Points(const std::vector<Mn::Vector3>& pts,
                                const std::vector<float>& ptKnotVals,
                                std::vector<Mn::Vector3>& trajectory,
                                int stIdx,
                                int numInterp) {
  // t values are based on distances between sequential points and type of
  // spline
  float t0 = 0.0f;
  float t1 = ptKnotVals[stIdx + 1];
  float t2 = ptKnotVals[stIdx + 2] + t1;
  float t3 = ptKnotVals[stIdx + 3] + t2;
  float incr = (t2 - t1) / (1.0f * (numInterp - 1));
  for (int i = 0; i < numInterp; ++i) {
    float t = t1 + i * incr;
    // don't allow float error to cause t to go past 3rd interpolated point in
    // spline
    t = (t > t2) ? t2 : t;
    Mn::Vector3 A0 = interp2Points(pts[stIdx], t0, pts[stIdx + 1], t1, t);
    Mn::Vector3 A1 = interp2Points(pts[stIdx + 1], t1, pts[stIdx + 2], t2, t);
    Mn::Vector3 A2 = interp2Points(pts[stIdx + 2], t2, pts[stIdx + 3], t3, t);

    Mn::Vector3 B0 = interp2Points(A0, t0, A1, t2, t);
    Mn::Vector3 B1 = interp2Points(A1, t1, A2, t3, t);
    // resultant point will be between t1 and t2 in this 4-point spline
    trajectory.emplace_back(interp2Points(B0, t1, B1, t2, t));
  }

}  // buildCatmullRomTraj4Points

std::vector<Mn::Vector3> buildCatmullRomTrajOfPoints(
    const std::vector<Mn::Vector3>& pts,
    int numInterp,
    float alpha) {
  // enforce alpha limits
  alpha = clamp(alpha, 0.0f, 1.0f);
  // points in trajectory
  std::vector<Mn::Vector3> trajectory;
  trajectory.reserve(pts.size() * numInterp);
  std::vector<Mn::Vector3> tmpPoints;
  tmpPoints.reserve(pts.size() + 2);
  std::vector<float> ptKnotVals;
  ptKnotVals.reserve(pts.size() + 2);
  // build padded array of points to use to synthesize centripetal catmul-rom
  // trajectory by adding "ghost" point so we start drawing from initial point
  // in trajectory.
  tmpPoints.emplace_back(pts[0] - (pts[1] - pts[0]));
  ptKnotVals.emplace_back(calcWeightedDistance(tmpPoints[0], pts[0], alpha));
  for (int i = 0; i < pts.size(); ++i) {
    tmpPoints.emplace_back(pts[i]);
    ptKnotVals.emplace_back(calcWeightedDistance(tmpPoints[i], pts[i], alpha));
  }
  // add final ghost point in trajectory
  int lastIdx = pts.size() - 1;
  tmpPoints.emplace_back(pts[lastIdx] + (pts[lastIdx] - pts[lastIdx - 1]));
  ptKnotVals.emplace_back(calcWeightedDistance(
      tmpPoints[tmpPoints.size() - 2], tmpPoints[tmpPoints.size() - 1], alpha));

  for (int i = 0; i < tmpPoints.size() - 3; ++i) {
    buildCatmullRomTraj4Points(tmpPoints, ptKnotVals, trajectory, i, numInterp);
  }
  return trajectory;
}  // buildCatmullRomTrajOfPoints

std::vector<float> getPointDistsAlongTrajectory(
    const std::vector<Mn::Vector3>& pts) {
  std::vector<float> dists;
  dists.emplace_back(0.0f);
  for (int i = 1; i < pts.size(); ++i) {
    dists.emplace_back(dists[i - 1] +
                       calcWeightedDistance(pts[i - 1], pts[i], 1.0f));
  }
  return dists;
}  // getPointDistsAlongTrajectory

namespace {
Mn::Vector3 convertClrToInterpClrVector(const Mn::Color3& clrFloat,
                                        ColorSpace clrType) {
  Mn::Color3ub clr = Mn::Math::pack<Mn::Color3ub>(clrFloat);
  switch (clrType) {
    case ColorSpace::RGBA: {
      return Mn::Vector3(clr.r(), clr.g(), clr.b());
    }
    case ColorSpace::sRGBA: {
      return clr.toSrgb();
    }
    case ColorSpace::HSV: {
      Mn::ColorHsv tmpHsv = clr.toHsv();
      return Mn::Vector3(tmpHsv.hue.operator float(), tmpHsv.saturation,
                         tmpHsv.value);
    }
    case ColorSpace::XYZ: {
      return clr.toXyz();
    }
    default: {
      CORRADE_ASSERT_UNREACHABLE("Unknown Geo::ColorSpace specified.",
                                 Mn::Vector3({clr.r(), clr.g(), clr.b()}));
    }
  }
}  // convertClrToInterpClrVector

Mn::Color3ub convertInterpClrVecToUBClr(const Mn::Vector3& clrVec,
                                        ColorSpace clrType) {
  Mn::Color3ub resClr(clrVec);
  switch (clrType) {
    case ColorSpace::RGBA: {
      break;
    }
    case ColorSpace::sRGBA: {
      resClr = Mn::Color3ub::fromSrgb(clrVec);
      break;
    }
    case ColorSpace::HSV: {
      resClr =
          Mn::Color3ub::fromHsv({Mn::Deg(clrVec[0]), clrVec[1], clrVec[2]});
      break;
    }
    case ColorSpace::XYZ: {
      resClr = Mn::Color3ub::fromXyz(clrVec);
      break;
    }
    default: {
      CORRADE_ASSERT_UNREACHABLE("Unknown Geo::ColorSpace specified.",
                                 {clrVec});
    }
  }
  return resClr;
}  // convertInterpClrVecToUBClr

}  // namespace

Mn::Trade::MeshData buildTrajectoryTubeSolid(
    const std::vector<Mn::Vector3>& pts,
    const std::vector<Mn::Color3>& interpColors,
    int numSegments,
    float radius,
    bool smooth,
    int numInterp,
    ColorSpace clrSpace) {
  // 1. Build smoothed trajectory through passed points if requested
  // points in trajectory
  // A centripetal CR spline (alpha == .5) will not have cusps, while remaining
  // true to underlying key point trajectory.
  float alpha = .5;
  std::vector<Mn::Vector3> trajectory =
      smooth ? buildCatmullRomTrajOfPoints(pts, numInterp, alpha) : pts;

  // size of trajectory
  const Mn::UnsignedInt trajSize = trajectory.size();

  // 2. Build list of interpolating colors for each ring of trajectory.
  // want to evenly interpolate between colors provided
  const Mn::UnsignedInt numColors = interpColors.size();
  std::vector<Mn::Vector3> trajColors;

  if (numColors == 1) {
    trajColors.reserve(trajSize);
    // with only 1 color, just make duplicates of color for every trajectory
    // point/vertex
    for (const auto& pt : trajectory) {
      trajColors.emplace_back(
          convertClrToInterpClrVector(interpColors[0], clrSpace));
    }

  } else {
    // interpolate in hsv space - first convert src colors to HSV
    std::vector<Mn::Vector3> srcClrs;
    srcClrs.reserve(numColors);
    for (const auto& clr : interpColors) {
      srcClrs.emplace_back(convertClrToInterpClrVector(clr, clrSpace));
    }
    // determine how many interpolations we should have : trajColors should be
    // trajSize in size
    int numClrInterp = (trajSize / (numColors - 1)) + 1;
    // now build interpolated vector of colors
    trajColors = buildCatmullRomTrajOfPoints(srcClrs, numClrInterp, alpha);
    // fill end of trajColors array with final color if smaller than size of
    // trajectory
    while (trajColors.size() < trajSize) {
      trajColors.push_back(trajColors.back());
    }
  }

  // 3. Build mesh vertex points around each trajectory point at appropriate
  // distance (radius). For each point in trajectory, add a wireframe circle
  // centered at that point, appropriately oriented based on tangents

  Cr::Containers::Array<Magnum::Vector3> circleVerts =
      Mn::Primitives::circle3DWireframe(numSegments).positions3DAsArray();
  // normalized verts will provide vert normals.
  Cr::Containers::Array<Magnum::Vector3> circleNormVerts{
      Cr::NoInit, sizeof(Magnum::Vector3) * numSegments};

  // transform points to be on circle of given radius, and make copy to
  // normalize points
  for (int i = 0; i < numSegments; ++i) {
    circleVerts[i] *= radius;
    circleNormVerts[i] = circleVerts[i].normalized();
  }

  // # of vertices in resultant tube == # circle verts * # points in trajectory
  const Mn::UnsignedInt vertexCount = numSegments * trajSize + 2;
  // a function-local struct representing a vertex
  struct Vertex {
    Mn::Vector3 position;
    Mn::Vector3 normal;
    // vertex color default to white
    Mn::Color3ub color = 0xffffff_rgb;
  };

  // Vertex data storage
  Cr::Containers::Array<char> vertexData{Cr::NoInit,
                                         sizeof(Vertex) * vertexCount};
  // Cast memory to be a strided array so it can be accessed via slices.
  Cr::Containers::StridedArrayView1D<Vertex> vertices =
      Cr::Containers::arrayCast<Vertex>(vertexData);
  // Position, normal and color views of vertex array
  Cr::Containers::StridedArrayView1D<Mn::Vector3> positions =
      vertices.slice(&Vertex::position);
  Cr::Containers::StridedArrayView1D<Mn::Vector3> normals =
      vertices.slice(&Vertex::normal);
  Cr::Containers::StridedArrayView1D<Mn::Color3ub> colors =
      vertices.slice(&Vertex::color);

  // Beginning Endcap
  Mn::UnsignedInt circlePtIDX = 0;
  Mn::Vector3 tangent = trajectory[1] - trajectory[0];
  // get the orientation matrix assuming y-up preference
  Mn::Matrix4 tangentOrientation = Mn::Matrix4::lookAt(
      trajectory[0], trajectory[0] + tangent, Mn::Vector3{0, 1.0, 0});
  for (int i = 0; i < numSegments; ++i) {
    // build vertex (circleVerts[i] is at radius)
    positions[circlePtIDX] = tangentOrientation.transformPoint(circleVerts[i]);
    // pre-rotated normal for circle is normalized point
    normals[circlePtIDX] =
        tangentOrientation.transformVector(circleNormVerts[i]);
    colors[circlePtIDX] = convertInterpClrVecToUBClr(trajColors[0], clrSpace);
    ++circlePtIDX;
  }
  // add beginning cap vert at the end of the list
  // build vertex (circleVerts[i] is at radius)
  positions[vertexCount - 2] = trajectory[0];
  // normal points out at beginning cap vert
  normals[vertexCount - 2] =
      tangentOrientation.transformVector({0.0f, 0.0f, -1.0f});
  colors[vertexCount - 2] = convertInterpClrVecToUBClr(trajColors[0], clrSpace);

  for (Mn::UnsignedInt vertIx = 1; vertIx < trajSize - 1; ++vertIx) {
    const Mn::Vector3& vert = trajectory[vertIx];
    Mn::Vector3 pTangent = vert - trajectory[vertIx - 1];
    Mn::Vector3 nTangent = trajectory[vertIx + 1] - vert;
    tangent = (pTangent + nTangent) / 2.0;
    // get the orientation matrix assuming y-up preference
    tangentOrientation =
        Mn::Matrix4::lookAt(vert, vert + tangent, Mn::Vector3{0, 1.0, 0});
    for (int i = 0; i < numSegments; ++i) {
      // build vertex (circleVerts[i] is at radius)
      positions[circlePtIDX] =
          tangentOrientation.transformPoint(circleVerts[i]);
      // pre-rotated normal for circle is normalized point
      normals[circlePtIDX] =
          tangentOrientation.transformVector(circleNormVerts[i]);
      colors[circlePtIDX] =
          convertInterpClrVecToUBClr(trajColors[vertIx], clrSpace);
      ++circlePtIDX;
    }
  }

  // Ending Endcap
  int idx = trajSize - 1;
  tangent = trajectory[idx] - trajectory[idx - 1];
  // get the orientation matrix assuming y-up preference
  tangentOrientation = Mn::Matrix4::lookAt(
      trajectory[idx], trajectory[idx] + tangent, Mn::Vector3{0, 1.0, 0});
  for (int i = 0; i < numSegments; ++i) {
    // build vertex (circleVerts[i] is at radius)
    positions[circlePtIDX] = tangentOrientation.transformPoint(circleVerts[i]);
    // pre-rotated normal for circle is normalized point
    normals[circlePtIDX] =
        tangentOrientation.transformVector(circleNormVerts[i]);
    colors[circlePtIDX] = convertInterpClrVecToUBClr(trajColors[idx], clrSpace);
    ++circlePtIDX;
  }
  // add end cap vert
  // build vertex (circleVerts[i] is at radius)
  positions[vertexCount - 1] = trajectory[idx];
  // normal points out at end cap vert
  normals[vertexCount - 1] =
      tangentOrientation.transformVector({0.0f, 0.0f, 1.0f});
  colors[vertexCount - 1] =
      convertInterpClrVecToUBClr(trajColors[idx], clrSpace);

  // 4. Create polys between all points
  Cr::Containers::Array<char> indexData{
      Cr::NoInit, 6 * numSegments * trajSize * sizeof(Mn::UnsignedInt)};
  Cr::Containers::ArrayView<Mn::UnsignedInt> indices =
      Cr::Containers::arrayCast<Mn::UnsignedInt>(indexData);

  // create triangle indices for each tube pair correspondance - cw winding
  /*
            +n---+n+1
            | \ F2|
            |  \  |
            |F1 \ |
            +0---+1
        F1 = [+0, +n, +1]
        F2 = [+1, +n, +n+1]
   */
  int iListIDX = 0;
  for (Mn::UnsignedInt vIdx = 0; vIdx < trajSize - 1;
       ++vIdx) {  // skip last circle (adding forward)
    int vIdxNumSeg = vIdx * numSegments;
    for (Mn::UnsignedInt circleIx = 0; circleIx < numSegments; ++circleIx) {
      Mn::UnsignedInt ix = circleIx + vIdxNumSeg;  //+0
      Mn::UnsignedInt ixNext = ix + numSegments;   //+n
      Mn::UnsignedInt ixPlus = ix + 1;             //+1
      Mn::UnsignedInt ixNextPlus = ixNext + 1;     //+n+1
      if (circleIx == numSegments - 1) {
        // last vert in a circle wraps to relative 0
        ixPlus = vIdxNumSeg;
        ixNextPlus = vIdxNumSeg + numSegments;
      }
      // F1
      indices[iListIDX++] = ix;
      indices[iListIDX++] = ixNext;
      indices[iListIDX++] = ixPlus;
      // F2
      indices[iListIDX++] = ixPlus;
      indices[iListIDX++] = ixNext;
      indices[iListIDX++] = ixNextPlus;
    }
  }
  int offset = numSegments * (trajSize - 1);

  // end caps - verts added at the end of the vertices array
  for (Mn::UnsignedInt circleIx = 0; circleIx < numSegments; ++circleIx) {
    // first endcap
    Mn::UnsignedInt ix = circleIx;
    Mn::UnsignedInt ixPlus = (ix + 1) % numSegments;  //+1
    indices[iListIDX++] = (ix);
    indices[iListIDX++] = (ixPlus);
    indices[iListIDX++] = (vertexCount - 2);
    // last endcap
    ix += offset;
    ixPlus += offset;  //+1
    indices[iListIDX++] = (ixPlus);
    indices[iListIDX++] = (ix);
    indices[iListIDX++] = (vertexCount - 1);
  }

  // Finally, make the MeshData. The indices have to be constructed first
  // because function argument evaluation order is not guaranteed and so you
  // might end up with the move happening before the MeshIndexData construction,
  // which would result in 0 indices)

  // Building the mesh this way obviates the need for an interleaving step

  Mn::Trade::MeshData meshData{
      Mn::MeshPrimitive::Triangles,
      std::move(indexData),
      Mn::Trade::MeshIndexData{indices},
      std::move(vertexData),
      {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                    positions},
       Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Normal, normals},
       Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Color, colors}},
      static_cast<Mn::UnsignedInt>(positions.size())};

  return meshData;
}  // buildTrajectoryTubeSolid

namespace {
// TODO remove when/if Magnum ever supports this function for Color3ub
constexpr const char Hex[]{"0123456789abcdef"};
}  // namespace
std::string getColorAsString(const Magnum::Color3ub& color) {
  char out[] = "#______";
  out[1] = Hex[(color.r() >> 4) & 0xf];
  out[2] = Hex[(color.r() >> 0) & 0xf];
  out[3] = Hex[(color.g() >> 4) & 0xf];
  out[4] = Hex[(color.g() >> 0) & 0xf];
  out[5] = Hex[(color.b() >> 4) & 0xf];
  out[6] = Hex[(color.b() >> 0) & 0xf];
  return std::string(out);
}

std::vector<std::set<uint32_t>> buildAdjList(
    int numVerts,
    const std::vector<uint32_t>& indexBuffer) {
  // build adj list by assuming that each sequence of 3 indices in index list
  // denote a triangle.  Idx matches vertex index in vert list, value is set of
  // verts that are adjacent
  std::vector<std::set<uint32_t>> adjList(numVerts, std::set<uint32_t>{});
  for (size_t i = 0; i < indexBuffer.size(); i += 3) {
    // find idxs of triangle
    const uint32_t idx0 = indexBuffer[i];
    const uint32_t idx1 = indexBuffer[i + 1];
    const uint32_t idx2 = indexBuffer[i + 2];
    // save adjacency info for triangle
    adjList[idx0].insert(idx1);
    adjList[idx1].insert(idx0);
    adjList[idx0].insert(idx2);
    adjList[idx2].insert(idx0);
    adjList[idx1].insert(idx2);
    adjList[idx2].insert(idx1);
  }
  return adjList;

}  // buildAdjList

uint32_t getValueAsUInt(const Mn::Color3ub& color) {
  return (unsigned(color[0]) << 16) | (unsigned(color[1]) << 8) |
         unsigned(color[2]);
}
uint32_t getValueAsUInt(const Mn::Color4ub& color) {
  return (unsigned(color[0]) << 24) | (unsigned(color[1]) << 16) |
         unsigned(color[2] << 8) | (unsigned(color[3]));
}
uint32_t getValueAsUInt(int color) {
  return static_cast<uint32_t>(color);
}

}  // namespace geo
}  // namespace esp
