// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/geo/geo.h"

#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Trade/MeshData.h>
#include <cmath>
#include <numeric>

namespace Mn = Magnum;
namespace Cr = Corrade;

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

Mn::Vector3 interp2Points(const Mn::Vector3& a,
                          float ta,
                          const Mn::Vector3& b,
                          float tb,
                          float t) {
  float denom = tb - ta;
  return (((tb - t) / denom) * a) + (((t - ta) / denom) * b);
}

float calcTForPoints(const Mn::Vector3& a, const Mn::Vector3& b) {
  // calc t value based on distance between a and b
  float alpha = .25f;
  Mn::Vector3 d = b - a;
  float sqD = dot(d, d);
  return Mn::Math::pow(sqD, alpha);
}

void buildCRTraj4Points(const std::vector<Mn::Vector3>& pts,
                        const std::vector<float>& ptKnotVals,
                        std::vector<Mn::Vector3>& trajectory,
                        int stIdx,
                        int numInterp) {
  std::vector<float> tAra;
  tAra.push_back(0.0f);
  tAra.push_back(ptKnotVals[stIdx]);
  tAra.push_back(ptKnotVals[stIdx + 1] + tAra[1]);
  tAra.push_back(ptKnotVals[stIdx + 2] + tAra[2]);
  float incr = (tAra[2] - tAra[1]) / (1.0f * numInterp);
  // for (float t = tAra[1]; t < tAra[2]; t += incr) {
  for (int i = 0; i < numInterp; ++i) {
    float t = tAra[1] + i * incr;
    t = (t > tAra[2]) ? tAra[2] : t;
    Mn::Vector3 A0 =
        interp2Points(pts[stIdx], tAra[0], pts[stIdx + 1], tAra[1], t);
    Mn::Vector3 A1 =
        interp2Points(pts[stIdx + 1], tAra[1], pts[stIdx + 2], tAra[2], t);
    Mn::Vector3 A2 =
        interp2Points(pts[stIdx + 2], tAra[2], pts[stIdx + 3], tAra[3], t);

    Mn::Vector3 B0 = interp2Points(A0, tAra[0], A1, tAra[2], t);
    Mn::Vector3 B1 = interp2Points(A1, tAra[1], A2, tAra[3], t);

    trajectory.emplace_back(interp2Points(B0, tAra[1], B1, tAra[2], t));
  }

}  // buildCRTraj4Points

std::vector<Mn::Vector3> buildSmoothTrajOfPoints(
    const std::vector<Mn::Vector3>& pts,
    int numInterp) {
  std::vector<Mn::Vector3> trajectory;
  std::vector<Mn::Vector3> tmpPoints;
  std::vector<float> ptKnotVals;
  // build padded array of points to use to synthesize centripetal catmul-rom
  // trajectory
  tmpPoints.emplace_back(pts[0]);

  for (int i = 1; i < pts.size(); ++i) {
    tmpPoints.emplace_back(pts[i]);
    ptKnotVals.emplace_back(calcTForPoints(pts[i - 1], pts[i]));
  }
  tmpPoints.emplace_back(pts[pts.size() - 1]);
  ptKnotVals.emplace_back(0);

  for (int i = 0; i < tmpPoints.size() - 2; ++i) {
    buildCRTraj4Points(pts, ptKnotVals, trajectory, i, numInterp);
  }

  return trajectory;
}  // buildSmoothTrajOfPoints

Mn::Trade::MeshData trajectoryTubeSolid(const std::vector<Mn::Vector3>& pts,
                                        int numSegments,
                                        float radius,
                                        bool smooth,
                                        int numInterp) {
  // 1. Build smoothed trajectory through passed points if requested
  std::vector<Mn::Vector3> trajectory =
      (smooth ? geo::buildSmoothTrajOfPoints(pts, numInterp) : pts);
  // size of trajectory
  const Mn::UnsignedInt trajSize = trajectory.size();
  LOG(INFO) << "geo::trajectoryTubeSolid : Number of trajectory points : "
            << trajSize;
  // 2. Build mesh vertex points around each trajectory point at appropriate
  // distance (radius). For each point in trajectory, add a wireframe circle
  // centered at that point, appropriately oriented based on tangents

  Cr::Containers::Array<Magnum::Vector3> circleVerts =
      Mn::Primitives::circle3DWireframe(numSegments).positions3DAsArray();
  // normalized verts
  Cr::Containers::Array<Magnum::Vector3> circleNormVerts{
      Cr::Containers::NoInit, sizeof(Magnum::Vector3) * numSegments};

  // transform points to be on circle of given radius, and make copy to
  // normalize points
  for (int i = 0; i < numSegments; ++i) {
    circleVerts[i] *= radius;
    circleNormVerts[i] = circleVerts[i].normalized();
  }

  // # of vertices in resultant tube == # circle verts * # points in trajectory
  const Mn::UnsignedInt vertexCount = numSegments * trajSize + 2;
  struct Vertex {  // a function-local struct
    Mn::Vector3 position;
    Mn::Vector3 normal;
  };
  // Vertex data storage
  Cr::Containers::Array<char> vertexData{Cr::Containers::NoInit,
                                         sizeof(Vertex) * vertexCount};

  Cr::Containers::StridedArrayView1D<Vertex> vertices =
      Cr::Containers::arrayCast<Vertex>(vertexData);
  // Position and normal views of vertex array
  Cr::Containers::StridedArrayView1D<Mn::Vector3> positions =
      vertices.slice(&Vertex::position);
  Cr::Containers::StridedArrayView1D<Mn::Vector3> normals =
      vertices.slice(&Vertex::normal);

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
    ++circlePtIDX;
  }
  // add cap vert at the end of the list
  // build vertex (circleVerts[i] is at radius)
  positions[vertexCount - 2] = trajectory[0];
  // pre-rotated normal for circle is normalized point
  normals[vertexCount - 2] =
      tangentOrientation.transformVector({0.0f, 0.0f, -1.0f});

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
      ++circlePtIDX;
    }
  }
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
    ++circlePtIDX;
  }
  // add cap vert
  // build vertex (circleVerts[i] is at radius)
  positions[vertexCount - 1] = trajectory[idx];
  // pre-rotated normal for circle is normalized point
  normals[vertexCount - 1] =
      tangentOrientation.transformVector({0.0f, 0.0f, 1.0f});

  // 3. Create polys between all points
  Cr::Containers::Array<char> indexData{
      Cr::Containers::NoInit,
      6 * numSegments * trajSize * sizeof(Mn::UnsignedInt)};
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
      indices[iListIDX++] = (ix);
      indices[iListIDX++] = (ixNext);
      indices[iListIDX++] = (ixPlus);
      // F2
      indices[iListIDX++] = (ixPlus);
      indices[iListIDX++] = (ixNext);
      indices[iListIDX++] = (ixNextPlus);
    }
  }
  int offset = numSegments * (trajSize - 1);
  // end caps
  for (Mn::UnsignedInt circleIx = 0; circleIx < numSegments; ++circleIx) {
    // endcap 1
    Mn::UnsignedInt ix = circleIx;
    Mn::UnsignedInt ixPlus = (ix + 1) % numSegments;  //+1
    indices[iListIDX++] = (ix);
    indices[iListIDX++] = (ixPlus);
    indices[iListIDX++] = (vertexCount - 2);
    // endcap 2
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

  // Building mesh this way should obviate the need for interleaving

  Mn::Trade::MeshData meshData{
      Mn::MeshPrimitive::Triangles,
      std::move(indexData),
      Mn::Trade::MeshIndexData{indices},
      std::move(vertexData),
      {Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Position,
                                    positions},
       Mn::Trade::MeshAttributeData{Mn::Trade::MeshAttribute::Normal, normals}},
      static_cast<Mn::UnsignedInt>(positions.size())};

  return meshData;
}  // ResourceManager::trajectoryTubeSolid

}  // namespace geo
}  // namespace esp
