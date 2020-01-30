//
//
// Collection of useful 3d geometric routines such as projections, intersection,
// etc.
//
// note:  still doing some reorg to make naming more consistent and pick the
// best and minimal set of routines here
//

#pragma once
#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <assert.h>
#include <algorithm>  // for std::max_element() used by maxdir and supportmap funcs
#include <functional>
#include <iosfwd>  // since we define some << and >> stream io operators
#include <limits>  // for std::numeric_limits used by Extents()
#include <vector>

#include "linalg.h"
using namespace linalg::aliases;  // usual defined aliases (int3,float3,float4x4
                                  // etc.) in the global namespace
#include "misc.h"  // for my std::transform wrapper Transform that creates and returns by value
// still reorganizing little geometry functions, putting these here for now:

struct rect_iteration  // nice programming convenience, but adds some loop
                       // overhead the compiler doesn't see through
{
  struct iterator {
    int2 coord;
    int dims_x;
    int2 operator*() const { return coord; }
    bool operator!=(const iterator& r) const { return coord.y != r.coord.y; }
    void operator++() { coord.y += !(++coord.x %= dims_x); }
  };

  int2 dims;
  rect_iteration(const int2& dims) : dims(dims) {}
  iterator begin() const { return {{0, 0}, dims.x}; }
  iterator end() const { return {{0, dims.y}, dims.x}; }
};
struct vol_iteration {
  struct iterator {
    int3 coord;
    int2 dims;
    int3 operator*() const { return coord; }
    bool operator!=(const iterator& r) const { return coord.z != r.coord.z; }
    void operator++() {
      coord.z += ((coord.y += !(++coord.x %= dims.x)) == dims.y);
      coord.y %= dims.y;
    }
  };

  int3 dims;
  vol_iteration(const int3& dims) : dims(dims) {}
  iterator begin() const { return {{0, 0, 0}, dims.xy()}; }
  iterator end() const { return {{0, 0, dims.z}, dims.xy()}; }
};

inline int2 asint2(const float2& v) {
  return {(int)v.x, (int)v.y};
}
inline float2 asfloat2(const int2& v) {
  return {(float)v.x, (float)v.y};
}

inline float3 safenormalize(const float3& v) {
  return (v == float3(0, 0, 0)) ? float3(0, 0, 1) : normalize(v);
}

// template <class T> T clamp(T a, const T mn = T(0), const T mx = T(1)) {
// return std::min(std::max(a, mn), mx); }  // templating this messed up the
// within_range and clamp for linalg types
inline int clamp(int a, const int mn = int(0), const int mx = int(1)) {
  return std::min(std::max(a, mn), mx);
}
inline float clamp(float a,
                   const float mn = float(0),
                   const float mx = float(1)) {
  return std::min(std::max(a, mn), mx);
}

template <class T, int M>
bool within_range(const linalg::vec<T, M>& v,
                  const linalg::vec<T, M>& mn,
                  const linalg::vec<T, M>& mx) {
  return v == linalg::clamp(v, mn, mx);
}

inline float4 quatfrommat(const float3x3& m) {
  float magw = m[0][0] + m[1][1] + m[2][2];
  float magxy;
  float magzw;
  float3 pre;
  float3 prexy;
  float3 prezw;

  bool wvsz = magw > m[2][2];
  magzw = wvsz ? magw : m[2][2];
  prezw = wvsz ? float3(1, 1, 1) : float3(-1, -1, 1);
  auto postzw = wvsz ? float4(0, 0, 0, 1) : float4(0, 0, 1, 0);

  bool xvsy = m[0][0] > m[1][1];
  magxy = xvsy ? m[0][0] : m[1][1];
  prexy = xvsy ? float3(1, -1, -1) : float3(-1, 1, -1);
  auto postxy = xvsy ? float4(1, 0, 0, 0) : float4(0, 1, 0, 0);

  bool zwvsxy = magzw > magxy;
  pre = zwvsxy ? prezw : prexy;
  auto post = zwvsxy ? postzw : postxy;

  float t = pre.x * m[0][0] + pre.y * m[1][1] + pre.z * m[2][2] + 1;
  float s = 1 / sqrt(t) / 2;
  float4 qp{(pre.y * m[1][2] - pre.z * m[2][1]) * s,
            (pre.z * m[2][0] - pre.x * m[0][2]) * s,
            (pre.x * m[0][1] - pre.y * m[1][0]) * s, t * s};
  return qmul(qp, post);
}

inline float4 QuatFromAxisAngle(const float3& axis, float angle) {
  return {axis * std::sin(angle / 2), std::cos(angle / 2)};
}
inline float4x4 MatrixFromRotation(const float4& rotationQuat) {
  return {{qxdir(rotationQuat), 0},
          {qydir(rotationQuat), 0},
          {qzdir(rotationQuat), 0},
          {0, 0, 0, 1}};
}
inline float4x4 MatrixFromTranslation(const float3& translationVec) {
  return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {translationVec, 1}};
}
inline float4x4 MatrixFromRotationTranslation(const float4& rotationQuat,
                                              const float3& translationVec) {
  return {{qxdir(rotationQuat), 0},
          {qydir(rotationQuat), 0},
          {qzdir(rotationQuat), 0},
          {translationVec, 1}};
}
inline float4x4 MatrixFromProjectionFrustum(float l,
                                            float r,
                                            float b,
                                            float t,
                                            float n,
                                            float f) {
  return {{2 * n / (r - l), 0, 0, 0},
          {0, 2 * n / (t - b), 0, 0},
          {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1},
          {0, 0, -2 * f * n / (f - n), 0}};
}
inline float4x4 MatrixFromVfovAspect(float vfov,
                                     float aspect,
                                     float n,
                                     float f) {
  float y = n * std::tan(vfov / 2), x = y * aspect;
  return MatrixFromProjectionFrustum(-x, x, -y, y, n, f);
}
inline float4x4 MatrixFromLookVector(const float3& fwd, const float3& up) {
  auto f = normalize(fwd), s = normalize(cross(f, up)), u = cross(s, f);
  return {{s.x, u.x, -f.x, 0},
          {s.y, u.y, -f.y, 0},
          {s.z, u.z, -f.z, 0},
          {0, 0, 0, 1}};
}
inline float4x4 MatrixFromLookAt(const float3& eye,
                                 const float3& center,
                                 const float3& up) {
  return mul(MatrixFromLookVector(center - eye, up),
             MatrixFromTranslation(-eye));
}

struct Pose  // Value type representing a rigid transformation consisting of a
             // translation and rotation component
{
  float3 position;
  float4 orientation;

  Pose(const float3& p, const float4& q) : position(p), orientation(q) {}
  Pose() : Pose({0, 0, 0}, {0, 0, 0, 1}) {}

  Pose inverse() const {
    auto q = qconj(orientation);
    return {qrot(q, -position), q};
  }
  float4x4 matrix() const {
    return MatrixFromRotationTranslation(orientation, position);
  }

  float3 operator*(const float3& point) const {
    return position + qrot(orientation, point);
  }
  Pose operator*(const Pose& pose) const {
    return {*this * pose.position, qmul(orientation, pose.orientation)};
  }
  float4 TransformPlane(const float4& p) {
    float3 n = qrot(orientation, p.xyz());
    return float4(n, p.w - dot(position, n));
  }
};

namespace linalg {
// Implement stream operators for vector types, and specifically interpret
// byte-sized integers as integers instead of characters
template <class T, int M>
std::ostream& operator<<(std::ostream& out, const vec<T, M>& v) {
  for (int i = 0; i < M; ++i)
    out << (i ? " " : "") << v[i];
  return out;
}
template <int M>
std::ostream& operator<<(std::ostream& out, const vec<int8_t, M>& v) {
  for (int i = 0; i < M; ++i)
    out << (i ? " " : "") << (int)v[i];
  return out;
}
template <int M>
std::ostream& operator<<(std::ostream& out, const vec<uint8_t, M>& v) {
  for (int i = 0; i < M; ++i)
    out << (i ? " " : "") << (int)v[i];
  return out;
}
template <class T, int M>
std::istream& operator>>(std::istream& in, vec<T, M>& v) {
  for (int i = 0; i < M; ++i)
    in >> v[i];
  return in;
}
template <int M>
std::istream& operator>>(std::istream& in, vec<int8_t, M>& v) {
  for (int i = 0, x; i < M; ++i)
    if (in >> x)
      v[i] = (int8_t)x;
  return in;
}
template <int M>
std::istream& operator>>(std::istream& in, vec<uint8_t, M>& v) {
  for (int i = 0, x; i < M; ++i)
    if (in >> x)
      v[i] = (uint8_t)x;
  return in;
}
}  // namespace linalg

inline std::ostream& operator<<(std::ostream& out, const Pose& p) {
  return out << p.position << " " << p.orientation;
}
inline std::istream& operator>>(std::istream& in, Pose& p) {
  return in >> p.position >> p.orientation;
}

inline float3 PlaneLineIntersection(
    const float3& n,
    const float d,
    const float3& p0,
    const float3&
        p1)  // returns the point where the line p0-p2 intersects the plane n&d
{
  float3 dif = p1 - p0;
  float dn = dot(n, dif);
  float t = -(d + dot(n, p0)) / dn;
  return p0 + (dif * t);
}
inline float3 PlaneLineIntersection(const float4& plane,
                                    const float3& p0,
                                    const float3& p1) {
  return PlaneLineIntersection(plane.xyz(), plane.w, p0, p1);
}  // returns the point where the line p0-p2 intersects the plane n&d

inline float LineProjectTime(const float3& p0,
                             const float3& p1,
                             const float3& a) {
  // project point a on segment [p0,p1]
  float3 d = p1 - p0;
  float t = dot(d, (a - p0)) / dot(d, d);
  return t;
}
inline float3 LineProject(const float3& p0, const float3& p1, const float3& a) {
  return p0 + (p1 - p0) * LineProjectTime(p0, p1, a);
}

inline float3 gradient(const float3& v0,
                       const float3& v1,
                       const float3& v2,
                       const float t0,
                       const float t1,
                       const float t2) {
  float3 e0 = v1 - v0;
  float3 e1 = v2 - v0;
  float d0 = t1 - t0;
  float d1 = t2 - t0;
  float3 pd = e1 * d0 - e0 * d1;
  if (pd == float3(0, 0, 0)) {
    return float3(0, 0, 1);
  }
  pd = normalize(pd);
  if (fabsf(d0) > fabsf(d1)) {
    e0 = e0 + pd * -dot(pd, e0);
    e0 = e0 * (1.0f / d0);
    return e0 * (1.0f / dot(e0, e0));
    ;
  }
  // else
  // assert(fabsf(d0) <= fabsf(d1));
  e1 = e1 + pd * -dot(pd, e1);
  e1 = e1 * (1.0f / d1);
  return e1 * (1.0f / dot(e1, e1));
}

inline float3 BaryCentric(const float3& v0,
                          const float3& v1,
                          const float3& v2,
                          float3 s) {
  float3x3 m(v0, v1, v2);
  if (determinant(m) == 0) {
    int k = (length(v1 - v2) > length(v0 - v2)) ? 1 : 0;
    float t = LineProjectTime(v2, m[k], s);
    return float3((1 - k) * t, k * t, 1 - t);
  }
  return mul(inverse(m), s);
}
inline bool tri_interior(const float3& v0,
                         const float3& v1,
                         const float3& v2,
                         const float3& d) {
  float3 b = BaryCentric(v0, v1, v2, d);
  return (b.x >= 0.0f && b.y >= 0.0f && b.z >= 0.0f);
}

inline float3 ProjectOntoPlane(const float4& plane, const float3& v) {
  return v - plane.xyz() * dot(plane, float4(v, 1));
}

inline float3 PlaneProjectOf(const float3& v0,
                             const float3& v1,
                             const float3& v2,
                             const float3& point) {
  float3 cp = cross(v2 - v0, v2 - v1);
  float dtcpm = -dot(cp, v0);
  float cpm2 = dot(cp, cp);
  if (cpm2 == 0.0f) {
    return LineProject(v0, (length(v1 - v0) > length(v2 - v0)) ? v1 : v2,
                       point);
  }
  return point - cp * (dot(cp, point) + dtcpm) / cpm2;
}

inline int maxdir(const float3* p,
                  int count,
                  const float3& dir)  // returns index
{
  assert(count > 0);
  if (count == 0)
    return -1;
  return std::max_element(p, p + count,
                          [dir](const float3& a, const float3& b) {
                            return dot(a, dir) < dot(b, dir);
                          }) -
         p;
}
inline int maxdir_index(const std::vector<float3>& points,
                        const float3& dir)  // returns index
{
  return maxdir(points.data(), points.size(), dir);
}
inline float3 maxdir_value(const std::vector<float3>& points,
                           const float3& dir)  // returns index
{
  return points[maxdir_index(points, dir)];
}

inline float3 TriNormal(const float3& v0,
                        const float3& v1,
                        const float3& v2)  // normal of the triangle with vertex
                                           // positions v0, v1, and v2
{
  float3 cp = cross(v1 - v0, v2 - v1);
  float m = length(cp);
  if (m == 0)
    return float3(0, 0, 1);
  return cp * (1.0f / m);
}

inline float4 plane_of(
    const float3& v0,
    const float3& v1,
    const float3& v2)  // plane of triangle with vertex positions v0, v1, and v2
{
  auto n = TriNormal(v0, v1, v2);
  return {n, -dot(n, v0)};
}
inline float4 PolyPlane(const std::vector<float3>& verts) {
  float4 p(0, 0, 0, 0);
  float3 c(0, 0, 0);
  for (const auto& v : verts)
    c += v * (1.0f / verts.size());
  for (unsigned int i = 0; i < verts.size(); i++)
    p.xyz() += cross(verts[i] - c, verts[(i + 1) % verts.size()] - c);
  if (p == float4(0, 0, 0, 0))
    return p;
  p.xyz() = normalize(p.xyz());
  p.w = -dot(c, p.xyz());
  return p;
}
struct HitInfo {
  bool hit;
  float3 impact;
  float3 normal;
  operator bool() { return hit; };
};

inline HitInfo PolyHitCheck(const std::vector<float3>& verts,
                            const float4& plane,
                            const float3& v0,
                            const float3& v1) {
  float d0 = dot(float4(v0, 1), plane);
  float d1 = dot(float4(v1, 1), plane);
  HitInfo hitinfo = {((d0 > 0) && (d1 < 0)),
                     {0, 0, 0},
                     {0, 0, 0}};  // if segment crosses into plane
  hitinfo.normal = plane.xyz();
  hitinfo.impact =
      v0 +
      (v1 - v0) * d0 / (d0 - d1);  //  if both points on plane this will be 0/0,
                                   //  if parallel you might get infinity
  for (unsigned int i = 0; hitinfo && i < verts.size(); i++)
    hitinfo.hit =
        hitinfo && (determinant(float3x3(verts[(i + 1) % verts.size()] - v0,
                                         verts[i] - v0, v1 - v0)) >=
                    0);  // use v0,v1 winding instead of impact to prevent mesh
                         // edge tunneling
  return hitinfo;
}
inline HitInfo PolyHitCheck(const std::vector<float3>& verts,
                            const float3& v0,
                            const float3& v1) {
  return PolyHitCheck(verts, PolyPlane(verts), v0, v1);
}

inline HitInfo ConvexHitCheck(const std::vector<float4>& planes,
                              float3 v0,
                              const float3& v1_) {
  float3 v1 = v1_;
  float3 n;
  for (auto plane : planes) {
    float d0 = dot(float4(v0, 1), plane);
    float d1 = dot(float4(v1, 1), plane);
    if (d0 >= 0 && d1 >= 0)            // segment above plane
      return {false, v1_, {0, 0, 0}};  // hitinfo;
    if (d0 <= 0 && d1 <= 0)
      continue;  //  start and end point under plane
    auto c = v0 + (v1 - v0) * d0 / (d0 - d1);
    if (d0 >= 0) {
      n = plane.xyz();
      v0 = c;
    } else
      v1 = c;
  }
  return {true, v0, n};
}
inline HitInfo ConvexHitCheck(const std::vector<float4>& planes,
                              const Pose& pose,
                              float3 v0,
                              const float3& v1) {
  auto h = ConvexHitCheck(planes, pose.inverse() * v0, pose.inverse() * v1);
  return {h.hit, pose * h.impact, qrot(pose.orientation, h.normal)};
}

inline int argmax(const float a[], int count)  // returns index
{
  if (count == 0)
    return -1;
  return std::max_element(a, a + count) - a;
}

// still in the process of rearranging basic math and geom routines, putting
// these here for now...

inline float3 Orth(const float3& v) {
  float3 absv = abs(v);
  float3 u(1, 1, 1);
  u[argmax(&absv[0], 3)] = 0.0f;
  return normalize(cross(u, v));
}
inline float4 quat_from_to(
    const float3& v0_,
    const float3&
        v1_)  // shortest arc quat from game programming gems 1 section 2.10
{
  auto v0 = normalize(
      v0_);  // Comment these two lines out if you know its not needed.
  auto v1 =
      normalize(v1_);  // If vector is already unit length then why do it again?
  auto c = cross(v0, v1);
  auto d = dot(v0, v1);
  if (d <= -1.0f) {
    float3 a = Orth(v0);
    return float4(a.x, a.y, a.z, 0);
  }  // 180 about any orthogonal axis
  auto s = sqrtf((1 + d) * 2);
  return {c.x / s, c.y / s, c.z / s, s / 2.0f};
}

inline float4 VirtualTrackBall(const float3& cop,
                               const float3& cor,
                               const float3& dir1,
                               const float3& dir2) {
  // Simple track ball functionality to spin stuf on the screen.
  //  cop   center of projection   cor   center of rotation
  //  dir1  old mouse direction    dir2  new mouse direction
  // Pretend there is a sphere around cor.    Take rotation
  // between apprx points where dir1 and dir2 intersect sphere.
  float3 nrml = cor - cop;  // compute plane
  float fudgefactor =
      1.0f / (length(nrml) *
              0.25f);  // since trackball proportional to distance from cop
  nrml = normalize(nrml);
  float dist = -dot(nrml, cor);
  float3 u =
      (PlaneLineIntersection(nrml, dist, cop, cop + dir1) - cor) * fudgefactor;
  float m = length(u);
  u = (m > 1) ? u / m : u - (nrml * sqrtf(1 - m * m));
  float3 v =
      (PlaneLineIntersection(nrml, dist, cop, cop + dir2) - cor) * fudgefactor;
  m = length(v);
  v = (m > 1) ? v / m : v - (nrml * sqrtf(1 - m * m));
  return quat_from_to(u, v);
}

template <class T, int N>
inline std::pair<linalg::vec<T, N>, linalg::vec<T, N> > Extents(
    const std::vector<linalg::vec<T, N> >& verts) {
  linalg::vec<T, N> bmin(std::numeric_limits<T>::max()),
      bmax(std::numeric_limits<T>::lowest());
  for (auto v : verts) {
    bmin = min(bmin, v);
    bmax = max(bmax, v);
  }
  return std::make_pair(
      bmin,
      bmax);  // typical useage:   std::tie(mymin,mymax) = Extents(myverts);
}

inline float Volume(const float3* vertices, const int3* tris, const int count) {
  // count is the number of triangles (tris)
  float volume = 0;
  for (int i = 0; i < count; i++)  // for each triangle
  {
    volume += determinant(
        float3x3(vertices[tris[i][0]], vertices[tris[i][1]],
                 vertices[tris[i][2]]));  // divide by 6 later for efficiency
  }
  return volume / 6.0f;  // since the determinant give 6 times tetra volume
}

inline float3 CenterOfMass(const float3* vertices,
                           const int3* tris,
                           const int count) {
  // count is the number of triangles (tris)
  float3 com(0, 0, 0);
  float volume = 0;                // actually accumulates the volume*6
  for (int i = 0; i < count; i++)  // for each triangle
  {
    float3x3 A(vertices[tris[i][0]], vertices[tris[i][1]],
               vertices[tris[i][2]]);
    float vol = determinant(A);      // dont bother to divide by 6
    com += vol * (A.x + A.y + A.z);  // divide by 4 at end
    volume += vol;
  }
  com /= volume * 4.0f;
  return com;
}
inline float3x3 Inertia(const float3* vertices,
                        const int3* tris,
                        const int count,
                        const float3& com) {
  // count is the number of triangles (tris)
  // The moments are calculated based on the center of rotation (com) which
  // defaults to [0,0,0] if unsupplied assume mass==1.0  you can multiply by
  // mass later. for improved accuracy the next 3 variables, the determinant d,
  // and its calculation should be changed to double
  float volume = 0;  // technically this variable accumulates the volume times 6
  float3 diag(0, 0,
              0);  // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
  float3 offd(0, 0,
              0);  // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]
  for (int i = 0; i < count; i++)  // for each triangle
  {
    float3x3 A(vertices[tris[i][0]] - com, vertices[tris[i][1]] - com,
               vertices[tris[i][2]] -
                   com);  // matrix trick for volume calc by taking determinant
    float d =
        determinant(A);  // vol of tiny parallelapiped= d * dr * ds * dt (the 3
                         // partials of my tetral triple integral equasion)
    volume += d;  // add vol of current tetra (note it could be negative -
                  // that's ok we need that sometimes)
    for (int j = 0; j < 3; j++) {
      int j1 = (j + 1) % 3;
      int j2 = (j + 2) % 3;
      diag[j] += (A[0][j] * A[1][j] + A[1][j] * A[2][j] + A[2][j] * A[0][j] +
                  A[0][j] * A[0][j] + A[1][j] * A[1][j] + A[2][j] * A[2][j]) *
                 d;  // divide by 60.0f later;
      offd[j] +=
          (A[0][j1] * A[1][j2] + A[1][j1] * A[2][j2] + A[2][j1] * A[0][j2] +
           A[0][j1] * A[2][j2] + A[1][j1] * A[0][j2] + A[2][j1] * A[1][j2] +
           A[0][j1] * A[0][j2] * 2 + A[1][j1] * A[1][j2] * 2 +
           A[2][j1] * A[2][j2] * 2) *
          d;  // divide by 120.0f later
    }
  }
  diag /=
      volume *
      (60.0f / 6.0f);  // divide by total volume (vol/6) since density=1/volume
  offd /= volume * (120.0f / 6.0f);
  return {{diag.y + diag.z, -offd.z, -offd.y},
          {-offd.z, diag.x + diag.z, -offd.x},
          {-offd.y, -offd.x, diag.x + diag.y}};
}

inline float3 Diagonal(const float3x3& m) {
  return {m.x.x, m.y.y, m.z.z};
}

inline float4 Diagonalizer(const float3x3& A) {
  // A must be a symmetric matrix.
  // returns orientation of the principle axes.
  // returns quaternion q such that its corresponding column major matrix Q
  // can be used to Diagonalize A
  // Diagonal matrix D = transpose(Q) * A * (Q);  thus  A == Q*D*QT
  // The directions of q (cols of Q) are the eigenvectors D's diagonal is the
  // eigenvalues As per 'col' convention if float3x3 Q = qgetmatrix(q); then Q*v
  // = q*v*conj(q)
  int maxsteps = 24;  // certainly wont need that many.
  int i;
  float4 q(0, 0, 0, 1);
  for (i = 0; i < maxsteps; i++) {
    float3x3 Q = qmat(q);                       // Q*v == q*v*conj(q)
    float3x3 D = mul(transpose(Q), A, Q);       // A = Q*D*Q^T
    float3 offdiag(D[1][2], D[0][2], D[0][1]);  // elements not on the diagonal
    float3 om(fabsf(offdiag.x), fabsf(offdiag.y),
              fabsf(offdiag.z));  // mag of each offdiag elem
    int k = (om.x > om.y && om.x > om.z)
                ? 0
                : (om.y > om.z) ? 1 : 2;  // index of largest element of offdiag
    int k1 = (k + 1) % 3;
    int k2 = (k + 2) % 3;
    if (offdiag[k] == 0.0f)
      break;  // diagonal already
    float thet = (D[k2][k2] - D[k1][k1]) / (2.0f * offdiag[k]);
    float sgn = (thet > 0.0f) ? 1.0f : -1.0f;
    thet *= sgn;  // make it positive
    float t =
        sgn / (thet + ((thet < 1.E6f) ? sqrtf(thet * thet + 1.0f)
                                      : thet));  // sign(T)/(|T|+sqrt(T^2+1))
    float c = 1.0f / sqrtf(t * t + 1.0f);        //  c= 1/(t^2+1) , t=s/c
    if (c == 1.0f)
      break;  // no room for improvement - reached machine precision.
    float4 jr(0, 0, 0, 0);  // jacobi rotation for this iteration.
    jr[k] =
        sgn *
        sqrtf((1.0f - c) /
              2.0f);  // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
    jr[k] *= -1.0f;   // note we want a final result semantic that takes D to A,
                      // not A to D
    jr.w = sqrtf(1.0f - (jr[k] * jr[k]));
    if (jr.w == 1.0f)
      break;  // reached limits of floating point precision
    q = qmul(q, jr);
    q = normalize(q);
  }
  float h = 1.0f / sqrtf(2.0f);  // M_SQRT2
  auto e = [&q, &A]() {
    return Diagonal(mul(transpose(qmat(q)), A, qmat(q)));
  };  // current ordering of eigenvals of q
  q = (e().x < e().z) ? qmul(q, float4(0, h, 0, h)) : q;
  q = (e().y < e().z) ? qmul(q, float4(h, 0, 0, h)) : q;
  q = (e().x < e().y) ? qmul(q, float4(0, 0, h, h))
                      : q;  // size order z,y,x so xy spans a planeish spread
  q = (qzdir(q).z < 0) ? qmul(q, float4(1, 0, 0, 0)) : q;
  q = (qydir(q).y < 0) ? qmul(q, float4(0, 0, 1, 0)) : q;
  q = (q.w < 0) ? -q : q;
  auto M = mul(transpose(qmat(q)), A, qmat(q));  // to test result
  return q;
}

inline float Diagonalizer(
    const float2x2& m)  // returns angle that rotates m into diagonal matrix d
                        // where d01==d10==0 and d00>d11 (the eigenvalues)
{
  float d = m.y.y - m.x.x;
  return atan2f(d + sqrtf(d * d + 4.0f * m.x.y * m.y.x), 2.0f * m.x.y);
}

inline void PlaneTranslate(float4& plane, const float3& translation) {
  plane.w -= dot(plane.xyz(), translation);
}
inline void PlaneRotate(float4& plane, const float4& rotation) {
  plane.xyz() = qrot(rotation, plane.xyz());
}
inline void PlaneScale(float4& plane, const float3& scaling) {
  plane.xyz() = plane.xyz() / scaling;
  plane /= length(plane.xyz());
}
inline void PlaneScale(float4& plane, float scaling) {
  plane.w *= scaling;
}

inline std::vector<int2> boxedges() {
  std::vector<int2> a;
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < i; j++)
      if (!((i ^ j) & ((i ^ j) - 1)))
        a.push_back({i, j});
  return a;
}  // the 12 indexed edges of box or cube

inline std::vector<float3x2> DeIndex(const std::vector<float3>& v,
                                     const std::vector<int2>& t) {
  return Transform(t, [&v](int2 t) { return float3x2(v[t[0]], v[t[1]]); });
}  // return 3x2 is pair of vertices m[0] and m[1].  eg line segment
inline std::vector<float3x3> DeIndex(const std::vector<float3>& v,
                                     const std::vector<int3>& t) {
  return Transform(
      t, [&v](int3 t) { return float3x3(v[t[0]], v[t[1]], v[t[2]]); });
}  // return 3x3 is pair of vertices m[0] and m[1].  eg triangle

inline std::pair<Pose, float3> PrincipalAxes(
    const std::vector<float3>&
        points)  // returns principal axes as a pose and population's variance
                 // along pose's local x,y,z
{
  float3 com(0, 0, 0);
  float3x3 cov;
  for (auto p : points)
    com += p;
  com /= (float)points.size();
  for (auto p : points)
    cov += outerprod(p - com, p - com);
  cov /= (float)points.size();
  auto q = Diagonalizer(cov);
  return std::make_pair<Pose, float3>(
      {com, q}, Diagonal(mul(transpose(qmat(q)), cov, qmat(q))));
}

#endif  // GEOMETRIC_H
