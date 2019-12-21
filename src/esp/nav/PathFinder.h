// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <string>
#include <vector>

#include "esp/core/esp.h"

// forward declarations
class dtNavMesh;
class dtNavMeshQuery;
class dtQueryFilter;
class dtQueryPathState;

namespace esp {
// forward declaration
namespace assets {
struct MeshData;
}
namespace nav {

struct HitRecord {
  vec3f hitPos;
  vec3f hitNormal;
  float hitDist;
};

namespace impl {
struct ActionSpaceGraph;
class IslandSystem;
}  // namespace impl

struct ShortestPath {
  vec3f requestedStart;
  vec3f requestedEnd;
  std::vector<vec3f> points;
  float geodesicDistance;
  ESP_SMART_POINTERS(ShortestPath)
};

struct MultiGoalShortestPath {
  vec3f requestedStart;
  std::vector<vec3f> requestedEnds;

  std::vector<vec3f> points;
  float geodesicDistance;

  ESP_SMART_POINTERS(MultiGoalShortestPath)
};

struct NavMeshSettings {
  //! Cell size in world units
  float cellSize;
  //! Cell height in world units
  float cellHeight;
  //! Agent height in world units
  float agentHeight;
  //! Agent radius in world units
  float agentRadius;
  //! Agent max climb in world units
  float agentMaxClimb;
  //! Agent max slope in degrees
  float agentMaxSlope;
  //! Region minimum size in voxels. regionMinSize = sqrt(regionMinArea)
  float regionMinSize;
  //! Region merge size in voxels. regionMergeSize = sqrt(regionMergeArea)
  float regionMergeSize;
  //! Edge max length in world units
  float edgeMaxLen;
  //! Edge max error in voxels
  float edgeMaxError;
  float vertsPerPoly;
  //! Detail sample distance in voxels
  float detailSampleDist;
  //! Detail sample max error in voxel heights.
  float detailSampleMaxError;
  //! Bounds of the area to mesh
  vec3f navMeshBMin;
  vec3f navMeshBMax;

  bool filterLowHangingObstacles;
  bool filterLedgeSpans;
  bool filterWalkableLowHeightSpans;

  void setDefaults() {
    cellSize = 0.05f;
    cellHeight = 0.2f;
    agentHeight = 1.5f;
    agentRadius = 0.1f;
    agentMaxClimb = 0.2f;
    agentMaxSlope = 45.0f;
    regionMinSize = 20;
    regionMergeSize = 20;
    edgeMaxLen = 12.0f;
    edgeMaxError = 1.3f;
    vertsPerPoly = 6.0f;
    detailSampleDist = 6.0f;
    detailSampleMaxError = 1.0f;
    filterLowHangingObstacles = true;
    filterLedgeSpans = true;
    filterWalkableLowHeightSpans = true;
  }

  ESP_SMART_POINTERS(NavMeshSettings)
};

class PathFinder : public std::enable_shared_from_this<PathFinder> {
 public:
  PathFinder();
  ~PathFinder() {
    free();
    LOG(INFO) << "Deconstructing PathFinder";
  }

  bool build(const NavMeshSettings& bs,
             const float* verts,
             const int nverts,
             const int* tris,
             const int ntris,
             const float* bmin,
             const float* bmax);
  bool build(const NavMeshSettings& bs, const esp::assets::MeshData& mesh);

  vec3f getRandomNavigablePoint();

  bool findPath(ShortestPath& path);
  bool findPath(MultiGoalShortestPath& path);

  template <typename T>
  T tryStep(const T& start, const T& end);

  template <typename T>
  T snapPoint(const T& pt);

  bool loadNavMesh(const std::string& path);

  bool saveNavMesh(const std::string& path);

  void free();

  bool isLoaded() { return navMesh_ != nullptr; }

  void seed(uint32_t newSeed);

  float islandRadius(const vec3f& pt) const;

  float distanceToClosestObstacle(const vec3f& pt,
                                  const float maxSearchRadius = 2.0) const;
  HitRecord closestObstacleSurfacePoint(
      const vec3f& pt,
      const float maxSearchRadius = 2.0) const;

  bool isNavigable(const vec3f& pt, const float maxYDelta = 0.5) const;

  std::pair<vec3f, vec3f> bounds() const { return bounds_; }

  friend impl::ActionSpaceGraph;

 protected:
  bool initNavQuery();
  void removeZeroAreaPolys();
  std::vector<vec3f> prevEnds;

  impl::IslandSystem* islandSystem_ = nullptr;

  dtNavMesh* navMesh_;
  dtNavMeshQuery* navQuery_;
  dtQueryFilter* filter_;
  std::pair<vec3f, vec3f> bounds_;
  ESP_SMART_POINTERS(PathFinder)
};

}  // namespace nav
}  // namespace esp
