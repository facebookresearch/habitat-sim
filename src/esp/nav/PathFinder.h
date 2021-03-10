// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_NAV_PATHFINDER_H_
#define ESP_NAV_PATHFINDER_H_

#include <string>
#include <vector>

#include "esp/core/esp.h"

namespace esp {
// forward declaration
namespace assets {
class MeshData;
}

namespace nav {

class PathFinder;

struct HitRecord {
  vec3f hitPos;
  vec3f hitNormal;
  float hitDist{};
};

/**
 * @brief Struct for shortest path finding. Used in conjunction with @ref
 * PathFinder.findPath
 */
struct ShortestPath {
  /**
   * @brief The starting point for the path
   */
  vec3f requestedStart;

  /**
   * @brief The ending point for the path
   */
  vec3f requestedEnd;

  /**
   * @brief A list of points that specify the shortest path on the navigation
   * mesh between @ref requestedStart and @ref requestedEnd
   *
   * @note Will be empty if no path exists
   */
  std::vector<vec3f> points;

  /**
   * @brief The geodesic distance between @ref requestedStart and @ref
   * requestedEnd
   *
   * @note Will be inf if no path exists
   */
  float geodesicDistance{};

  ESP_SMART_POINTERS(ShortestPath)
};

/**
 * @brief Struct for multi-goal shortest path finding. Used in conjunction with
 * @ref PathFinder.findPath
 */
struct MultiGoalShortestPath {
  MultiGoalShortestPath();

  /**
   * @brief The starting point for the path
   */
  vec3f requestedStart;

  /**
   * @brief Set the list of desired potential end points
   */
  void setRequestedEnds(const std::vector<vec3f>& newEnds);

  const std::vector<vec3f>& getRequestedEnds() const;

  /**
   * @brief A list of points that specify the shortest path on the navigation
   * mesh between @ref requestedStart and the closest (by geodesic distance)
   * point in @ref requestedEnds
   *
   * Will be empty if no path exists
   */
  std::vector<vec3f> points;

  /**
   * @brief The geodesic distance
   *
   * Will be inf if no path exists
   */
  float geodesicDistance{};

  friend class PathFinder;

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(MultiGoalShortestPath);
};

struct NavMeshSettings {
  //! Cell size in world units
  float cellSize{};
  //! Cell height in world units
  float cellHeight{};
  //! Agent height in world units
  float agentHeight{};
  //! Agent radius in world units
  float agentRadius{};
  //! Agent max climb in world units
  float agentMaxClimb{};
  //! Agent max slope in degrees
  float agentMaxSlope{};
  //! Region minimum size in voxels. regionMinSize = sqrt(regionMinArea)
  float regionMinSize{};
  //! Region merge size in voxels. regionMergeSize = sqrt(regionMergeArea)
  float regionMergeSize{};
  //! Edge max length in world units
  float edgeMaxLen{};
  //! Edge max error in voxels
  float edgeMaxError{};
  float vertsPerPoly{};
  //! Detail sample distance in voxels
  float detailSampleDist{};
  //! Detail sample max error in voxel heights.
  float detailSampleMaxError{};
  //! Bounds of the area to mesh
  vec3f navMeshBMin;
  vec3f navMeshBMax;

  bool filterLowHangingObstacles{};
  bool filterLedgeSpans{};
  bool filterWalkableLowHeightSpans{};

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

  NavMeshSettings() { setDefaults(); }

  ESP_SMART_POINTERS(NavMeshSettings)
};

/** Loads and/or builds a navigation mesh and then performs path
 * finding and collision queries on that navmesh
 *
 */
class PathFinder {
 public:
  /**
   * @brief Constructor.
   */
  PathFinder();
  ~PathFinder() = default;

  bool build(const NavMeshSettings& bs,
             const float* verts,
             const int nverts,
             const int* tris,
             const int ntris,
             const float* bmin,
             const float* bmax);
  bool build(const NavMeshSettings& bs, const esp::assets::MeshData& mesh);

  /**
   * @brief Returns a random navigable point
   *
   * @param maxTries[in] The maximum number of tries sampling will be retried if
   * it fails.
   *
   * @return A random navigable point.
   *
   * @note This method can fail.  If it does,
   * the returned point will be `{NAN, NAN, NAN}`. Use @ref
   * isNavigable to check if the point is navigable.
   */
  vec3f getRandomNavigablePoint(int maxTries = 10);

  /**
   * @brief Finds the shortest path between two points on the navigation mesh
   *
   * @param[inout] path The @ref ShortestPath structure contain the starting and
   * end point. This method will populate the @ref ShortestPath.points and @ref
   * ShortestPath.geodesicDistance fields.
   *
   * @return Whether or not a path exists between @ref
   * ShortestPath.requestedStart and @ref ShortestPath.requestedEnd
   */
  bool findPath(ShortestPath& path);

  /**
   * @brief Finds the shortest path from a start point to the closest (by
   * geoddesic distance) end point.
   *
   * @param[inout] path The @ref MultiGoalShortestPath structure contain the
   * start point and list of possible end points. This method will populate the
   * @ref  MultiGoalShortestPath.points and @ref
   * MultiGoalShortestPath.geodesicDistance fields.
   *
   * @return Whether or not a path exists between @ref
   * MultiGoalShortestPath.requestedStart and any @ref
   * MultiGoalShortestPath.requestedEnds
   */
  bool findPath(MultiGoalShortestPath& path);

  /**
   * @brief Attempts to move from @ref start to @ref end and returns the
   * navigable point closest to @ref end that is feasibly reachable from @ref
   * start
   *
   * @param[in] start The starting location
   * @param[out] end The desired end location
   *
   * @return The found end location
   */
  template <typename T>
  T tryStep(const T& start, const T& end);

  /**
   * @brief Same as @ref tryStep but does not allow for sliding along walls
   */
  template <typename T>
  T tryStepNoSliding(const T& start, const T& end);

  /**
   * @brief Snaps a point to the navigation mesh
   *
   * @param[in] pt The point to snap to the navigation mesh
   *
   * @return The closest navigation point to @ref pt.  Will be `{NAN, NAN, NAN}`
   * if no navigable point was within a reasonable distance
   */
  template <typename T>
  T snapPoint(const T& pt);

  /**
   * @brief Loads a navigation meshed saved by @ref saveNavMesh
   *
   * @param[in] path The saved navigation mesh file, generally has extension
   * ``.navmesh``
   *
   * @return Whether or not the navmesh was successfully loaded
   */
  bool loadNavMesh(const std::string& path);

  /**
   * @brief Saves a navigation mesh to later be loaded by @ref loadNavMesh
   *
   * @param[in] path The name of the file, generally has extension ``.navmesh``
   *
   * @return Whether or not the navmesh was successfully saved
   */
  bool saveNavMesh(const std::string& path);

  /**
   * @return If a navigation mesh is current loaded or not
   */
  bool isLoaded() const;

  /**
   * @brief Seed the pathfinder.  Useful for @ref getRandomNavigablePoint
   *
   * @param[in] newSeed The random seed
   *
   * @note This just seeds the global c @ref rand function.
   */
  void seed(uint32_t newSeed);

  /**
   * @brief returns the size of the connected component @ ref pt belongs to.
   *
   * @param[in] pt The point to specify the connected component
   *
   * @return Size of the connected component
   */
  float islandRadius(const vec3f& pt) const;

  /**
   * @brief Finds the distance to the closest non-navigable location
   *
   * @param[in] pt The point to begin searching from
   * @param[in] pt The radius to search in
   *
   * @return The distance to the closest non-navigable location or @ref
   * maxSearchRadius if all locations within @ref maxSearchRadius are navigable
   */
  float distanceToClosestObstacle(const vec3f& pt,
                                  const float maxSearchRadius = 2.0) const;

  /**
   * @brief Same as @ref distanceToClosestObstacle but returns additional
   * information.
   */
  HitRecord closestObstacleSurfacePoint(
      const vec3f& pt,
      const float maxSearchRadius = 2.0) const;

  /**
   * @brief Query whether or not a given location is navigable
   *
   * This method works by snapping @ref pt to the navigation mesh with @ref
   * snapPoint and then checking to see if there was no displacement in the x-z
   * plane and at most @ref maxYDelta displacement in the y direction.
   *
   * @param[in] pt The location to check whether or not it is navigable
   * @param[in] maxYDelta The maximum y displacement.  This tolerance is useful
   * for computing a top-down occupancy grid as the floor is not perfectly level
   *
   * @return Whether or not @ref pt is navigable
   */
  bool isNavigable(const vec3f& pt, const float maxYDelta = 0.5) const;

  /**
   * Compute and return the total area of all NavMesh polygons
   */
  float getNavigableArea() const;

  /**
   * @return The axis aligned bounding box containing the navigation mesh.
   */
  std::pair<vec3f, vec3f> bounds() const;

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getTopDownView(
      const float metersPerPixel,
      const float height);

  /**
   * @brief Returns a MeshData object containing triangulated NavMesh polys. The
   * object is generated and stored if this is the first query.
   *
   * Does nothing if the PathFinder is not loaded.
   *
   * @return The object containing triangulated NavMesh polys.
   */
  const std::shared_ptr<assets::MeshData> getNavMeshData();

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(PathFinder);
};

}  // namespace nav
}  // namespace esp

#endif  // ESP_NAV_PATHFINDER_H_
