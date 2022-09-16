// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_NAV_PATHFINDER_H_
#define ESP_NAV_PATHFINDER_H_

#include <Corrade/Containers/Optional.h>
#include <string>
#include <vector>

#include "esp/core/Esp.h"

namespace esp {
// forward declaration
namespace assets {
struct MeshData;
}

namespace nav {

class PathFinder;

/**
 * @brief Struct for recording closest obstacle information.
 */
struct HitRecord {
  //! World position of the closest obstacle.
  vec3f hitPos;
  //! Normal of the navmesh at the obstacle in xz plane.
  vec3f hitNormal;
  //! Distance from query point to closest obstacle. Inf if no valid point was
  //! found.
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

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(MultiGoalShortestPath)
};

//! Configuration structure for NavMesh generation with recast.
/**
 * @brief Configuration structure for NavMesh generation with recast.
 *
 * See examples/tutorials/colabs/ECCV_2020_Navigation.py and
 * http://digestingduck.blogspot.com/2009/08/recast-settings-uncovered.html for
 * more details on configuring these parameters for your use case.
 */
struct NavMeshSettings {
  //! XZ-plane cell size in world units. Size of square voxel sides in XZ.
  float cellSize{};
  //! Y-axis cell height in world units. Voxel height.
  float cellHeight{};
  //! Minimum floor to 'ceiling' height that will still allow the floor area to
  //! be considered unobstructed in world units. Will be rounded up to a
  //! multiple of cellHeight.
  float agentHeight{};
  //! Agent radius in world units. The distance to erode/shrink the walkable
  //! area of the heightfield away from obstructions. Will be rounded up to a
  //! multiple of cellSize.
  float agentRadius{};
  //! Maximum ledge height that is considered to be traversable in world units
  //! (e.g. for stair steps). Will be truncated to a multiple of cellHeight.
  float agentMaxClimb{};
  //! The maximum slope that is considered walkable in degrees.
  float agentMaxSlope{};
  //! Region minimum size in voxels. regionMinSize = sqrt(regionMinArea) The
  //! minimum number of cells allowed to form isolated island areas.
  float regionMinSize{};
  //! Region merge size in voxels. regionMergeSize = sqrt(regionMergeArea) Any
  //! 2-D regions with a smaller span (cell count) will, if possible, be merged
  //! with larger regions.
  float regionMergeSize{};
  //! Edge max length in world units. The maximum allowed length for contour
  //! edges along the border of the mesh. Extra vertices will be inserted as
  //! needed to keep contour edges below this length. A value of zero
  //! effectively disables this feature. A good value for edgeMaxLen is
  //! something like agenRadius*8. Will be rounded to a multiple of cellSize.
  float edgeMaxLen{};
  //! The maximum distance a simplfied contour's border edges should deviate the
  //! original raw contour. Good values are between 1.1-1.5 (1.3 usually yield
  //! good results). More results in jaggies, less cuts corners.
  float edgeMaxError{};
  //! The maximum number of vertices allowed for polygons generated during the
  //! contour to polygon conversion process. [Limit: >= 3]
  float vertsPerPoly{};
  //! Detail sample distance in voxels. Sets the sampling distance to use when
  //! generating the detail mesh. (For height detail only.) [Limits: 0 or >=
  //! 0.9] [x cell_size]
  float detailSampleDist{};
  //! Detail sample max error in voxel heights. The maximum distance the detail
  //! mesh surface should deviate from heightfield data. (For height detail
  //! only.) [Limit: >=0] [x cell_height]
  float detailSampleMaxError{};

  //! Marks navigable spans as non-navigable if the clearence above the span is
  //! less than the specified height.
  bool filterLowHangingObstacles{};
  //! Marks spans that are ledges as non-navigable. This filter reduces the
  //! impact of the overestimation of conservative voxelization so the resulting
  //! mesh will not have regions hanging in the air over ledges.
  bool filterLedgeSpans{};
  //! Marks navigable spans as non-navigable if the clearence above the span is
  //! less than the specified height. Allows the formation of navigable regions
  //! that will flow over low lying objects such as curbs, and up structures
  //! such as stairways.
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

bool operator==(const NavMeshSettings& a, const NavMeshSettings& b);
bool operator!=(const NavMeshSettings& a, const NavMeshSettings& b);

/** @brief Loads and/or builds a navigation mesh and then performs path
 * finding and collision queries on that navmesh.
 */
class PathFinder {
 public:
  /**
   * @brief Constructor.
   */
  PathFinder();
  ~PathFinder() = default;

  /**
   * @brief Construct a NavMesh from @ref NavMeshSettings and mesh data
   * pointers.
   *
   * @param bs Parameter settings for NavMesh construction.
   * @param verts Vertex array of the mesh.
   * @param nverts Number of verts in the array.
   * @param tris Index array of the mesh triangles.
   * @param ntris Number of triangle indices in the array.
   * @param bmin Navigable region bounding box min corner. Could be mesh bb or
   * user defined override for subset of the mesh.
   * @param bmax Navigable region bounding box max corner. Could be mesh bb or
   * user defined override for subset of the mesh.
   *
   * @return Whether or not construction was successful.
   */
  bool build(const NavMeshSettings& bs,
             const float* verts,
             int nverts,
             const int* tris,
             int ntris,
             const float* bmin,
             const float* bmax);

  /**
   * @brief Construct a NavMesh from @ref NavMeshSettings and a @ref MeshData
   * object.
   *
   * @param bs Parameter settings for NavMesh construction.
   * @param mesh A joined mesh for which to compute a navmesh.
   *
   * @return Whether or not construction was successful.
   */
  bool build(const NavMeshSettings& bs, const esp::assets::MeshData& mesh);

  /**
   * @brief Returns a random navigable point
   *
   *  @param[in] maxTries The maximum number of tries sampling will be retried
   * if it fails.
   *  @param[in] islandIndex Optionally specify the island from which to sample
   * the point. Default -1 queries the full navmesh.
   *
   * @return A random navigable point.
   *
   * @note This method can fail.  If it does,
   * the returned point will be `{NAN, NAN, NAN}`. Use @ref
   * isNavigable to check if the point is navigable.
   */
  vec3f getRandomNavigablePoint(int maxTries = 10,
                                int islandIndex = ID_UNDEFINED);

  vec3f getRandomNavigablePointAroundSphere(const vec3f& circleCenter,
                                            float radius,
                                            int maxTries = 10,
                                            int islandIndex = ID_UNDEFINED);

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
   * @param[in] islandIndex Optionally specify the island from which to sample
   * the point. Default -1 queries the full navmesh.
   *
   * @return The closest navigation point to @ref pt.  Will be `{NAN, NAN, NAN}`
   * if no navigable point was within a reasonable distance
   */
  template <typename T>
  T snapPoint(const T& pt, int islandIndex = ID_UNDEFINED);

  /**
   * @brief Identifies the island closest to a point.
   *
   * @param[in] pt The point to check against the navigation mesh island system

   * @return The island for the point or ID_UNDEFINED (-1) if failed.
   */
  template <typename T>
  int getIsland(const T& pt);

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
   * @brief returns the size of the specified connected component.
   *
   * @param[in] islandIndex The index of the specified connected component
   *
   * @return Size of the connected component
   */
  float islandRadius(int islandIndex) const;

  /**
   * @brief returns the number of connected components making up the navmesh.
   *
   * @return Number of the connected components
   */
  int numIslands() const;

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
                                  float maxSearchRadius = 2.0) const;

  /**
   * @brief Same as @ref distanceToClosestObstacle but returns additional
   * information.
   */
  HitRecord closestObstacleSurfacePoint(const vec3f& pt,
                                        float maxSearchRadius = 2.0) const;

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
  bool isNavigable(const vec3f& pt, float maxYDelta = 0.5) const;

  /**
   * Compute and return the total area of all NavMesh polygons.
   *
   * @param[in] islandIndex Optionally limit results to a specific island.
   * Default -1 queries all islands.
   */
  float getNavigableArea(int islandIndex = ID_UNDEFINED) const;

  /**
   * @return The axis aligned bounding box containing the navigation mesh.
   */
  std::pair<vec3f, vec3f> bounds() const;

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getTopDownView(
      float metersPerPixel,
      float height);

  /**
   * @brief Returns a MeshData object containing triangulated NavMesh polys. The
   * object is generated and stored if this is the first query.
   *
   * Does nothing if the PathFinder is not loaded.
   *
   * @param[in] islandIndex Optionally limit results to a specific island.
   * Default -1 queries all islands.
   *
   * @return The object containing triangulated NavMesh polys.
   */
  std::shared_ptr<assets::MeshData> getNavMeshData(
      int islandIndex = ID_UNDEFINED);

  /**
   * @brief Return the settings for the current NavMesh.
   */
  Corrade::Containers::Optional<NavMeshSettings> getNavMeshSettings() const;

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(PathFinder)
};

}  // namespace nav
}  // namespace esp

#endif  // ESP_NAV_PATHFINDER_H_
