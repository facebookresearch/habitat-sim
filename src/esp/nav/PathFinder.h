// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_NAV_PATHFINDER_H_
#define ESP_NAV_PATHFINDER_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/Math/Vector3.h>
#include <string>
#include <vector>

#include "esp/core/Esp.h"
#include "esp/core/EspEigen.h"

namespace esp {
// forward declaration
namespace assets {
struct MeshData;
}

//! NavMesh namespace
namespace nav {

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

class PathFinder;

/**
 * @brief Struct for recording closest obstacle information.
 */
struct HitRecord {
  //! World position of the closest obstacle.
  Magnum::Vector3 hitPos;
  //! Normal of the navmesh at the obstacle in xz plane.
  Magnum::Vector3 hitNormal;
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
  Magnum::Vector3 requestedStart;

  /**
   * @brief The ending point for the path
   */
  Magnum::Vector3 requestedEnd;

  /**
   * @brief A list of points that specify the shortest path on the navigation
   * mesh between @ref requestedStart and @ref requestedEnd
   *
   * @note Will be empty if no path exists
   */
  std::vector<Magnum::Vector3> points;

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
  Magnum::Vector3 requestedStart;

  /**
   * @brief Set the list of desired potential end points
   */
  void setRequestedEnds(const std::vector<Magnum::Vector3>& newEnds);

  const std::vector<Magnum::Vector3>& getRequestedEnds() const;

  /**
   * @brief A list of points that specify the shortest path on the navigation
   * mesh between @ref requestedStart and the closest (by geodesic distance)
   * point in @ref requestedEnds
   *
   * Will be empty if no path exists
   */
  std::vector<Magnum::Vector3> points;

  /**
   * @brief The geodesic distance
   *
   * Will be inf if no path exists
   */
  float geodesicDistance{};

  /**
   * @brief The index of the closest end point corresponding to end of the
   * shortest path.
   *
   * Will be -1 if no path exists.
   */
  int closestEndPointIndex{};

  friend class PathFinder;

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(MultiGoalShortestPath)
};

/**
 * @brief Configuration structure for NavMesh generation with recast.
 *
 * Passed to @ref PathFinder::build to construct the NavMesh.
 *
 * Serialized with saved .navmesh files for later equivalency checks upon
 * re-load.
 *
 * See examples/tutorials/notebooks/ECCV_2020_Navigation.py and
 * http://digestingduck.blogspot.com/2009/08/recast-settings-uncovered.html for
 * more details on configuring these parameters for your use case.
 */
struct NavMeshSettings {
  /**
   * @brief XZ-plane cell size in world units. Size of square voxel sides in XZ.
   */
  float cellSize{};

  /**
   * @brief Y-axis cell height in world units. Voxel height.
   */
  float cellHeight{};

  /**
   * @brief Minimum floor to 'ceiling' height that will still allow the floor
   * area to be considered unobstructed in world units.
   *
   * Will be rounded up to a multiple of cellHeight.
   */
  float agentHeight{};

  /**
   * @brief Agent radius in world units.
   *
   * The distance to erode/shrink the walkable area of the heightfield away from
   * obstructions. Will be rounded up to a multiple of cellSize.
   */
  float agentRadius{};

  /**
   * @brief Maximum ledge height that is considered to be traversable in world
   * units.
   *
   * For example, this constrains the maximum step height for traversing
   * stairways. Will be truncated to a multiple of cellHeight.
   */
  float agentMaxClimb{};

  /** @brief The maximum slope that is considered walkable in degrees. */
  float agentMaxSlope{};

  /**
   * @brief Region minimum size in voxels.
   *
   * regionMinSize = sqrt(regionMinArea) The minimum number of cells allowed to
   * form isolated island areas.
   */
  float regionMinSize{};

  /**
   * @brief Region merge size in voxels.
   *
   * regionMergeSize = sqrt(regionMergeArea) Any 2-D regions with a smaller span
   * (cell count) will, if possible, be merged with larger regions.
   */
  float regionMergeSize{};

  /**
   * @brief Edge max length in world units.
   *
   * The maximum allowed length for contour edges along the border of the mesh.
   *
   * Extra vertices will be inserted as needed to keep contour edges below this
   * length. A value of zero effectively disables this feature.
   *
   * A good value for edgeMaxLen is something like agenRadius*8. Will be rounded
   * to a multiple of cellSize.
   */
  float edgeMaxLen{};

  /**
   * @brief The maximum distance a simplfied contour's border edges should
   * deviate the original raw contour.
   *
   * Good values are between 1.1-1.5 (1.3 usually yield good results). More
   * results in jaggies, less cuts corners.
   */
  float edgeMaxError{};

  /**
   * @brief The maximum number of vertices allowed for polygons generated during
   * the contour to polygon conversion process. [Limit: >= 3]
   */
  float vertsPerPoly{};

  /**
   * @brief Detail sample distance in voxels.
   *
   * Sets the sampling distance to use when generating the detail mesh. (For
   * height detail only.)
   *
   * [Limits: 0 or >= 0.9] [x cell_size]
   */
  float detailSampleDist{};

  /**
   * @brief Detail sample max error in voxel heights.
   *
   * The maximum distance the detail mesh surface should deviate from
   * heightfield data. (For height detail only.)
   *
   * [Limit: >=0] [x cell_height]
   */
  float detailSampleMaxError{};

  /**
   * @brief Marks navigable spans as non-navigable if the clearence above the
   * span is less than the specified height.
   */
  bool filterLowHangingObstacles{};

  /**
   * @brief Marks spans that are ledges as non-navigable.
   *
   * This filter reduces the impact of the overestimation of conservative
   * voxelization so the resulting mesh will not have regions hanging in the air
   * over ledges.
   */
  bool filterLedgeSpans{};

  /**
   * @brief Marks navigable spans as non-navigable if the clearence above the
   * span is less than the specified height.
   *
   * Allows the formation of navigable regions that will flow over low lying
   * objects such as curbs, and up structures such as stairways.
   */
  bool filterWalkableLowHeightSpans{};

  /**
   * @brief Whether or not to include STATIC RigidObjects as NavMesh
   * constraints. Note: Used in Simulator recomputeNavMesh pre-process.
   */
  bool includeStaticObjects{};

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
    includeStaticObjects = false;
  }

  //! Load the settings from a JSON file
  void readFromJSON(const std::string& jsonFile);

  //! Save the settings to a JSON file
  void writeToJSON(const std::string& jsonFile) const;

  NavMeshSettings() { setDefaults(); }

  ESP_SMART_POINTERS(NavMeshSettings)
};

/** @brief Check equivalency of @ref NavMeshSettings objects.
 *
 * Checks for equivalency of (or < eps 1e-5 distance between) each parameter.
 */
bool operator==(const NavMeshSettings& a, const NavMeshSettings& b);

/** @brief Check non-equivalency of @ref NavMeshSettings objects with
 * nav::operator==.
 */
bool operator!=(const NavMeshSettings& a, const NavMeshSettings& b);

/** @brief Loads and/or builds a navigation mesh and then allows point sampling,
 * path finding, collision, and island queries on that navmesh.
 *
 * Also allows export of the navmesh data.
 *
 * Powered by integration with [Recast Navigation |
 * Detour](https://masagroup.github.io/recastdetour/).
 *
 * A navigation mesh (NavMesh) is a collection of two-dimensional convex
 * polygons (i.e., a polygon mesh) that define which areas of an environment are
 * traversable by an agent with a particular embodiement. In other words, an
 * agent could freely navigate around within these areas unobstructed by
 * objects, walls, gaps, overhangs, or other barriers that are part of the
 * environment. Adjacent polygons are connected to each other in a graph
 * enabling efficient pathfinding algorithms to chart routes between points on
 * the NavMesh.
 *
 * Using a NavMesh approximation of navigability, an agent is embodied as a
 * rigid cylinder aligned with the gravity direction. The NavMesh is then
 * computed by voxelizing the static scene and generating polygons on the top
 * surfaces of solid voxels where the cylinder would sit without intersection or
 * overhanging and respecting configured constraints such as maximum climbable
 * slope and step-height.
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
   * @brief Returns a random navigable point.
   *
   *  @param[in] maxTries The maximum number of tries sampling will be retried
   * if it fails.
   *  @param[in] islandIndex Optionally specify the island from which to sample
   * the point. Default -1 queries the full navmesh.
   *
   * @return A random navigable point or NAN if none found.
   *
   * @note This method can fail.  If it does,
   * the returned point will be `{NAN, NAN, NAN}`. Use @ref
   * isNavigable to check if the point is navigable.
   */
  Magnum::Vector3 getRandomNavigablePoint(int maxTries = 10,
                                          int islandIndex = ID_UNDEFINED);

  /**
   * @brief Returns a random navigable point within a specified radius about a
   * given point.
   *
   *  @param[in] circleCenter The center of the spherical sample region.
   *  @param[in] radius The spherical sample radius.
   *  @param[in] maxTries The maximum number of tries sampling will be retried
   * if it fails.
   *  @param[in] islandIndex Optionally specify the island from which to sample
   * the point. Default -1 queries the full navmesh.
   *
   * @return A random navigable point or NAN if none found.
   *
   * @note This method can fail.  If it does,
   * the returned point will be `{NAN, NAN, NAN}`. Use @ref
   * isNavigable to check if the point is navigable.
   */
  Magnum::Vector3 getRandomNavigablePointAroundSphere(
      const Magnum::Vector3& circleCenter,
      float radius,
      int maxTries = 10,
      int islandIndex = ID_UNDEFINED);

  /**
   * @brief Finds the shortest path between two points on the navigation mesh
   *
   * For this method to succeed, both end points must exist on the same navmesh
   * island.
   *
   * @param[inout] path The @ref ShortestPath structure contain the starting and
   * end point. This method will populate the @ref ShortestPath.points and @ref
   * ShortestPath.geodesicDistance fields.
   *
   * @return Whether or not a path exists between @ref
   * ShortestPath.requestedStart and @ref ShortestPath.requestedEnd.
   */
  bool findPath(ShortestPath& path);

  /**
   * @brief Finds the shortest path from a start point to the closest (by
   * geoddesic distance) end point.
   *
   * For this method to succeed, start point and one end point must exist on the
   * same navmesh island.
   *
   * @param[inout] path The @ref MultiGoalShortestPath structure contain the
   * start point and list of possible end points. This method will populate the
   * @ref  MultiGoalShortestPath.points and @ref
   * MultiGoalShortestPath.geodesicDistance fields.
   *
   * @return Whether or not a path exists between @ref
   * MultiGoalShortestPath.requestedStart and any @ref
   * MultiGoalShortestPath.requestedEnds.
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
   * @return The found end location.
   */
  template <typename T>
  T tryStep(const T& start, const T& end);

  /**
   * @brief Same as @ref tryStep but does not allow for sliding along walls
   */
  template <typename T>
  T tryStepNoSliding(const T& start, const T& end);

  /**
   * @brief Snaps a point to the navigation mesh.
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
   * Snaps the point to the navmesh and then queries the island at the snap
   * point.
   *
   * When the input point is not assumed to be on the NavMesh, it is possible
   * that the snapped point is quite far from the query point. In this case,
   * also consider checking @ref isNavigable or check distance to the snap point
   * to validate.
   *
   * @param[in] pt The point to check against the navigation mesh island system.
   *
   * @return The island for the point or ID_UNDEFINED (-1) if failed.
   */
  template <typename T>
  int getIsland(const T& pt);

  /**
   * @brief Loads a navigation meshed saved by @ref saveNavMesh
   *
   * Also imports serialized @ref NavMeshSettings if available.
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
   * Also serializes @ref NavMeshSettings into the file.
   *
   * @param[in] path The name of the file, generally has extension ``.navmesh``
   *
   * @return Whether or not the navmesh was successfully saved
   */
  bool saveNavMesh(const std::string& path);

  /**
   * @return If a valid navigation mesh is currently loaded or not.
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
   * @brief returns a heuristic for the size of the connected component that
   * @ref pt belongs to.
   *
   * The point will be snapped to the NavMesh and the resulting island radius
   * returned.
   *
   * @param[in] pt The point which specifies the connected component in
   * question.
   *
   * @return Heuristic size of the connected component.
   */
  float islandRadius(const Magnum::Vector3& pt) const;

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
  float distanceToClosestObstacle(const Magnum::Vector3& pt,
                                  float maxSearchRadius = 2.0) const;

  /**
   * @brief Same as @ref distanceToClosestObstacle but returns additional
   * information.
   */
  HitRecord closestObstacleSurfacePoint(const Magnum::Vector3& pt,
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
  bool isNavigable(const Magnum::Vector3& pt, float maxYDelta = 0.5) const;

  /**
   * Compute and return the total area of all NavMesh polygons.
   *
   * Values are pre-computed and cached for constant time access.
   *
   * @param[in] islandIndex Optionally limit results to a specific island.
   * Default -1 queries all islands.
   */
  float getNavigableArea(int islandIndex = ID_UNDEFINED) const;

  /**
   * @return The axis aligned bounding box containing the navigation mesh.
   */
  std::pair<Magnum::Vector3, Magnum::Vector3> bounds() const;

  /**
   * @brief Get a 2D grid marking navigable and non-navigable cells at a
   * specified height and resolution.
   *
   * The size of the grid depends on the navmesh bounds and selected resolution.
   *
   * Can be further processed by Habitat-lab utilities. See
   * examples/tutorials/notebooks/ECCV_Navigation.ipynb for details.
   *
   * @param metersPerPixel size of the discrete grid cells. Controls grid
   * resolution.
   * @param height The vertical height of the 2D slice.
   * @param eps Sets allowable epsilon meter Y offsets from the configured
   * height value.
   *
   * @return The 2D grid marking cells as navigable or not.
   */
  MatrixXb getTopDownView(float metersPerPixel, float height, float eps = 0.5);

  /**
   * @brief Get a 2D grid marking island index for navigable cells and -1 for
   * non-navigable cells at a specified height and resolution.
   *
   * The size of the grid depends on the navmesh bounds and selected resolution.
   *
   * @param metersPerPixel size of the discrete grid cells. Controls grid
   * resolution.
   * @param height The vertical height of the 2D slice.
   * @param eps Sets allowable epsilon meter Y offsets from the configured
   * height value.
   *
   * @return The 2D grid marking cell islands or -1 for not navigable.
   */
  MatrixXi getTopDownIslandView(float metersPerPixel,
                                float height,
                                float eps = 0.5);

  /**
   * @brief Returns a MeshData object containing triangulated NavMesh polys.
   *
   * Theobject is generated and stored if this is the first query for constant
   * time access in subsequent calls.
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
