// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <Corrade/Containers/OptionalPythonBindings.h>
#include <Magnum/Math/Vector3.h>

#include "esp/assets/MeshData.h"
#include "esp/core/Esp.h"
#include "esp/nav/GreedyFollower.h"
#include "esp/nav/PathFinder.h"

namespace py = pybind11;
namespace Mn = Magnum;

using py::literals::operator""_a;

namespace esp {
namespace nav {

void initShortestPathBindings(py::module& m) {
  py::class_<HitRecord>(m, "HitRecord",
                        R"(Struct for recording closest obstacle information.)")
      .def(py::init())
      .def_readwrite("hit_pos", &HitRecord::hitPos,
                     R"(World position of the closest obstacle.)")
      .def_readwrite("hit_normal", &HitRecord::hitNormal,
                     R"(Normal of the navmesh at the obstacle in xz plane.)")
      .def_readwrite(
          "hit_dist", &HitRecord::hitDist,
          R"(Distance from query point to closest obstacle. Inf if no valid point was found.)");

  py::class_<ShortestPath, ShortestPath::ptr>(
      m, "ShortestPath",
      R"(Struct for shortest path finding. Used in conjunction with PathFinder.findPath().)")
      .def(py::init(&ShortestPath::create<>))
      .def_readwrite("requested_start", &ShortestPath::requestedStart,
                     R"(The starting point for the path.)")
      .def_readwrite("requested_end", &ShortestPath::requestedEnd,
                     R"(The ending point for the path.)")
      .def_readwrite(
          "points", &ShortestPath::points,
          R"(A list of points that specify the shortest path on the navigation mesh between requestedStart and requestedEnd. Will be empty if no path exists.)")
      .def_readwrite(
          "geodesic_distance", &ShortestPath::geodesicDistance,
          R"(The geodesic distance between requestedStart and requestedEnd. Will be inf if no path exists.)");

  py::class_<MultiGoalShortestPath, MultiGoalShortestPath::ptr>(
      m, "MultiGoalShortestPath",
      R"(Struct for multi-goal shortest path finding. Used in conjunction with PathFinder.findPath().)")
      .def(py::init(&MultiGoalShortestPath::create<>))
      .def_readwrite("requested_start", &MultiGoalShortestPath::requestedStart,
                     R"(The starting point for the path.)")
      .def_property("requested_ends", &MultiGoalShortestPath::getRequestedEnds,
                    &MultiGoalShortestPath::setRequestedEnds,
                    R"(The list of desired potential end points.)")
      .def_readwrite(
          "points", &MultiGoalShortestPath::points,
          R"(A list of points that specify the shortest path on the navigation mesh between requestedStart and the closest (by geodesic distance) point in requestedEnds. Will be empty if no path exists.)")
      .def_readwrite(
          "geodesic_distance", &MultiGoalShortestPath::geodesicDistance,
          R"(The total geodesic distance of the path. Will be inf if no path exists.)")
      .def_readwrite(
          "closest_end_point_index",
          &MultiGoalShortestPath::closestEndPointIndex,
          R"(The index of the closest end point corresponding to end of the shortest path. Will be -1 if no path exists.)");

  py::class_<NavMeshSettings, NavMeshSettings::ptr>(
      m, "NavMeshSettings",
      R"(Configuration structure for NavMesh generation with recast. Passed to PathFinder::build to construct the NavMesh. Serialized with saved .navmesh files for later equivalency checks upon re-load.)")
      .def(py::init(&NavMeshSettings::create<>))
      .def_readwrite(
          "cell_size", &NavMeshSettings::cellSize,
          R"(XZ-plane cell size in world units. Size of square voxel sides in XZ.)")
      .def_readwrite("cell_height", &NavMeshSettings::cellHeight,
                     R"(Y-axis cell height in world units. Voxel height.)")
      .def_readwrite(
          "agent_height", &NavMeshSettings::agentHeight,
          R"(Minimum floor to 'ceiling' height that will still allow the floor area to be considered unobstructed in world units. Will be rounded up to a multiple of cellHeight.)")
      .def_readwrite(
          "agent_radius", &NavMeshSettings::agentRadius,
          R"(Agent radius in world units. The distance to erode/shrink the walkable area of the heightfield away from obstructions. Will be rounded up to a multiple of cellSize.)")
      .def_readwrite(
          "agent_max_climb", &NavMeshSettings::agentMaxClimb,
          R"(Maximum ledge height that is considered to be traversable in world units (e.g. for stair steps). Will be truncated to a multiple of cellHeight.)")
      .def_readwrite(
          "agent_max_slope", &NavMeshSettings::agentMaxSlope,
          R"(The maximum slope that is considered walkable in degrees.)")
      .def_readwrite(
          "region_min_size", &NavMeshSettings::regionMinSize,
          R"(Region minimum size in voxels. regionMinSize = sqrt(regionMinArea) The minimum number of cells allowed to form isolated island areas.)")
      .def_readwrite(
          "region_merge_size", &NavMeshSettings::regionMergeSize,
          R"(Region merge size in voxels. regionMergeSize = sqrt(regionMergeArea) Any 2-D regions with a smaller span (cell count) will, if possible, be merged with larger regions.)")
      .def_readwrite(
          "edge_max_len", &NavMeshSettings::edgeMaxLen,
          R"(Edge max length in world units. The maximum allowed length for contour edges along the border of the mesh. Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature. A good value for edgeMaxLen is something like agentRadius*8. Will be rounded to a multiple of cellSize.)")
      .def_readwrite(
          "edge_max_error", &NavMeshSettings::edgeMaxError,
          R"(The maximum distance a simplified contour's border edges should deviate the original raw contour. Good values are between 1.1-1.5 (1.3 usually yield good results). More results in jaggies, less cuts corners.)")
      .def_readwrite(
          "verts_per_poly", &NavMeshSettings::vertsPerPoly,
          R"(The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process. [Limit: >= 3])")
      .def_readwrite(
          "detail_sample_dist", &NavMeshSettings::detailSampleDist,
          R"(Detail sample distance in voxels. Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >= 0.9] [x cell_size])")
      .def_readwrite(
          "detail_sample_max_error", &NavMeshSettings::detailSampleMaxError,
          R"(Detail sample max error in voxel heights. The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.) [Limit: >=0] [x cell_height])")
      .def_readwrite(
          "filter_low_hanging_obstacles",
          &NavMeshSettings::filterLowHangingObstacles,
          R"(Marks navigable spans as non-navigable if the clearance above the span is less than the specified height. Default True.)")
      .def_readwrite(
          "filter_ledge_spans", &NavMeshSettings::filterLedgeSpans,
          R"(Marks spans that are ledges as non-navigable. This filter reduces the impact of the overestimation of conservative voxelization so the resulting mesh will not have regions hanging in the air over ledges. Default True.)")
      .def_readwrite(
          "filter_walkable_low_height_spans",
          &NavMeshSettings::filterWalkableLowHeightSpans,
          R"(Marks navigable spans as non-navigable if the clearance above the span is less than the specified height. Allows the formation of navigable regions that will flow over low lying objects such as curbs, and up structures such as stairways. Default True.)")
      .def_readwrite(
          "include_static_objects", &NavMeshSettings::includeStaticObjects,
          R"(Whether or not to include STATIC RigidObjects as NavMesh constraints. Note: Used in Simulator recomputeNavMesh pre-process. Default False.)")
      .def("set_defaults", &NavMeshSettings::setDefaults)
      .def("read_from_json", &NavMeshSettings::readFromJSON,
           R"(Overwrite these settings with values from a JSON file.)")
      .def("write_to_json", &NavMeshSettings::writeToJSON,
           R"(Write these settings to a JSON file.)")
      .def(
          py::self == py::self,
          R"(Checks for equivalency of (or < eps 1e-5 distance between) each parameter.)")
      .def(py::self != py::self);

  py::class_<PathFinder, PathFinder::ptr>(
      m, "PathFinder",
      R"(Loads and/or builds a navigation mesh and then allows point sampling, path finding, collision, and island queries on that navmesh. See PathFinder C++ API docs for more details.)")
      .def(py::init(&PathFinder::create<>))
      .def(
          "get_bounds", &PathFinder::bounds,
          R"(Get the axis aligned bounding box containing the navigation mesh.)")
      .def(
          "seed", &PathFinder::seed,
          R"(Seed the pathfinder.  Useful for get_random_navigable_point(). Seeds the global c rand function.)")
      .def(
          "get_topdown_view", &PathFinder::getTopDownView,
          R"(Returns the topdown view of the PathFinder's navmesh at a given vertical slice with eps slack.)",
          "meters_per_pixel"_a, "height"_a, "eps"_a = 0.5)
      .def(
          "get_topdown_island_view", &PathFinder::getTopDownIslandView,
          R"(Returns the topdown view of the PathFinder's navmesh with island indices at each point or -1 for non-navigable cells for a given vertical slice with eps slack.)",
          "meters_per_pixel"_a, "height"_a, "eps"_a = 0.5)
      // detailed docs in docs/docs.rst
      .def("get_random_navigable_point", &PathFinder::getRandomNavigablePoint,
           "max_tries"_a = 10, "island_index"_a = ID_UNDEFINED)
      .def(
          "get_random_navigable_point_near",
          &PathFinder::getRandomNavigablePointAroundSphere, "circle_center"_a,
          "radius"_a, "max_tries"_a = 100, "island_index"_a = ID_UNDEFINED,
          R"(Returns a random navigable point within a specified radius about a given point. Optionally specify the island from which to sample the point. Default -1 queries the full navmesh.)")
      .def(
          "find_path", py::overload_cast<ShortestPath&>(&PathFinder::findPath),
          "path"_a,
          R"(Finds the shortest path between two points on the navigation mesh using ShortestPath module. Path variable is filled if successful. Returns boolean success.)")
      .def(
          "find_path",
          py::overload_cast<MultiGoalShortestPath&>(&PathFinder::findPath),
          "path"_a,
          R"(Finds the shortest path between a start point and the closest of a set of end points (in geodesic distance) on the navigation mesh using MultiGoalShortestPath module. Path variable is filled if successful. Returns boolean success.)")
      .def("try_step", &PathFinder::tryStep<Magnum::Vector3>, "start"_a,
           "end"_a)
      .def("try_step", &PathFinder::tryStep<Magnum::Vector3>, "start"_a,
           "end"_a)
      .def("try_step_no_sliding",
           &PathFinder::tryStepNoSliding<Magnum::Vector3>, "start"_a, "end"_a)
      .def("try_step_no_sliding",
           &PathFinder::tryStepNoSliding<Magnum::Vector3>, "start"_a, "end"_a)
      .def("snap_point", &PathFinder::snapPoint<Magnum::Vector3>, "point"_a,
           "island_index"_a = ID_UNDEFINED)
      .def("snap_point", &PathFinder::snapPoint<Magnum::Vector3>, "point"_a,
           "island_index"_a = ID_UNDEFINED)
      .def(
          "get_island", &PathFinder::getIsland<Magnum::Vector3>, "point"_a,
          R"(Query the island closest to a point. Snaps the point to the NavMesh first, so check the snap distance also if unsure.)")
      .def(
          "get_island", &PathFinder::getIsland<Magnum::Vector3>, "point"_a,
          R"(Query the island closest to a point. Snaps the point to the NavMesh first, so check the snap distance also if unsure.)")
      .def(
          "island_radius",
          [](PathFinder& self, const Magnum::Vector3& pt) {
            return self.islandRadius(pt);
          },
          "pt"_a,
          R"(Given a point, snaps to an island and gets a heuristic of island size: the radius of a circle containing all NavMesh polygons within the specified island.)")
      .def(
          "island_radius",
          [](PathFinder& self, int islandIndex) {
            return self.islandRadius(islandIndex);
          },
          "island_index"_a,
          R"(Given an island index, gets a heuristic of island size: the radius of a circle containing all NavMesh polygons within the specified island.)")
      .def_property_readonly(
          "num_islands", &PathFinder::numIslands,
          R"(The number of connected components making up the navmesh.)")
      .def_property_readonly(
          "is_loaded", &PathFinder::isLoaded,
          R"(Whether a valid navigation mesh is currently loaded or not.)")
      .def_property_readonly(
          "navigable_area",
          [](PathFinder& self) { return self.getNavigableArea(ID_UNDEFINED); },
          R"(The total area of all NavMesh polygons.)")
      .def(
          "island_area", &PathFinder::getNavigableArea,
          "island_index"_a = ID_UNDEFINED,
          R"(The total area of all NavMesh polygons within the specified island.)")
      .def(
          "build_navmesh_vertices",
          [](PathFinder& self, int islandIndex) {
            return self.getNavMeshData(islandIndex)->vbo;
          },
          "island_index"_a = ID_UNDEFINED,
          R"(Returns an array of vertex data for the triangulated NavMesh polys. Optionally limit results to a specific island. Default (island_index==-1) queries all islands.)")
      .def(
          "build_navmesh_vertex_indices",
          [](PathFinder& self, int islandIndex) {
            return self.getNavMeshData(islandIndex)->ibo;
          },
          "island_index"_a = ID_UNDEFINED,
          R"(Returns an array of triangle index data for the triangulated NavMesh poly vertices returned by build_navmesh_vertices(). Optionally limit results to a specific island. Default (island_index==-1) queries all islands.)")
      .def("load_nav_mesh", &PathFinder::loadNavMesh, "path"_a,
           R"(Load a .navmesh file overriding this PathFinder instance.)")
      .def(
          "save_nav_mesh", &PathFinder::saveNavMesh, "path"_a,
          R"(Serialize this PathFinder instance and current NavMesh settings to a .navmesh file.)")
      .def("distance_to_closest_obstacle",
           &PathFinder::distanceToClosestObstacle,
           R"(Returns the distance to the closest obstacle.)", "pt"_a,
           "max_search_radius"_a = 2.0)
      // detailed docs in docs/docs.rst
      .def("closest_obstacle_surface_point",
           &PathFinder::closestObstacleSurfacePoint,
           R"(Returns the hit_pos, hit_normal and hit_dist of the surface point
          on the closest obstacle.)",
           "pt"_a, "max_search_radius"_a = 2.0)
      .def("is_navigable", &PathFinder::isNavigable,
           R"(Checks to see if the agent can stand at the specified point.)",
           "pt"_a, "max_y_delta"_a = 0.5)
      .def_property_readonly("nav_mesh_settings",
                             &PathFinder::getNavMeshSettings,
                             R"(The settings for the current NavMesh.)");

  // this enum is used by GreedyGeodesicFollowerImpl so it needs to be defined
  // before it
  py::enum_<GreedyGeodesicFollowerImpl::CODES>(m, "GreedyFollowerCodes")
      .value("ERROR", GreedyGeodesicFollowerImpl::CODES::ERROR)
      .value("STOP", GreedyGeodesicFollowerImpl::CODES::STOP)
      .value("FORWARD", GreedyGeodesicFollowerImpl::CODES::FORWARD)
      .value("LEFT", GreedyGeodesicFollowerImpl::CODES::LEFT)
      .value("RIGHT", GreedyGeodesicFollowerImpl::CODES::RIGHT);

  py::bind_vector<std::vector<GreedyGeodesicFollowerImpl::CODES>>(
      m, "VectorGreedyCodes");

  py::class_<GreedyGeodesicFollowerImpl, GreedyGeodesicFollowerImpl::ptr>(
      m, "GreedyGeodesicFollowerImpl")
      .def(py::init(&GreedyGeodesicFollowerImpl::create<
                    PathFinder::ptr&, GreedyGeodesicFollowerImpl::MoveFn&,
                    GreedyGeodesicFollowerImpl::MoveFn&,
                    GreedyGeodesicFollowerImpl::MoveFn&, double, double, double,
                    bool, int>))
      .def("next_action_along",
           py::overload_cast<const Mn::Quaternion&, const Mn::Vector3&,
                             const Mn::Vector3&>(
               &GreedyGeodesicFollowerImpl::nextActionAlong),
           py::return_value_policy::move)
      .def("next_action_along",
           py::overload_cast<const core::RigidState&, const Mn::Vector3&>(
               &GreedyGeodesicFollowerImpl::nextActionAlong),
           py::return_value_policy::move)
      .def("find_path",
           py::overload_cast<const Mn::Quaternion&, const Mn::Vector3&,
                             const Mn::Vector3&>(
               &GreedyGeodesicFollowerImpl::findPath),
           py::return_value_policy::move)
      .def("find_path",
           py::overload_cast<const core::RigidState&, const Mn::Vector3&>(
               &GreedyGeodesicFollowerImpl::findPath),
           py::return_value_policy::move)
      .def("reset", &GreedyGeodesicFollowerImpl::reset);
}

}  // namespace nav
}  // namespace esp
