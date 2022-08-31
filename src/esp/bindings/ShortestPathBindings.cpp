// Copyright (c) Facebook, Inc. and its affiliates.
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
#include "esp/scene/ObjectControls.h"

namespace py = pybind11;
namespace Mn = Magnum;

using py::literals::operator""_a;

namespace esp {
namespace nav {

void initShortestPathBindings(py::module& m) {
  py::class_<HitRecord>(m, "HitRecord")
      .def(py::init())
      .def_readwrite("hit_pos", &HitRecord::hitPos)
      .def_readwrite("hit_normal", &HitRecord::hitNormal)
      .def_readwrite("hit_dist", &HitRecord::hitDist);

  py::class_<ShortestPath, ShortestPath::ptr>(m, "ShortestPath")
      .def(py::init(&ShortestPath::create<>))
      .def_readwrite("requested_start", &ShortestPath::requestedStart)
      .def_readwrite("requested_end", &ShortestPath::requestedEnd)
      .def_readwrite("points", &ShortestPath::points)
      .def_readwrite("geodesic_distance", &ShortestPath::geodesicDistance);

  py::class_<MultiGoalShortestPath, MultiGoalShortestPath::ptr>(
      m, "MultiGoalShortestPath")
      .def(py::init(&MultiGoalShortestPath::create<>))
      .def_readwrite("requested_start", &MultiGoalShortestPath::requestedStart)
      .def_property("requested_ends", &MultiGoalShortestPath::getRequestedEnds,
                    &MultiGoalShortestPath::setRequestedEnds)
      .def_readwrite("points", &MultiGoalShortestPath::points)
      .def_readwrite("geodesic_distance",
                     &MultiGoalShortestPath::geodesicDistance);

  py::class_<NavMeshSettings, NavMeshSettings::ptr>(m, "NavMeshSettings")
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
          R"(Edge max length in world units. The maximum allowed length for contour edges along the border of the mesh. Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature. A good value for edgeMaxLen is something like agenRadius*8. Will be rounded to a multiple of cellSize.)")
      .def_readwrite(
          "edge_max_error", &NavMeshSettings::edgeMaxError,
          R"(The maximum distance a simplfied contour's border edges should deviate the original raw contour. Good values are between 1.1-1.5 (1.3 usually yield good results). More results in jaggies, less cuts corners.)")
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
          R"(Marks navigable spans as non-navigable if the clearence above the span is less than the specified height. Default True.)")
      .def_readwrite(
          "filter_ledge_spans", &NavMeshSettings::filterLedgeSpans,
          R"(Marks spans that are ledges as non-navigable. This filter reduces the impact of the overestimation of conservative voxelization so the resulting mesh will not have regions hanging in the air over ledges. Default True.)")
      .def_readwrite(
          "filter_walkable_low_height_spans",
          &NavMeshSettings::filterWalkableLowHeightSpans,
          R"(Marks navigable spans as non-navigable if the clearence above the span is less than the specified height. Allows the formation of navigable regions that will flow over low lying objects such as curbs, and up structures such as stairways. Default True.)")
      .def("set_defaults", &NavMeshSettings::setDefaults)
      .def(py::self == py::self)
      .def(py::self != py::self);

  py::class_<PathFinder, PathFinder::ptr>(m, "PathFinder")
      .def(py::init(&PathFinder::create<>))
      .def("get_bounds", &PathFinder::bounds)
      .def("seed", &PathFinder::seed)
      .def("get_topdown_view", &PathFinder::getTopDownView,
           R"(Returns the topdown view of the PathFinder's navmesh.)",
           "meters_per_pixel"_a, "height"_a)
      .def("get_random_navigable_point", &PathFinder::getRandomNavigablePoint,
           "max_tries"_a = 10)
      .def("get_random_navigable_point_near",
           &PathFinder::getRandomNavigablePointAroundSphere, "circle_center"_a,
           "radius"_a, "max_tries"_a = 100)
      .def("find_path", py::overload_cast<ShortestPath&>(&PathFinder::findPath),
           "path"_a)
      .def("find_path",
           py::overload_cast<MultiGoalShortestPath&>(&PathFinder::findPath),
           "path"_a)
      .def("try_step", &PathFinder::tryStep<Magnum::Vector3>, "start"_a,
           "end"_a)
      .def("try_step", &PathFinder::tryStep<vec3f>, "start"_a, "end"_a)
      .def("try_step_no_sliding",
           &PathFinder::tryStepNoSliding<Magnum::Vector3>, "start"_a, "end"_a)
      .def("try_step_no_sliding", &PathFinder::tryStepNoSliding<vec3f>,
           "start"_a, "end"_a)
      .def("snap_point", &PathFinder::snapPoint<Magnum::Vector3>)
      .def("snap_point", &PathFinder::snapPoint<vec3f>)
      .def("island_radius", &PathFinder::islandRadius, "pt"_a)
      .def_property_readonly("is_loaded", &PathFinder::isLoaded)
      .def_property_readonly("navigable_area", &PathFinder::getNavigableArea)
      .def("build_navmesh_vertices",
           [](PathFinder& self) { return self.getNavMeshData()->vbo; })
      .def("build_navmesh_vertex_indices",
           [](PathFinder& self) { return self.getNavMeshData()->ibo; })
      .def("load_nav_mesh", &PathFinder::loadNavMesh, "path"_a)
      .def("save_nav_mesh", &PathFinder::saveNavMesh, "path"_a)
      .def("distance_to_closest_obstacle",
           &PathFinder::distanceToClosestObstacle,
           R"(Returns the distance to the closest obstacle.)", "pt"_a,
           "max_search_radius"_a = 2.0)
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
                             R"(The settings for the current nav mesh)");

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
