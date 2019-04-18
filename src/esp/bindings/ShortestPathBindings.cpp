// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/OpaqueTypes.h"

#include "esp/agent/Agent.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"

namespace py = pybind11;
using namespace py::literals;
using namespace esp::nav;

void initShortestPathBindings(py::module& m) {
  py::class_<HitRecord>(m, "HitRecord")
      .def(py::init())
      .def_readwrite("hit_pos", &HitRecord::hitPos)
      .def_readwrite("hit_normal", &HitRecord::hitNormal)
      .def_readwrite("hit_dist", &HitRecord::hitDist);

  py::class_<ActionSpacePathLocation, ActionSpacePathLocation::ptr>(
      m, "ActionSpacePathLocation")
      .def(py::init(&ActionSpacePathLocation::create<>))
      .def(py::init(&ActionSpacePathLocation::create<const esp::vec3f&,
                                                     const esp::vec4f&>))
      .def_readwrite("position", &ActionSpacePathLocation::pos_)
      .def_readwrite("rotation", &ActionSpacePathLocation::rotation_);

  py::bind_vector<std::vector<esp::nav::ActionSpacePathLocation::ptr>>(
      m, "VectorActionSpacePathLocation");

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
      .def_readwrite("requested_ends", &MultiGoalShortestPath::requestedEnds)
      .def_readwrite("points", &MultiGoalShortestPath::points)
      .def_readwrite("geodesic_distance",
                     &MultiGoalShortestPath::geodesicDistance);

  py::class_<PathFinder, PathFinder::ptr>(m, "PathFinder")
      .def(py::init(&PathFinder::create<>))
      .def("get_random_navigable_point", &PathFinder::getRandomNavigablePoint)
      .def("find_path", py::overload_cast<ShortestPath&>(&PathFinder::findPath),
           "path"_a)
      .def("find_path",
           py::overload_cast<MultiGoalShortestPath&>(&PathFinder::findPath),
           "path"_a)
      .def("try_step", &PathFinder::tryStep, R"()", "start"_a, "end"_a)
      .def("island_radius", &PathFinder::islandRadius, R"()", "pt"_a)
      .def_property_readonly("is_loaded", &PathFinder::isLoaded)
      .def("distance_to_closest_obstacle",
           &PathFinder::distanceToClosestObstacle,
           R"(Returns the distance to the closest obstacle.
           If this distance is greater than :py:attr:`max_search_radius`,
           :py:attr:`max_search_radius` is returned instead.)",
           "pt"_a, "max_search_radius"_a = 2.0)
      .def(
          "closest_obstacle_surface_point",
          &PathFinder::closestObstacleSurfacePoint,
          R"(Returns the hit_pos, hit_normal, and hit_dist of the surface point on the closest obstacle.
           If the returned hit_dist is equal to :py:attr:`max_search_radius`,
           no obstacle was found.)",
          "pt"_a, "max_search_radius"_a = 2.0)
      .def("is_navigable", &PathFinder::isNavigable,
           R"(Checks to see if the agent can stand at the specified point.
          To check navigability, the point is snapped to the nearest polygon and
          then the snapped point is compared to the original point.
          Any amount of x-z translation indicates that the given point is not navigable.
          The amount of y-translation allowed is specified by max_y_delta to account
          for slight differences in floor height)",
           "pt"_a, "max_y_delta"_a = 0.5);

  py::class_<ActionSpaceShortestPath, ActionSpaceShortestPath::ptr>(
      m, "ActionSpaceShortestPath")
      .def(py::init(&ActionSpaceShortestPath::create<>))
      .def_readwrite("requested_start",
                     &ActionSpaceShortestPath::requestedStart)
      .def_readwrite("requested_end", &ActionSpaceShortestPath::requestedEnd)
      .def_readwrite("points", &ActionSpaceShortestPath::points)
      .def_readwrite("rotations", &ActionSpaceShortestPath::rotations)
      .def_readwrite("geodesic_distance",
                     &ActionSpaceShortestPath::geodesicDistance)
      .def_readwrite("actions", &ActionSpaceShortestPath::actions);

  py::class_<MultiGoalActionSpaceShortestPath,
             MultiGoalActionSpaceShortestPath::ptr>(
      m, "MultiGoalActionSpaceShortestPath")
      .def(py::init(&MultiGoalActionSpaceShortestPath::create<>))
      .def_readwrite("requested_start",
                     &MultiGoalActionSpaceShortestPath::requestedStart)
      .def_readwrite("requested_ends",
                     &MultiGoalActionSpaceShortestPath::requestedEnds)
      .def_readwrite("points", &MultiGoalActionSpaceShortestPath::points)
      .def_readwrite("rotations", &MultiGoalActionSpaceShortestPath::rotations)
      .def_readwrite("geodesic_distance",
                     &MultiGoalActionSpaceShortestPath::geodesicDistance)
      .def_readwrite("actions", &MultiGoalActionSpaceShortestPath::actions);

  py::class_<ActionSpacePathFinder, ActionSpacePathFinder::ptr>(
      m, "ActionSpacePathFinder")
      .def(py::init(&ActionSpacePathFinder::create<
                    PathFinder::ptr&, const esp::agent::AgentConfiguration&,
                    const esp::scene::ObjectControls&, const esp::vec4f&>))
      .def("find_next_action_along_path",
           py::overload_cast<ActionSpaceShortestPath&>(
               &ActionSpacePathFinder::findNextActionAlongPath))
      .def("find_next_action_along_path",
           py::overload_cast<MultiGoalActionSpaceShortestPath&>(
               &ActionSpacePathFinder::findNextActionAlongPath))
      .def("find_path", py::overload_cast<ActionSpaceShortestPath&>(
                            &ActionSpacePathFinder::findPath))
      .def("find_path", py::overload_cast<MultiGoalActionSpaceShortestPath&>(
                            &ActionSpacePathFinder::findPath))
      .def_property(
          "close_to_obstacle_cost_multiplier",
          py::overload_cast<>(
              &ActionSpacePathFinder::closeToObstacleCostMultiplier,
              py::const_),
          py::overload_cast<const double>(
              &ActionSpacePathFinder::closeToObstacleCostMultiplier),
          R"(Cost multiplier used to discourage the shortest path from being close to obstacles

            If the distance to the closest obstacle is less than :py:attr:`padding_radius` after an action is taken,
            the cost for taking that action is multiplied by :py:attr:`close_to_obstacle_cost_multiplier`
            )")
      .def_property(
          "padding_radius",
          py::overload_cast<>(&ActionSpacePathFinder::paddingRadius,
                              py::const_),
          py::overload_cast<const double>(
              &ActionSpacePathFinder::paddingRadius),
          R"(Defines the distance to the nearest obstacle at which taking an action results in the cost being multiplied
            by :py:attr:`close_to_obstacle_cost_multiplier`
            )");
}
