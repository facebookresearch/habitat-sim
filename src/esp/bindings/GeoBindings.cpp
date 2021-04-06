// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/geo/OBB.h"
#include "esp/geo/VoxelUtils.h"
#include "esp/geo/VoxelWrapper.h"
#include "esp/geo/geo.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace geo {

void initGeoBindings(py::module& m) {
  auto geo = m.def_submodule("geo");

  geo.attr("UP") = ESP_UP;
  geo.attr("GRAVITY") = ESP_GRAVITY;
  geo.attr("FRONT") = ESP_FRONT;
  geo.attr("BACK") = ESP_BACK;
  geo.attr("LEFT") = ESP_FRONT.cross(ESP_GRAVITY);
  geo.attr("RIGHT") = ESP_FRONT.cross(ESP_UP);

  // ==== OBB ====
  py::class_<OBB>(m, "OBB")
      .def(py::init<const vec3f&, const vec3f&, const quatf&>())
      .def(py::init<box3f&>())
      .def("contains", &OBB::contains)
      .def("closest_point", &OBB::closestPoint)
      .def("distance", &OBB::distance)
      .def("to_aabb", &OBB::toAABB)
      .def_property_readonly("center", &OBB::center)
      .def_property_readonly("sizes", &OBB::sizes)
      .def_property_readonly("half_extents", &OBB::halfExtents)
      .def_property_readonly(
          "rotation", [](const OBB& self) { return self.rotation().coeffs(); })
      .def_property_readonly(
          "local_to_world",
          [](const OBB& self) { return self.localToWorld().matrix(); })
      .def_property_readonly("world_to_local", [](const OBB& self) {
        return self.worldToLocal().matrix();
      });

  geo.def("compute_gravity_aligned_MOBB", &geo::computeGravityAlignedMOBB);
  geo.def("get_transformed_bb", &geo::getTransformedBB, "range"_a, "xform"_a);

  // ==== Ray ====
  py::class_<Ray>(m, "Ray")
      .def(py::init<Magnum::Vector3, Magnum::Vector3>())
      .def(py::init<>())
      .def_readwrite("origin", &Ray::origin)
      .def_readwrite("direction", &Ray::direction);

  // ==== Trajectory utilities ====
  geo.def(
      "build_catmull_rom_spline", &geo::buildCatmullRomTrajOfPoints,
      R"(This function builds an interpolating Catmull-Rom spline through the passed list of
      key points, with num_interpolations interpolated points between each key point, and
      alpha [0,1] deteriming the nature of the spline (default is .5) :
           0.0 is a standard Catmull-Rom spline (which may have cusps)
           0.5 is a centripetal Catmull-Rom spline
           1.0 is a chordal Catmull-Rom spline)",
      "key_points"_a, "num_interpolations"_a, "alpha"_a = .5f);

  // ==== Voxel Utilities ====
  geo.def(
      "generate_interior_exterior_voxel_grid",
      &geo::generateInteriorExteriorVoxelGrid,
      R"(This utility function generates an integer grid where marking boundary
  (0), interior (-inf), and exterior (+inf) cells and registers this grid under
  "InteriorExterior".)",
      "voxelization"_a);
  geo.def(
      "generate_manhattan_distance_sdf", &geo::generateManhattanDistanceSDF,
      R"(This utility function generates a manhattan distance signed distance
  field for a voxelization and registers it under the specified grid name.)",
      "voxelization"_a, "grid_name"_a);
  geo.def(
      "generate_euclidean_distance_sdf", &geo::generateEuclideanDistanceSDF,
      R"(This utility function generates a euclidean distance signed distance
  field for a voxelization and registers it under the specified grid name.)",
      "voxelization"_a, "grid_name"_a);
  geo.def("generate_distance_gradient_field",
          &geo::generateDistanceGradientField,
          R"(Generates a vector field where each vector is the gradient of the
  euclidean signed distance field.)",
          "voxelization"_a, "grid_name"_a);

  // ==== VoxelWrapper ====
  py::class_<VoxelWrapper, VoxelWrapper::ptr>(m, "VoxelWrapper")
      // TODO: Replace with magnum binded Strided Array views (Not sure how to
      // do this yet..)
      .def("set_bool_voxel", &VoxelWrapper::setVoxel<bool>, "index"_a,
           "grid_name"_a, "value"_a,
           R"(Sets a voxel at an index in a specified grid to a value.)")
      .def("set_int_voxel", &VoxelWrapper::setVoxel<int>, "index"_a,
           "grid_name"_a, "value"_a,
           R"(Sets a voxel at an index in a specified grid to a value.)")
      .def("set_float_voxel", &VoxelWrapper::setVoxel<float>, "index"_a,
           "grid_name"_a, "value"_a,
           R"(Sets a voxel at an index in a specified grid to a value.)")
      .def("set_vector3_voxel", &VoxelWrapper::setVoxel<Mn::Vector3>, "index"_a,
           "grid_name"_a, "value"_a,
           R"(Sets a voxel at an index in a specified grid to a value.)")

      .def("get_bool_voxel", &VoxelWrapper::getVoxel<bool>, "index"_a,
           "grid_name"_a, R"(Gets a voxel at an index in a specified grid.)")
      .def("get_int_voxel", &VoxelWrapper::getVoxel<int>, "index"_a,
           "grid_name"_a, R"(Gets a voxel at an index in a specified grid.)")
      .def("get_float_voxel", &VoxelWrapper::getVoxel<float>, "index"_a,
           "grid_name"_a, R"(Gets a voxel at an index in a specified grid.)")
      .def("get_vector3_voxel", &VoxelWrapper::getVoxel<Mn::Vector3>, "index"_a,
           "grid_name"_a, R"(Gets a voxel at an index in a specified grid.)")

      .def("get_bool_grid", &VoxelWrapper::getGrid<bool>, "grid_name"_a,
           R"(Gets a specified bool grid by the grid's name.)")
      .def("get_int_grid", &VoxelWrapper::getGrid<int>, "grid_name"_a,
           R"(Gets a specified int grid by the grid's name.)")
      .def("get_float_grid", &VoxelWrapper::getGrid<float>, "grid_name"_a,
           R"(Gets a specified float grid by the grid's name.)")
      .def("get_vector3_grid", &VoxelWrapper::getGrid<Mn::Vector3>,
           "grid_name"_a,
           R"(Gets a specified Vector3 grid by the grid's name.)")

      .def("add_bool_grid", &VoxelWrapper::addGrid<bool>, "grid_name"_a,
           R"(Creates an empty new bool grid.)")
      .def("add_int_grid", &VoxelWrapper::addGrid<int>, "grid_name"_a,
           R"(Creates an empty new int grid.)")
      .def("add_float_grid", &VoxelWrapper::addGrid<float>, "grid_name"_a,
           R"(Creates an empty new float grid.)")
      .def("add_vector3_grid", &VoxelWrapper::addGrid<Mn::Vector3>,
           "grid_name"_a, R"(Creates an empty new vector3 grid.)")

      .def("remove_grid", &VoxelWrapper::removeGrid, "grid_name"_a,
           R"(Deletes a specified grid and frees its memory.)")

      .def("get_voxel_index_from_global_coords",
           &VoxelWrapper::getVoxelIndexFromGlobalCoords, "coords"_a,
           R"(Gets the voxel index located at a coordinate in world space.)")
      .def("get_global_coords_from_voxel_index",
           &VoxelWrapper::getGlobalCoordsFromVoxelIndex, "coords"_a,
           R"(Gets the world positioning of a particular voxel in a
  voxelization.)")
      .def("get_voxel_grid_dimensions", &VoxelWrapper::getVoxelGridDimensions,
           R"(Returns a numpy array with the
  dimensions of the voxel grid.)")
      .def("get_voxel_size", &VoxelWrapper::getVoxelSize,
           R"(Returns a numpy array with the size of a
  single voxel.)")
      .def("generate_mesh", &VoxelWrapper::generateMesh,
           "grid_name"_a = "Boundary",
           R"(Generates a distance flow vector field, where each vector points
  away from the closest boundary cell.)")
      .def("generate_float_slice_mesh", &VoxelWrapper::generateSliceMesh<float>,
           "grid_name"_a = "Boundary", "x_value"_a = 0, "min_value"_a = 0,
           "max_value"_a = 1, R"(Generates a mesh of
  a slice of a given float grid.)")
      .def("generate_int_slice_mesh", &VoxelWrapper::generateSliceMesh<int>,
           "grid_name"_a = "Boundary", "x_value"_a = 0, "min_value"_a = 0,
           "max_value"_a = 1,
           R"(Generates a mesh of a slice of a given int grid.)");

}  // initGeoBindings

}  // namespace geo
}  // namespace esp
