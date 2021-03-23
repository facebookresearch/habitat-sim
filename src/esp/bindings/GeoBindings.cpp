// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <pybind11/functional.h>
#include "esp/bindings/bindings.h"

#include "esp/geo/OBB.h"
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

  // ==== VoxelWrapper ====
  py::class_<VoxelWrapper, VoxelWrapper::ptr>(m, "VoxelWrapper")
      .def("get_voxel_index_from_global_coords",
           &VoxelWrapper::getVoxelIndexFromGlobalCoords, "coords"_a,
           R"(Gets the voxel index located at a coordinate in world space.)")
      .def(
          "get_global_coords_from_voxel_index",
          &VoxelWrapper::getGlobalCoordsFromVoxelIndex, "coords"_a,
          R"(Gets the world positioning of a particular voxel in a voxelization.)")
      .def("get_voxel_grid_dimensions", &VoxelWrapper::getVoxelGridDimensions,
           R"(Returns a numpy array with the dimensions of the voxel grid.)")
      .def("get_voxel_size", &VoxelWrapper::getVoxelSize,
           R"(Returns a numpy array with the size of a single voxel.)")
      // TODO: put voxel grid creation, deletion, and retrieval here
      .def(
          "generate_bool_grid_from_int_grid",
          &VoxelWrapper::generateBoolGridFromIntGrid, "int_grid_name"_a,
          "bool_grid_name"_a, "start_range"_a, "end_range"_a,
          R"(Takes in the name of an existing integer grid and produces a boolean grid where each voxel is set to true if the value of that voxel in the integer grid falls within the specified range. The newly created boolean grid is registered under bool_grid_name.)")
      .def(
          "generate_bool_grid_from_float_grid",
          &VoxelWrapper::generateBoolGridFromFloatGrid, "float_grid_name"_a,
          "bool_grid_name"_a, "start_range"_a, "end_range"_a,
          R"(Takes in the name of an existing float grid and produces a boolean grid where each voxel is set to true if the value of that voxel in the float grid falls within the specified range. The newly created boolean grid is registered under bool_grid_name.)")
      // TODO: put voxel set filling functions here (need to figure out how to
      // pass in function ptr as arg)
      .def(
          "generate_interior_exterior_voxel_grid",
          &VoxelWrapper::generateInteriorExteriorVoxelGrid,
          R"(Generates an int grid where 0 represents boundaries, INT_MIN represents interior voxels, and INT_MAX represents exterior voxels. The int grid is registered under "InteriorExterior")")
      .def(
          "generate_manhattan_distance_sdf",
          &VoxelWrapper::generateManhattanDistanceSDF,
          "grid_name"_a = "MSignedDistanceField",
          R"(Generates a signed distance field using manhattan distance under a specified grid name.)")
      .def(
          "generate_euclidean_distance_sdf",
          &VoxelWrapper::generateEuclideanDistanceSDF,
          "grid_name"_a = "ESignedDistanceField",
          R"(Generates a signed distance field using euclidean distance under a specified grid name. Also creates a supplementary grid called "ClosestBoundaryCell" which holds the boundary voxel closest to each voxel.)")
      .def(
          "generate_distance_flow_field",
          &VoxelWrapper::generateDistanceFlowField, "grid_name"_a = "FlowField",
          R"(Generates a distance flow vector field, where each vector points away from the closest boundary cell.)")
      .def(
          "generate_mesh", &VoxelWrapper::generateMesh,
          "grid_name"_a = "Boundary", "is_vector_field"_a = false,
          R"(Generates a distance flow vector field, where each vector points away from the closest boundary cell.)")
      .def("generate_float_slice_mesh", &VoxelWrapper::generateSliceMesh<float>,
           "grid_name"_a = "Boundary", "x_value"_a = 0, "min_value"_a = 0,
           "max_value"_a = 1,
           R"(Generates a mesh of a slice of a given float grid.)")
      .def("generate_int_slice_mesh", &VoxelWrapper::generateSliceMesh<int>,
           "grid_name"_a = "Boundary", "x_value"_a = 0, "min_value"_a = 0,
           "max_value"_a = 1,
           R"(Generates a mesh of a slice of a given int grid.)");
}  // initGeoBindings

}  // namespace geo
}  // namespace esp
