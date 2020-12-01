// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/geo/OBB.h"
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

}  // initGeoBindings

}  // namespace geo
}  // namespace esp
