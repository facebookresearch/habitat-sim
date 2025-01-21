// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include "esp/geo/Geo.h"
#include "esp/geo/OBB.h"

namespace Mn = Magnum;
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
  geo.attr("LEFT") = Mn::Math::cross(ESP_FRONT, ESP_GRAVITY);
  geo.attr("RIGHT") = Mn::Math::cross(ESP_FRONT, ESP_UP);

  // ==== OBB ====
  py::class_<OBB>(m, "OBB", R"(This is an OBB.)")
      .def(py::init([](const Mn::Vector3& center, const Mn::Vector3& dimensions,
                       const Mn::Quaternion& rotation) {
             return OBB(center, dimensions, rotation);
           }),
           "center"_a, "dimensions"_a, "rotation"_a)
      .def(py::init<Mn::Range3D&>())
      .def(
          "contains", &OBB::contains,
          R"(Returns whether world coordinate point p is contained in this OBB within threshold distance epsilon.)")
      .def("closest_point", &OBB::closestPoint,
           R"(Return closest point to p within OBB.  If p is inside return p.)")
      .def(
          "distance", &OBB::distance,
          R"(Returns distance to p from closest point on OBB surface (0 if point p is inside box))")
      .def("to_aabb", &OBB::toAABB,
           R"(Returns an axis aligned bounding box bounding this OBB.)")
      .def(
          "rotate",
          [](OBB& self, const Mn::Quaternion& rotation) {
            return self.rotate(rotation);
          },
          R"(Rotate this OBB by the given rotation and return reference to self.)")
      .def_property_readonly("center", &OBB::center, R"(Centroid of this OBB.)")
      .def_property_readonly("sizes", &OBB::sizes,
                             R"(The dimensions of this OBB in its own frame.)")
      .def_property_readonly("volume", &OBB::volume,
                             R"(The volume of this bbox.)")
      .def_property_readonly("half_extents", &OBB::halfExtents,
                             R"(Half-extents of this OBB (dimensions).)")
      .def_property_readonly(
          // Return as an array of format [x,y,z,w] to retain existing
          // expectations
          "rotation",
          [](const OBB& self) {
            const Mn::Quaternion q = self.rotation();
            return Mn::Vector4(q.vector(), q.scalar());
          },
          R"(Quaternion representing rotation of this OBB.)")
      .def_property_readonly(
          "local_to_world", [](const OBB& self) { return self.localToWorld(); },
          R"(Transform from local [0,1]^3 coordinates to world coordinates.)")
      .def_property_readonly(
          "world_to_local", [](const OBB& self) { return self.worldToLocal(); },
          R"(Transform from world coordinates to local [0,1]^3 coordinates.)");

  geo.def(
      "compute_gravity_aligned_MOBB", &geo::computeGravityAlignedMOBB,
      R"(Compute a minimum area OBB containing given points, and constrained to have -Z axis along given gravity orientation.)");
  geo.def(
      "get_transformed_bb", &geo::getTransformedBB, "range"_a, "xform"_a,
      R"(Compute the axis-aligned bounding box which results from applying a transform to an existing bounding box.)");

  // ==== Ray ====
  py::class_<Ray>(m, "Ray")
      .def(py::init<Magnum::Vector3, Magnum::Vector3>(), "origin"_a,
           "direction"_a)
      .def(py::init<>())
      .def_readwrite("origin", &Ray::origin)
      .def_readwrite("direction", &Ray::direction);

  // ==== Trajectory utilities ====
  geo.def(
      "build_catmull_rom_spline", &geo::buildCatmullRomTrajOfPoints,
      R"(This function builds an interpolating Catmull-Rom spline through the passed list of
      key points, with num_interpolations interpolated points between each key point, and
      alpha [0,1] determining the nature of the spline (default is .5) :
           0.0 is a standard Catmull-Rom spline (which may have cusps)
           0.5 is a centripetal Catmull-Rom spline
           1.0 is a chordal Catmull-Rom spline)",
      "key_points"_a, "num_interpolations"_a, "alpha"_a = .5f);

}  // initGeoBindings

}  // namespace geo
}  // namespace esp
