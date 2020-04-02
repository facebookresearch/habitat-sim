// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/geo/CoordinateFrame.h"
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

  // ==== CoordinateFrame ===
  py::class_<CoordinateFrame, CoordinateFrame::ptr>(m, "CoordinateFrame")
      .def(py::init(&CoordinateFrame::create<>))
      .def(py::init(
          &CoordinateFrame::create<const vec3f&, const vec3f&, const vec3f&>))
      .def(py::init(&CoordinateFrame::create<const quatf&, const vec3f&>))
      .def(py::init(
          &CoordinateFrame::create<const Magnum::Quaternion&, const vec3f&>))
      .def("origin", &CoordinateFrame::origin)
      .def("up", &CoordinateFrame::up)
      .def("gravity", &CoordinateFrame::gravity)
      .def("front", &CoordinateFrame::front)
      .def("back", &CoordinateFrame::back)
      .def("rotation_world_to_frame", &CoordinateFrame::rotationWorldToFrame)
      .def("rotation_frame_to_world", &CoordinateFrame::rotationFrameToWorld)
      //.def("transformation_world_to_frame",
      //&CoordinateFrame::transformationWorldToFrame)
      .def("to_json", &CoordinateFrame::toJson)
      .def("from_json", &CoordinateFrame::fromJson, "json"_a);
}

}  // namespace geo
}  // namespace esp
