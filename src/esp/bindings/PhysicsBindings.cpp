#include "esp/bindings/bindings.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/RigidObject.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace physics {

void initPhysicsBindings(py::module& m) {
  // ==== enum object PhysicsSimulationLibrary ====
  py::enum_<PhysicsManager::PhysicsSimulationLibrary>(
      m, "PhysicsSimulationLibrary")
      .value("NONE", PhysicsManager::PhysicsSimulationLibrary::NONE)
      .value("BULLET", PhysicsManager::PhysicsSimulationLibrary::BULLET);

  // ==== enum object MotionType ====
  py::enum_<MotionType>(m, "MotionType")
      .value("ERROR_MOTIONTYPE", MotionType::ERROR_MOTIONTYPE)
      .value("STATIC", MotionType::STATIC)
      .value("KINEMATIC", MotionType::KINEMATIC)
      .value("DYNAMIC", MotionType::DYNAMIC);

  // ==== struct object VelocityControl ====
  py::class_<VelocityControl, VelocityControl::ptr>(m, "VelocityControl")
      .def(py::init(&VelocityControl::create<>))
      .def_readwrite("linear_velocity", &VelocityControl::linVel)
      .def_readwrite("angular_velocity", &VelocityControl::angVel)
      .def_readwrite("controlling_lin_vel", &VelocityControl::controllingLinVel)
      .def_readwrite("lin_vel_is_local", &VelocityControl::linVelIsLocal)
      .def_readwrite("controlling_ang_vel", &VelocityControl::controllingAngVel)
      .def_readwrite("ang_vel_is_local", &VelocityControl::angVelIsLocal)
      .def("integrate_transform", &VelocityControl::integrateTransform, "dt"_a,
           "rigid_state"_a);

  // ==== struct object RayHitInfo ====
  py::class_<RayHitInfo, RayHitInfo::ptr>(m, "RayHitInfo")
      .def(py::init(&RayHitInfo::create<>))
      .def_readonly("object_id", &RayHitInfo::objectId)
      .def_readonly("point", &RayHitInfo::point)
      .def_readonly("normal", &RayHitInfo::normal)
      .def_readonly("ray_distance", &RayHitInfo::rayDistance);

  // ==== struct object RaycastResults ====
  py::class_<RaycastResults, RaycastResults::ptr>(m, "RaycastResults")
      .def(py::init(&RaycastResults::create<>))
      .def_readonly("hits", &RaycastResults::hits)
      .def_readonly("ray", &RaycastResults::ray)
      .def("has_hits", &RaycastResults::hasHits);
}

}  // namespace physics
}  // namespace esp
