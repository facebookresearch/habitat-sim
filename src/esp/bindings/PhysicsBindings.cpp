#include "esp/bindings/bindings.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/RigidObject.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace physics {

void initPhysicsBindings(py::module& m) {
  // ==== enum object MotionType ====
  py::enum_<MotionType>(m, "MotionType")
      .value("ERROR_MOTIONTYPE", MotionType::ERROR_MOTIONTYPE)
      .value("STATIC", MotionType::STATIC)
      .value("KINEMATIC", MotionType::KINEMATIC)
      .value("DYNAMIC", MotionType::DYNAMIC);

  // ==== struct RigidState ===
  py::class_<RigidState, RigidState::ptr>(m, "RigidState")
      .def(py::init(&RigidState::create<>))
      .def(py::init(&RigidState::create<const Magnum::Quaternion&,
                                        const Magnum::Vector3&>))
      .def_readwrite("rotation", &RigidState::rotation)
      .def_readwrite("translation", &RigidState::translation);

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
}

}  // namespace physics
}  // namespace esp
