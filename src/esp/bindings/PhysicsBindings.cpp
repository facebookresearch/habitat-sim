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
      .value("UNDEFINED", MotionType::UNDEFINED)
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

  // ==== struct object JointMotorSettings ====
  py::class_<JointMotorSettings, JointMotorSettings::ptr>(m,
                                                          "JointMotorSettings")
      .def(py::init(&JointMotorSettings::create<>))
      .def(py::init(
          &JointMotorSettings::create<double, double, double, double, double>))
      .def_readwrite("position_target", &JointMotorSettings::positionTarget)
      .def_readwrite("position_gain", &JointMotorSettings::positionGain)
      .def_readwrite("velocity_target", &JointMotorSettings::velocityTarget)
      .def_readwrite("velocity_gain", &JointMotorSettings::velocityGain)
      .def_readwrite("max_impulse", &JointMotorSettings::maxImpulse);

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

  py::class_<ContactPointData, ContactPointData::ptr>(m, "ContactPointData")
      .def(py::init(&ContactPointData::create<>))
      .def_readwrite("object_id_a", &ContactPointData::objectIdA)
      .def_readwrite("object_id_b", &ContactPointData::objectIdB)
      .def_readwrite("link_id_a", &ContactPointData::linkIndexA)
      .def_readwrite("link_id_b", &ContactPointData::linkIndexB)
      .def_readwrite("position_on_a_in_ws", &ContactPointData::positionOnAInWS)
      .def_readwrite("position_on_b_in_ws", &ContactPointData::positionOnBInWS)
      .def_readwrite("contact_normal_on_b_in_ws",
                     &ContactPointData::contactNormalOnBInWS)
      .def_readwrite("contact_distance", &ContactPointData::contactDistance)
      .def_readwrite("normal_force", &ContactPointData::normalForce)
      .def_readwrite("linear_friction_force1",
                     &ContactPointData::linearFrictionForce1)
      .def_readwrite("linear_friction_force2",
                     &ContactPointData::linearFrictionForce2)
      .def_readwrite("linear_friction_direction1",
                     &ContactPointData::linearFrictionDirection1)
      .def_readwrite("linear_friction_direction2",
                     &ContactPointData::linearFrictionDirection2)
      .def_readwrite("is_active", &ContactPointData::isActive);
}

}  // namespace physics
}  // namespace esp
