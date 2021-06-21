#include "esp/bindings/bindings.h"
#include "esp/physics/PhysicsManager.h"

#include "python/corrade/EnumOperators.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace physics {

void initPhysicsBindings(py::module& m) {
  // ==== enum object PhysicsSimulationLibrary ====
  py::enum_<PhysicsManager::PhysicsSimulationLibrary>(
      m, "PhysicsSimulationLibrary")
      .value("NoPhysics", PhysicsManager::PhysicsSimulationLibrary::NoPhysics)
      .value("Bullet", PhysicsManager::PhysicsSimulationLibrary::Bullet);

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

  // ==== enum articulated JointType ====
  py::enum_<JointType>(m, "JointType")
      .value("Revolute", JointType::Revolute)
      .value("Prismatic", JointType::Prismatic)
      .value("Spherical", JointType::Spherical)
      .value("Planar", JointType::Planar)
      .value("Fixed", JointType::Fixed)
      .value("Invalid", JointType::Invalid);

  // ==== enum object JointMotorType ====
  py::enum_<JointMotorType>(m, "JointMotorType")
      .value("SingleDof", JointMotorType::SingleDof)
      .value("Spherical", JointMotorType::Spherical);

  // ==== struct object JointMotorSettings ====
  py::class_<JointMotorSettings, JointMotorSettings::ptr>(m,
                                                          "JointMotorSettings")
      .def(py::init(&JointMotorSettings::create<>))
      .def(py::init(&JointMotorSettings::create<double, double, double, double,
                                                double>),
           "position_target"_a, "position_gain"_a, "velocity_target"_a,
           "velocity_gain"_a, "max_impulse"_a)
      .def(py::init(
               &JointMotorSettings::create<const Mn::Quaternion&, double,
                                           const Mn::Vector3&, double, double>),
           "spherical_position_target"_a, "position_gain"_a,
           "spherical_velocity_target"_a, "velocity_gain"_a, "max_impulse"_a)
      .def_readwrite("position_target", &JointMotorSettings::positionTarget,
                     R"(Single DoF joint position target.)")
      .def_readwrite("spherical_position_target",
                     &JointMotorSettings::sphericalPositionTarget,
                     R"(Spherical joint position target (Mn::Quaternion).)")
      .def_readwrite("position_gain", &JointMotorSettings::positionGain,
                     R"(Position (proportional) gain Kp.)")
      .def_readwrite(
          "velocity_target", &JointMotorSettings::velocityTarget,
          R"(Single DoF joint velocity target. Zero acts like joint damping/friction.)")
      .def_readwrite("spherical_velocity_target",
                     &JointMotorSettings::sphericalVelocityTarget,
                     R"(Spherical joint velocity target.)")
      .def_readwrite("velocity_gain", &JointMotorSettings::velocityGain,
                     R"(Velocity (derivative) gain Kd.)")
      .def_readwrite(
          "max_impulse", &JointMotorSettings::maxImpulse,
          R"(The maximum impulse applied by this motor. Should be tuned relative to physics timestep.)")
      .def_readwrite(
          "motor_type", &JointMotorSettings::motorType,
          R"(The type of motor parameterized by these settings. Determines which parameters to use.)");

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

  // ==== struct object ContactPointData ====
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

  // ==== enum object CollisionGroup ====
  py::enum_<CollisionGroup> collisionGroups{m, "CollisionGroups",
                                            "CollisionGroups"};
  collisionGroups.value("Default", CollisionGroup::Default)
      .value("Static", CollisionGroup::Static)
      .value("Kinematic", CollisionGroup::Kinematic)
      .value("Dynamic", CollisionGroup::Dynamic)
      .value("Robot", CollisionGroup::Robot)
      .value("Noncollidable", CollisionGroup::Noncollidable)
      .value("UserGroup0", CollisionGroup::UserGroup0)
      .value("UserGroup1", CollisionGroup::UserGroup1)
      .value("UserGroup2", CollisionGroup::UserGroup2)
      .value("UserGroup3", CollisionGroup::UserGroup3)
      .value("UserGroup4", CollisionGroup::UserGroup4)
      .value("UserGroup5", CollisionGroup::UserGroup5)
      .value("UserGroup6", CollisionGroup::UserGroup6)
      .value("UserGroup7", CollisionGroup::UserGroup7)
      .value("UserGroup8", CollisionGroup::UserGroup8)
      .value("UserGroup9", CollisionGroup::UserGroup9)
      .value("None", CollisionGroup{});
  corrade::enumOperators(collisionGroups);

  // ==== class object CollisionGroupHelper ====
  py::class_<CollisionGroupHelper, std::shared_ptr<CollisionGroupHelper>>(
      m, "CollisionGroupHelper")
      .def_static("get_group", &CollisionGroupHelper::getGroup, "name"_a,
                  R"(Get a group by assigned name.)")
      .def_static("get_group_name", &CollisionGroupHelper::getGroupName,
                  "group"_a, R"(Get the name assigned to a CollisionGroup.)")
      .def_static("set_group_name", &CollisionGroupHelper::setGroupName,
                  "group"_a, "name"_a, R"(Assign a name to a CollisionGroup.)")
      .def_static(
          "get_mask_for_group",
          [](CollisionGroup group) {
            return CollisionGroup(
                uint32_t(CollisionGroupHelper::getMaskForGroup(group)));
          },
          "group"_a,
          R"(Get the mask for a collision group describing its interaction with other groups.)")
      .def_static(
          "get_mask_for_group",
          [](const std::string& group_name) {
            return CollisionGroup(
                uint32_t(CollisionGroupHelper::getMaskForGroup(group_name)));
          },
          "group"_a,
          R"(Get the mask for a collision group describing its interaction with other groups.)")
      .def_static(
          "set_mask_for_group",
          [](CollisionGroup group, CollisionGroup mask) {
            CollisionGroupHelper::setMaskForGroup(group, CollisionGroups(mask));
          },
          "group"_a, "mask"_a,
          R"(Set the mask for a collision group describing its interaction with other groups. It is not recommended to modify the mask for default, non-user groups.)")
      .def_static(
          "set_group_interacts_with",
          &CollisionGroupHelper::setGroupInteractsWith, "group_a"_a,
          "group_b"_a, "interact"_a,
          R"(Set groupA's collision mask to a specific interaction state with respect to groupB.)")
      .def_static("get_all_group_names",
                  &CollisionGroupHelper::getAllGroupNames,
                  R"(Get a list of all configured collision group names.)");
}

}  // namespace physics
}  // namespace esp
