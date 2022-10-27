// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"
#include "esp/bindings/EnumOperators.h"
#include "esp/physics/PhysicsManager.h"

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
      .def_readwrite("linear_velocity", &VelocityControl::linVel,
                     R"(The linear velocity in meters/second.)")
      .def_readwrite(
          "angular_velocity", &VelocityControl::angVel,
          R"(The angular velocity (Omega) in units of radians per second.)")
      .def_readwrite("controlling_lin_vel", &VelocityControl::controllingLinVel,
                     R"(Whether or not linear velocity is integrated.)")
      .def_readwrite(
          "lin_vel_is_local", &VelocityControl::linVelIsLocal,
          R"(Whether the linear velocity is considered to be in object local space or global space.)")
      .def_readwrite("controlling_ang_vel", &VelocityControl::controllingAngVel,
                     R"(Whether or not angular velocity is integrated.)")
      .def_readwrite(
          "ang_vel_is_local", &VelocityControl::angVelIsLocal,
          R"(Whether the angular velocity is considered to be in object local space or global space.)")
      .def(
          "integrate_transform", &VelocityControl::integrateTransform, "dt"_a,
          "rigid_state"_a,
          R"(Integrate the velocity (with explicit Euler) over a discrete timestep (dt) starting at a given state and using configured parameters. Returns the new state after integration.)");

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

  // ==== enum RigidConstraintType ====
  py::enum_<RigidConstraintType>(m, "RigidConstraintType")
      .value("PointToPoint", RigidConstraintType::PointToPoint)
      .value("Fixed", RigidConstraintType::Fixed);

  // ==== struct object RigidConstraintSettings ====
  py::class_<RigidConstraintSettings, RigidConstraintSettings::ptr>(
      m, "RigidConstraintSettings")
      .def(py::init(&RigidConstraintSettings::create<>))
      .def_readwrite("constraint_type",
                     &RigidConstraintSettings::constraintType,
                     R"(The type of constraint described by these settings.)")
      .def_readwrite(
          "max_impulse", &RigidConstraintSettings::maxImpulse,
          R"(The maximum impulse applied by this constraint. Should be tuned relative to physics timestep.)")
      .def_readwrite(
          "object_id_a", &RigidConstraintSettings::objectIdA,
          R"(The id of the first object. Must be >=0. For mixed type constraints, objectA must be the ArticulatedObject.)")
      .def_readwrite("object_id_b", &RigidConstraintSettings::objectIdB,
                     R"(The id of the second object. -1 for world/global.)")
      .def_readwrite(
          "link_id_a", &RigidConstraintSettings::linkIdA,
          R"(The id of the link for objectA if articulated, otherwise ignored. -1 for base link.)")
      .def_readwrite(
          "link_id_b", &RigidConstraintSettings::linkIdB,
          R"(The id of the link for objectB if articulated, otherwise ignored. -1 for base link.)")
      .def_readwrite("pivot_a", &RigidConstraintSettings::pivotA,
                     R"(Constraint point in local space of objectA.)")
      .def_readwrite("pivot_b", &RigidConstraintSettings::pivotB,
                     R"(Constraint point in local space of objectB.)")
      .def_readwrite(
          "frame_a", &RigidConstraintSettings::frameA,
          R"(Constraint orientation frame in local space of objectA as 3x3 rotation matrix for RigidConstraintType::Fixed.)")
      .def_readwrite(
          "frame_b", &RigidConstraintSettings::frameB,
          R"(Constraint orientation frame in local space of objectB as 3x3 rotation matrix for RigidConstraintType::Fixed.)");

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
      .def_readwrite(
          "object_id_a", &ContactPointData::objectIdA,
          R"(The Habitat object id of the first object in this collision pair.)")
      .def_readwrite(
          "object_id_b", &ContactPointData::objectIdB,
          R"(The Habitat object id of the second object in this collision pair.)")
      .def_readwrite(
          "link_id_a", &ContactPointData::linkIndexA,
          R"(The Habitat link id of the first object in this collision pair if an articulated link. -1 can indicate base link.)")
      .def_readwrite(
          "link_id_b", &ContactPointData::linkIndexB,
          R"(The Habitat link id of the second object in this collision pair if an articulated link. -1 can indicate base link.)")
      .def_readwrite(
          "position_on_a_in_ws", &ContactPointData::positionOnAInWS,
          R"(The global position of the contact point on the first object.)")
      .def_readwrite(
          "position_on_b_in_ws", &ContactPointData::positionOnBInWS,
          R"(The global position of the contact point on the second object.)")
      .def_readwrite(
          "contact_normal_on_b_in_ws", &ContactPointData::contactNormalOnBInWS,
          R"(The contact normal relative to the second object in world space.)")
      .def_readwrite("contact_distance", &ContactPointData::contactDistance,
                     R"(The penetration depth of the contact point.)")
      .def_readwrite("normal_force", &ContactPointData::normalForce,
                     R"(The normal force produced by the contact point.)")
      .def_readwrite("linear_friction_force1",
                     &ContactPointData::linearFrictionForce1)
      .def_readwrite("linear_friction_force2",
                     &ContactPointData::linearFrictionForce2)
      .def_readwrite("linear_friction_direction1",
                     &ContactPointData::linearFrictionDirection1)
      .def_readwrite("linear_friction_direction2",
                     &ContactPointData::linearFrictionDirection2)
      .def_readwrite(
          "is_active", &ContactPointData::isActive,
          R"(Whether or not the contact is between active objects. Deactivated objects may produce contact points but no reaction.)");

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
  pybindEnumOperators(collisionGroups);

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
