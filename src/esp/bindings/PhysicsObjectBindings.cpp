// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/physics/PhysicsObjectBase.h"
#include "esp/physics/RigidBase.h"
#include "esp/physics/RigidObject.h"
#include "esp/physics/RigidStage.h"
#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"
#include "esp/physics/objectWrappers/ManagedRigidBase.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace PhysWraps = esp::physics;
using PhysWraps::ManagedRigidObject;

namespace esp {
namespace physics {

template <class T>
void declareBasePhysicsObjectWrapper(py::module& m,
                                     const std::string& objType,
                                     const std::string& classStrPrefix) {
  using PhysObjWrapper = AbstractManagedPhysicsObject<T>;
  std::string pyclass_name =
      classStrPrefix + std::string("PhysicsObjectWrapper");
  // ==== AbstractManagedPhysicsObject ====
  py::class_<PhysObjWrapper, std::shared_ptr<PhysObjWrapper>>(
      m, pyclass_name.c_str())
      .def_property_readonly("handle", &PhysObjWrapper::getHandle,
                             ("Name of this " + objType).c_str())
      .def_property(
          "motion_type", &PhysObjWrapper::getMotionType,
          &PhysObjWrapper::setMotionType,
          ("Get or set the MotionType of this " + objType + ".").c_str())
      .def_property_readonly(
          "object_id", &PhysObjWrapper::getID,
          ("System-generated ID for this " + objType +
           " construct.  Will be unique among " + objType + "s.")
              .c_str())
      .def_property_readonly(
          "is_alive", &PhysObjWrapper::isAlive,
          ("Whether this " + objType + " still exists and is still valid.")
              .c_str())
      .def_property_readonly("template_class", &PhysObjWrapper::getClassKey,
                             ("Class name of this " + objType).c_str())
      .def("user_attributes", &PhysObjWrapper::userAttributes,
           ("User-defined " + objType +
            " attributes.  These are not used internally by Habitat in any "
            "capacity, but are available for a user to consume how they wish.")
               .c_str())
      .def_property(
          "transformation", &PhysObjWrapper::getTransformation,
          &PhysObjWrapper::setTransformation,
          ("Get or set the transformation matrix of this " + objType +
           "'s root SceneNode. If modified, sim state will be updated.")
              .c_str())
      .def_property(
          "translation", &PhysObjWrapper::getTranslation,
          &PhysObjWrapper::setTranslation,
          ("Get or set the translation vector of this " + objType +
           "'s root SceneNode. If modified, sim state will be updated.")
              .c_str())
      .def_property(
          "rotation", &PhysObjWrapper::getRotation,
          &PhysObjWrapper::setRotation,
          ("Get or set the rotation quaternion of this " + objType +
           "'s root SceneNode. If modified, sim state will be updated.")
              .c_str())
      .def_property("rigid_state", &PhysObjWrapper::getRigidState,
                    &PhysObjWrapper::setRigidState,
                    ("Get or set this " + objType +
                     "'s transformation as a Rigid State (i.e. vector, "
                     "quaternion). If modified, sim state will be updated.")
                        .c_str())
      .def_property_readonly("root_scene_node", &PhysObjWrapper::getSceneNode,
                             ("Get a reference to the root SceneNode of this " +
                              objType + "'s  SceneGraph subtree.")
                                 .c_str())

      .def("set_light_setup", &PhysObjWrapper::setLightSetup,
           ("Set this " + objType +
            "'s light setup using passed light_setup_key.")
               .c_str(),
           "light_setup_key"_a)
      .def_property("awake", &PhysObjWrapper::isActive,
                    &PhysObjWrapper::setActive,
                    ("Get or set whether this " + objType +
                     " is actively being simulated, or is sleeping.")
                        .c_str())
      .def(
          "translate", &PhysObjWrapper::translate, "vector"_a,
          ("Move this " + objType + " using passed translation vector").c_str())
      .def(
          "rotate",
          [](PhysObjWrapper& self, Mn::Radd angle, Mn::Vector3& normAxis) {
            self.rotate(Mn::Rad(angle), normAxis);
          },
          "angle_in_rad"_a, "norm_axis"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around passed 3-element normalized "
           "norm_axis.")
              .c_str())
      .def(
          "rotate_local",
          [](PhysObjWrapper& self, Mn::Radd angle, Mn::Vector3& normAxis) {
            self.rotateLocal(Mn::Rad(angle), normAxis);
          },
          "angle_in_rad"_a, "norm_axis"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around passed 3-element normalized "
           "norm_axis in the local frame.")
              .c_str())
      .def(
          "rotate_x",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateX(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the x-axis in global frame.")
              .c_str())
      .def(
          "rotate_x_local",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateXLocal(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the x-axis in local frame.")
              .c_str())
      .def(
          "rotate_y",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateY(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the y-axis in global frame.")
              .c_str())
      .def(
          "rotate_y_local",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateYLocal(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the y-axis in local frame.")
              .c_str())
      .def(
          "rotate_z",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateZ(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the z-axis in global frame.")
              .c_str())
      .def(
          "rotate_z_local",
          [](PhysObjWrapper& self, Mn::Radd angle) {
            self.rotateZLocal(Mn::Rad(angle));
          },
          "angle_in_rad"_a,
          ("Rotate this " + objType +
           " by passed angle_in_rad around the z-axis in local frame.")
              .c_str())

      ;
}  // declareBasePhysicsObjectWrapper

template <class T>
void declareRigidBaseWrapper(py::module& m,
                             const std::string& objType,
                             const std::string& classStrPrefix) {
  using RigidBaseWrapper = AbstractManagedRigidBase<T>;
  std::string pyclass_name = classStrPrefix + std::string("RigidBaseWrapper");
  // ==== AbstractManagedRigidBase ====
  py::class_<RigidBaseWrapper, AbstractManagedPhysicsObject<T>,
             std::shared_ptr<RigidBaseWrapper>>(m, pyclass_name.c_str())

      /* --- Geometry & Transformations --- */

      .def_property_readonly("scale", &RigidBaseWrapper::getScale,
                             ("Get the scale of the " + objType).c_str())

      /* --- Physics Properties and Functions --- */
      .def("apply_force", &RigidBaseWrapper::applyForce, "force"_a,
           "relative_position"_a,
           ("Apply an external force to this " + objType +
            " at a specific point relative to the " + objType +
            "'s center of mass in global coordinates. Only applies to "
            "MotionType::DYNAMIC objects.")
               .c_str())
      .def("apply_impulse", &RigidBaseWrapper::applyImpulse, "impulse"_a,
           "relative_position"_a,
           ("Apply an external impulse to this " + objType +
            " at a specific point relative to the " + objType +
            "'s center of mass in global coordinates. Only applies to "
            "MotionType::DYNAMIC objects.")
               .c_str())
      .def("apply_torque", &RigidBaseWrapper::applyTorque, "torque"_a,
           ("Apply torque to this " + objType +
            ". Only applies to MotionType::DYNAMIC objects.")
               .c_str())
      .def("apply_impulse_torque", &RigidBaseWrapper::applyImpulseTorque,
           "impulse"_a,
           ("Apply torque impulse to this " + objType +
            ". Only applies to MotionType::DYNAMIC objects.")
               .c_str())
      .def_property("angular_damping", &RigidBaseWrapper::getAngularDamping,
                    &RigidBaseWrapper::setAngularDamping,
                    ("Get or set this " + objType +
                     "'s scalar angular damping coefficient. Only applies "
                     "to MotionType::DYNAMIC objects.")
                        .c_str())
      .def_property("angular_velocity", &RigidBaseWrapper::getAngularVelocity,
                    &RigidBaseWrapper::setAngularVelocity,
                    ("Get or set this " + objType +
                     "'s scalar angular velocity vector. Only applies to "
                     "MotionType::DYNAMIC objects.")
                        .c_str())

      .def_property(
          "collidable", &RigidBaseWrapper::getCollidable,
          &RigidBaseWrapper::setCollidable,
          ("Get or set whether this " + objType + " has collisions enabled.")
              .c_str())
      .def_property("com", &RigidBaseWrapper::getCOM, &RigidBaseWrapper::setCOM,
                    ("Get or set this " + objType +
                     "'s center of mass (COM) in global coordinate frame.")
                        .c_str())
      .def_property("friction_coefficient",
                    &RigidBaseWrapper::getFrictionCoefficient,
                    &RigidBaseWrapper::setFrictionCoefficient,
                    ("Get or set this " + objType +
                     "'s scalar coefficient of friction. Only applies to "
                     "MotionType::DYNAMIC objects.")
                        .c_str())
      .def_property(
          "intertia_diagonal", &RigidBaseWrapper::getInertiaVector,
          &RigidBaseWrapper::setInertiaVector,
          ("Get or set the inertia matrix's diagonal for this " + objType +
           ". If an object is aligned with its principle axii of inertia, "
           "the 3x3 inertia matrix can be reduced to a diagonal. Only "
           "applies to MotionType::DYNAMIC objects.")
              .c_str())
      .def_property_readonly("inertia_matrix",
                             &RigidBaseWrapper::getInertiaMatrix,
                             ("Get the inertia matrix for this " + objType +
                              ".  To change the values, use the object's "
                              "'intertia_diagonal' property.")
                                 .c_str())
      .def_property("linear_damping", &RigidBaseWrapper::getLinearDamping,
                    &RigidBaseWrapper::setLinearDamping,
                    ("Get or set this " + objType +
                     "'s scalar linear damping coefficient. Only applies to "
                     "MotionType::DYNAMIC objects.")
                        .c_str())
      .def_property("linear_velocity", &RigidBaseWrapper::getLinearVelocity,
                    &RigidBaseWrapper::setLinearVelocity,
                    ("Get or set this " + objType +
                     "'s vector linear velocity. Only applies to "
                     "MotionType::DYNAMIC objects.")
                        .c_str())
      .def_property("mass", &RigidBaseWrapper::getMass,
                    &RigidBaseWrapper::setMass,
                    ("Get or set this " + objType +
                     "'s mass. Only applies to MotionType::DYNAMIC objects.")
                        .c_str())
      .def_property("restitution_coefficient",
                    &RigidBaseWrapper::getRestitutionCoefficient,
                    &RigidBaseWrapper::setRestitutionCoefficient,
                    ("Get or set this " + objType +
                     "'s scalar coefficient of restitution. Only applies to "
                     "MotionType::DYNAMIC objects.")
                        .c_str())

      /* --- Miscellaneous --- */
      .def_property("semantic_id", &RigidBaseWrapper::getSemanticId,
                    &RigidBaseWrapper::setSemanticId,
                    ("Get or set this " + objType + "'s semantic ID.").c_str())
      .def_property_readonly(
          "visual_scene_nodes", &RigidBaseWrapper::getVisualSceneNodes,
          ("Get a list of references to the SceneNodes with this " + objType +
           "' render assets attached. Use this to manipulate this " + objType +
           "'s visual state. Changes to these nodes will not affect physics "
           "simulation.")
              .c_str());

}  // declareRigidBaseWrapper

void initPhysicsObjectBindings(py::module& m) {
  // create bindings for RigidObjects
  declareBasePhysicsObjectWrapper<RigidObject>(m, "Rigid Object",
                                               "RigidObject");

  declareRigidBaseWrapper<RigidObject>(m, "Rigid Object", "RigidObject");

  // rigid object instance

  py::class_<ManagedRigidObject, AbstractManagedRigidBase<RigidObject>,
             std::shared_ptr<ManagedRigidObject>>(m, "ManagedRigidObject")
      .def_property_readonly(
          "creation_attributes",
          &ManagedRigidObject::getInitializationAttributes,
          R"(Get a copy of the attributes used to create this rigid object)")
      .def_property_readonly(
          "velocity_control", &ManagedRigidObject::getVelocityControl,
          R"(Retrieves a reference to the VelocityControl struct for this object.)");

  // create bindings for RigidStage
  // declareBasePhysicsObjectWrapper<RigidStage>(m, "Rigid Stage",
  // "BaseRigidStage");

  // declareRigidBaseWrapper<RigidStage>(m, "Rigid Stage", "BaseRigidStage");

  // rigid stage instance

}  // initPhysicsObjectBindings

}  // namespace physics
}  // namespace esp
