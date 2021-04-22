// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/core/AbstractManagedObject.h"
#include "esp/physics/PhysicsObjectBase.h"
#include "esp/physics/RigidBase.h"
#include "esp/physics/RigidObject.h"
#include "esp/physics/RigidStage.h"
#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"
#include "esp/physics/objectWrappers/ManagedRigidBase.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"

namespace py = pybind11;
using py::literals::operator""_a;

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
      .def_property("handle", &PhysObjWrapper::getHandle,
                    &PhysObjWrapper::setHandle,
                    ("Name of this " + objType).c_str())
      .def_property(
          "motion_type", &PhysObjWrapper::getMotionType,
          &PhysObjWrapper::setMotionType,
          ("Get or set the MotionType of this " + objType + ".").c_str())
      .def_property_readonly(
          "ID", &PhysObjWrapper::getID,
          ("System-generated ID for this construct.  Will be unique among " +
           objType + "s.")
              .c_str())
      .def_property_readonly("template_class", &PhysObjWrapper::getClassKey,
                             ("Class name of this " + objType).c_str());
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
      .def("user_attributes", &RigidBaseWrapper::userAttributes,
           ("User-defined " + objType + " attributes.").c_str())
      .def_property("transformation", &RigidBaseWrapper::getTransformation,
                    &RigidBaseWrapper::setTransformation,
                    ("Get or set the transformation matrix of this " + objType +
                     "'s root SceneNode. If set, sim state will be updated.")
                        .c_str())
      .def_property("translation", &RigidBaseWrapper::getTranslation,
                    &RigidBaseWrapper::setTranslation,
                    ("Get or set the translation vector of this " + objType +
                     "'s root SceneNode. If set, sim state will be updated.")
                        .c_str())
      .def_property("rotation", &RigidBaseWrapper::getRotation,
                    &RigidBaseWrapper::getRotation,
                    ("Get or set the rotation quaternion of this " + objType +
                     "'s root SceneNode. If set, sim state will be updated.")
                        .c_str())
      .def_property("rigid_state", &RigidBaseWrapper::getRigidState,
                    &RigidBaseWrapper::setRigidState,
                    ("Get or set this " + objType +
                     "'s transformation as a Rigid State (i.e. vector, "
                     "quaternion). If set, sim state will be updated.")
                        .c_str())
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
              .c_str())
      .def_property_readonly("root_scene_node", &RigidBaseWrapper::getSceneNode,
                             ("Get a reference to the root SceneNode of this " +
                              objType + "'s  SceneGraph subtree.")
                                 .c_str());

}  // declareRigidBaseWrapper

void initPhysicsObjectBindings(py::module& m) {
  // create bindings for RigidObjects
  declareBasePhysicsObjectWrapper<esp::physics::RigidObject>(m, "Rigid Object",
                                                             "BaseRigidObject");

  declareRigidBaseWrapper<RigidObject>(m, "Rigid Object", "BaseRigidObject");

  // rigid object instance

  py::class_<ManagedRigidObject, AbstractManagedRigidBase<RigidObject>,
             std::shared_ptr<ManagedRigidObject>>(m, "ManagedRigidObject")
      .def_property_readonly(
          "creation_attributes",
          &ManagedRigidObject::getInitializationAttributes,
          R"(Get a copy of the attributes used to create this rigid object)");

  // create bindings for RigidStage
  // declareBasePhysicsObjectWrapper<RigidStage>(m, "Rigid Stage",
  // "BaseRigidStage");

  // declareRigidBaseWrapper<RigidStage>(m, "Rigid Stage", "BaseRigidStage");

  // rigid stage instance

}  // initPhysicsObjectBindings

}  // namespace physics
}  // namespace esp
