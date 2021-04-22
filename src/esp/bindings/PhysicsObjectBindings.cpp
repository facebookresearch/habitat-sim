// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"
#include "esp/physics/objectWrappers/ManagedRigidBase.h"

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
  py::class_<PhysObjWrapper, esp::core::AbstractManagedObject,
             typename PhysObjWrapper::ptr>(m, pyclass_name.c_str())
      .def_property("handle", &PhysObjWrapper::getHandle,
                    &PhysObjWrapper::setHandle, R"(Name of )" + objType)
      .def_property(
          "motion_type", &PhysObjWrapper::getMotionType,
          &PhysObjWrapper::setMotionType,
          R"(Get or set the MotionType of this his )" + objType + R"(.)")
      .def_property_readonly(
          "ID", &PhysObjWrapper::getID,
          R"(System-generated ID for construct.  Will be unique among )" +
              objType + R"(s.)")
      .def_property_readonly("template_class", &PhysObjWrapper::getClassKey,
                             R"(Class name of )" + objType);
}  // declareBasePhysicsObjectWrapper

template <class T>
void declareRigidBaseWrapper(py::module& m,
                             const std::string& objType,
                             const std::string& classStrPrefix) {
  using RigidBaseWrapper = AbstractManagedRigidBase<T>;
  std::string pyclass_name = classStrPrefix + std::string("RigidBaseWrapper");
  // ==== AbstractManagedRigidBase ====
  py::class_<RigidBaseWrapper, AbstractManagedPhysicsObject<T>,
             typename RigidBaseWrapper::ptr>(m, pyclass_name.c_str())

      /* --- Geometry & Transformations --- */

      .def_property(
          "transformation", &RigidBaseWrapper::getTransformation,
          &RigidBaseWrapper::setTransformation,
          R"(Get or set the transformation matrix of this )" + objType +
              R"('s root SceneNode. If set, sim state will be updated.)")
      .def_property(
          "translation", &RigidBaseWrapper::getTranslation,
          &RigidBaseWrapper::setTranslation,
          R"(Get or set the translation vector of this )" + objType +
              R"('s root SceneNode. If set, sim state will be updated.)")
      .def_property(
          "rotation", &RigidBaseWrapper::getRotation,
          &RigidBaseWrapper::getRotation,
          R"(Get or set the rotation quaternion of this )" + objType +
              R"('s root SceneNode. If set, sim state will be updated.)")
      .def_property(
          "rigid_state", &RigidBaseWrapper::getRigidState,
          &RigidBaseWrapper::setRigidState,
          R"(Get or set this )" + objType +
              R"('s transformation as a Rigid State (i.e. vector, quaternion). If set, sim state will be updated.)")
      .def_property_readonly("scale", &RigidBaseWrapper::getScale,
                             R"(Get the scale of the )" + objType)

      /* --- Physics Properties and Functions --- */
      .def(
          "apply_force", &RigidBaseWrapper::applyForce, "force"_a,
          "relative_position"_a,
          R"(Apply an external force to this )" + objType +
              R"( at a specific point relative to the )" + objType +
              R"('s center of mass in global coordinates. Only applies to MotionType::DYNAMIC objects.)")
      .def(
          "apply_impulse", &RigidBaseWrapper::applyImpulse, "impulse"_a,
          "relative_position"_a,
          R"(Apply an external impulse to this )" + objType +
              R"( at a specific point relative to the )" + objType +
              R"('s center of mass in global coordinates. Only applies to MotionType::DYNAMIC objects.)")
      .def("apply_torque", &RigidBaseWrapper::applyTorque, "torque"_a,
           R"(Apply torque to this )" + objType +
               R"(. Only applies to MotionType::DYNAMIC objects.)")
      .def("apply_impulse_torque", &RigidBaseWrapper::applyImpulseTorque,
           "impulse"_a,
           R"(Apply torque impulse to this )" + objType +
               R"(. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "angular_damping", &RigidBaseWrapper::getAngularDamping,
          &RigidBaseWrapper::setAngularDamping,
          R"(Get or set this )" + objType +
              R"('s scalar angular damping coefficient. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "angular_velocity", &RigidBaseWrapper::getAngularVelocity,
          &RigidBaseWrapper::setAngularVelocity,
          R"(Get or set this )" + objType +
              R"('s scalar angular velocity vector. Only applies to MotionType::DYNAMIC objects.)")

      .def_property("collidable", &RigidBaseWrapper::getCollidable,
                    &RigidBaseWrapper::setCollidable,
                    R"(Get or set whether this )" + objType +
                        R"( has collisions enabled.)")
      .def_property(
          "com", &RigidBaseWrapper::getCOM, &RigidBaseWrapper::setCOM,
          R"(Get or set this )" + objType +
              R"('s center of mass (COM) in global coordinate frame.)")
      .def_property(
          "friction_coefficient", &RigidBaseWrapper::getFrictionCoefficient,
          &RigidBaseWrapper::setFrictionCoefficient,
          R"(Get or set this )" + objType +
              R"('s scalar coefficient of friction. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "intertia_diagonal", &RigidBaseWrapper::getInertiaVector,
          &RigidBaseWrapper::setInertiaVector,
          R"(Get or set the inertia matrix's diagonal for this )" + objType +
              R"(. If an object is aligned with its principle axii of inertia, the 3x3 inertia matrix can be reduced to a diagonal. Only applies to MotionType::DYNAMIC objects.)")
      .def_property_readonly(
          "inertia_matrix", &RigidBaseWrapper::getInertiaMatrix,
          R"(Get the inertia matrix for this )" + objType +
              R"(.  To change the values, use the object's "inertia_diagonal" property.)")
      .def_property(
          "linear_damping", &RigidBaseWrapper::getLinearDamping,
          &RigidBaseWrapper::setLinearDamping,
          R"(Get or set this )" + objType +
              R"('s scalar linear damping coefficient. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "linear_velocity", &RigidBaseWrapper::getLinearVelocity,
          &RigidBaseWrapper::setLinearVelocity,
          R"(Get or set this )" + objType +
              R"('s vector linear velocity. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "mass", &RigidBaseWrapper::getMass, &RigidBaseWrapper::setMass,
          R"(Get or set this )" + objType +
              R"('s mass. Only applies to MotionType::DYNAMIC objects.)")
      .def_property(
          "restitution_coefficient",
          &RigidBaseWrapper::getRestitutionCoefficient,
          &RigidBaseWrapper::setRestitutionCoefficient,
          R"(Get or set this )" + objType +
              R"('s scalar coefficient of restitution. Only applies to MotionType::DYNAMIC objects.)")

      /* --- Miscellaneous --- */
      .def_property("semantic_id", &RigidBaseWrapper::getSemanticId,
                    &RigidBaseWrapper::setSemanticId,
                    R"(Get or set this )" + objType + R"('s semantic ID.)")
      .def_property_readonly(
          "visual_scene_nodes", &RigidBaseWrapper::getVisualSceneNodes,
          R"(Get a list of references to the SceneNodes with this )" + objType +
              R"(' render assets attached. Use this to manipulate this )" +
              objType +
              R"('s visual state. Changes to these nodes will not affect physics simulation.)")
      .def_property_readonly(
          "root_scene_node", &RigidBaseWrapper::getSceneNode,
          R"(Get a reference to the root SceneNode of this )" + objType +
              R"('s  SceneGraph subtree.)");

}  // declareRigidBaseWrapper

void initPhysicsObjectBindings(py::module& m) {}  // initPhysicsObjectBindings

}  // namespace physics
}  // namespace esp
