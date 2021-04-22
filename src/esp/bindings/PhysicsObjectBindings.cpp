// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace physics {

template <class T>
void declareBasePhysicsObjectWrapper(py::module& m,
                                     const std::string& objType,
                                     const std::string& classStrPrefix) {
  using PhysObjWrapper = AbstractManagedPhysicsObject<T>;
  std::string pyclass_name = classStrPrefix + std::string("ObjectWrapper");
  // ==== AbstractManagedPhysicsObject ====
  py::class_<PhysObjWrapper, esp::core::AbstractManagedObject,
             typename PhysObjWrapper::ptr>(m, pyclass_name.c_str())
      .def(py::init(&PhysObjWrapper::template create<std::shared_ptr<T>&,
                                                     const std::string&>))
      .def_property("handle", &PhysObjWrapper::getHandle,
                    &PhysObjWrapper::setHandle, R"(Name of physics object.)")
      .def_property_readonly(
          "ID", &PhysObjWrapper::getID,
          R"(System-generated ID for object.  Will be unique among )" +
              objType + R"(s.)")
      .def_property_readonly("template_class", &PhysObjWrapper::getClassKey,
                             R"(Class name of physics object.)");
}  // declareBasePhysicsObjectWrapper

template <class T>
void declareRigidBaseWrapper(py::module& m, const std::string& classStrPrefix) {
}  // declareRigidBaseWrapper

void initPhysicsObjectBindings(py::module& m) {}  // initPhysicsObjectBindings

}  // namespace physics
}  // namespace esp
