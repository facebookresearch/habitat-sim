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
                                     const std::string& classStrPrefix) {
}  // declareBasePhysicsObjectWrapper

template <class T>
void declareRigidBaseWrapper(py::module& m, const std::string& classStrPrefix) {
}  // declareRigidBaseWrapper

void initPhysicsObjectBindings(py::module& m) {
  // // ==== AbstractManagedPhysicsObject ====
  // py::class_<AbstractManagedPhysicsObject, esp::core::AbstractManagedObject,
  //            AbstractAttributes::ptr>(m, "Managed")
  //     .def(py::init(&AbstractManagedPhysicsObject::create<std::shared_ptr<T>&,
  //                                                         const
  //                                                         std::string&>))
  //     .def_property("handle", &AbstractAttributes::getHandle,
  //                   &AbstractAttributes::setHandle,
  //                   R"(Name of attributes template. )")
  //     .def_property_readonly(
  //         "file_directory", &AbstractManagedPhysicsObject::getFileDirectory,
  //         R"(Directory where file-based templates were loaded from.)")
  //     .def_property_readonly(
  //         "ID", &AbstractManagedPhysicsObject::getID,
  //         R"(System-generated ID for template.  Will be unique among
  //         templates of same type.)")
  //     .def_property_readonly("template_class",
  //                            &AbstractManagedPhysicsObject::getClassKey,
  //                            R"(Class name of physics object.)");

}  // initPhysicsObjectBindings

}  // namespace physics
}  // namespace esp
