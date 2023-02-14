// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BINDINGS_ENUMOPERATORS_H_
#define ESP_BINDINGS_ENUMOPERATORS_H_

#include <pybind11/pybind11.h>

namespace esp {

/* Pybind's py::arithmetic() doesn't work on strongly-typed enums with custom
   operators */
template <class T>
void pybindEnumOperators(pybind11::enum_<T>& e) {
  e.def("__or__",
        [](const T& a, const T& b) {
          return T(typename std::underlying_type<T>::type(a | b));
        })
      .def("__and__",
           [](const T& a, const T& b) {
             return T(typename std::underlying_type<T>::type(a & b));
           })
      .def("__xor__",
           [](const T& a, const T& b) {
             return T(typename std::underlying_type<T>::type(a ^ b));
           })
      .def("__invert__",
           [](const T& a) {
             return T(typename std::underlying_type<T>::type(~a));
           })
      .def("__bool__", [](const T& a) {
        return bool(typename std::underlying_type<T>::type(a));
      });
}

}  // namespace esp

#endif
