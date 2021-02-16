
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

#include "esp/metadata/MetadataMediator.h"
namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace metadata {

void initMetadataMediatorBindings(py::module& m) {
  py::class_<MetadataMediator, MetadataMediator::ptr>(m, "MetadataMediator")
      .def(py::init(&MetadataMediator::create<>))
      .def(py::init<const sim::SimulatorConfiguration&>())
      .def_property(
          "active_dataset", &MetadataMediator::getActiveSceneDatasetName,
          &MetadataMediator::setActiveSceneDatasetName,
          R"(The currently active dataset being used.  Will attempt to load
            configuration files specified if does not already exist.)")

      /* --- Template Manager accessors --- */
      .def_property_readonly(
          "asset_template_manager",
          &MetadataMediator::getAssetAttributesManager,
          pybind11::return_value_policy::reference,
          R"(The current dataset's AssetAttributesManager instance
            for configuring primitive asset templates.)")
      .def_property_readonly(
          "lighting_template_manager",
          &MetadataMediator::getLightLayoutAttributesManager,
          pybind11::return_value_policy::reference,
          R"(The current dataset's LightLayoutAttributesManager instance
            for configuring light templates and layouts.)")
      .def_property_readonly(
          "object_template_manager",
          &MetadataMediator::getObjectAttributesManager,
          pybind11::return_value_policy::reference,
          R"(The current dataset's ObjectAttributesManager instance
            for configuring object templates.)")
      .def_property_readonly("physics_template_manager",
                             &MetadataMediator::getPhysicsAttributesManager,
                             pybind11::return_value_policy::reference,
                             R"(The current PhysicsAttributesManager instance
            for configuring PhysicsManager templates.)")
      .def_property_readonly(
          "stage_template_manager",
          &MetadataMediator::getStageAttributesManager,
          pybind11::return_value_policy::reference,
          R"(The current dataset's StageAttributesManager instance
            for configuring simulation stage templates.)");

}  // initMetadataMediatorBindings

}  // namespace metadata
}  // namespace esp
