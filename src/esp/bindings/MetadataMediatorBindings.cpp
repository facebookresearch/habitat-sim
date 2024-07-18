// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/PythonBindings.h>

#include "esp/metadata/MetadataMediator.h"
namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace metadata {

void initMetadataMediatorBindings(py::module& m) {
  py::class_<MetadataMediator, MetadataMediator::ptr>(
      m, "MetadataMediator",
      R"(Aggregates all AttributesManagers and provides an API for swapping the active SceneDataset. It can exist independant of a :ref:`Simulator` object for programmatic metadata management and can be passed into the constructor via the :ref:`SimulatorConfiguration`.)")
      .def(py::init(&MetadataMediator::create<>))
      .def(py::init<const sim::SimulatorConfiguration&>())
      .def_property(
          "active_dataset", &MetadataMediator::getActiveSceneDatasetName,
          &MetadataMediator::setActiveSceneDatasetName,
          R"(The currently active dataset being used.  Will attempt to load
            configuration files specified if does not already exist.)")

      /* --- Methods --- */
      .def(
          "dataset_exists", &MetadataMediator::sceneDatasetExists,
          R"(Returns whether the passed name references an existing scene dataset or not.)",
          "dataset_name"_a)
      .def(
          "remove_dataset", &MetadataMediator::removeSceneDataset,
          R"(Remove the given dataset from MetadataMediator.  If specified dataset is currently active, this will fail.)",
          "dataset_name"_a)
      /* --- Template Manager accessors --- */
      .def_property_readonly(
          "ao_template_manager", &MetadataMediator::getAOAttributesManager,
          pybind11::return_value_policy::reference,
          R"(The current dataset's AOAttributesManager instance
            for configuring articulated object templates.)")
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
            for configuring simulation stage templates.)")
      .def_property_readonly(
          "urdf_paths", &MetadataMediator::getArticulatedObjectModelFilenames,
          pybind11::return_value_policy::reference,
          R"(Access to the dictionary of URDF paths, keyed by shortened name, value being full path.)")
      .def(
          "get_scene_handles", &MetadataMediator::getAllSceneInstanceHandles,
          R"(Returns a list the names of all the available scene instances in the currently active dataset.)")
      .def(
          "get_scene_user_defined",
          &MetadataMediator::getSceneInstanceUserConfiguration,
          R"(Returns the user_defined attributes for the scene instance specified by scene_name)",
          "scene_name"_a)
      .def_property_readonly(
          "summary", &MetadataMediator::getDatasetsOverview,
          R"(This provides a summary of the datasets currently loaded.)")
      .def(
          "dataset_report", &MetadataMediator::createDatasetReport,
          R"(This provides an indepth report of the loaded templates for the specified dataset.
          If no dataset_name is specified, returns a report on the currently active dataset)",
          "dataset_name"_a = "");

}  // initMetadataMediatorBindings

}  // namespace metadata
}  // namespace esp
