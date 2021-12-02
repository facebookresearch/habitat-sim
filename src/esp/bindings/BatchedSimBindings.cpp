// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/batched_sim/BatchedSimulator.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace batched_sim {

namespace {

py::capsule getColorMemory(BatchedSimulator& bsim, const uint32_t groupIdx) {
  return py::capsule(bsim.getBpsRenderer().getColorPointer(groupIdx));
}

}  // namespace

void initBatchedSimBindings(py::module& m) {
  py::class_<CameraSensorConfig>(m, "CameraSensorConfig")
      .def_readwrite("width", &CameraSensorConfig::width, R"(Todo)")
      .def_readwrite("height", &CameraSensorConfig::height, R"(Todo)")
      .def_readwrite("hfov", &CameraSensorConfig::hfov, R"(Todo)");

  py::class_<BatchedSimulatorConfig, BatchedSimulatorConfig::ptr>(
      m, "BatchedSimulatorConfig")
      .def(py::init(&BatchedSimulatorConfig::create<>))
      .def_readwrite("num_envs", &BatchedSimulatorConfig::numEnvs, R"(Todo)")
      .def_readwrite("gpu_id", &BatchedSimulatorConfig::gpuId, R"(Todo)")
      .def_readwrite("sensor0", &BatchedSimulatorConfig::sensor0, R"(Todo)");

  py::class_<BatchedSimulator, BatchedSimulator::ptr>(m, "BatchedSimulator")
      .def(py::init(&BatchedSimulator::create<const BatchedSimulatorConfig&>))
      .def("set_actions", &BatchedSimulator::setActions, R"(todo)")
      .def("auto_reset_or_step_physics",
           &BatchedSimulator::autoResetOrStepPhysics, R"(todo)")
      // .def("step_physics", &BatchedSimulator::stepPhysics, R"(todo)")
      .def("get_rewards", &BatchedSimulator::getRewards, R"(todo)")
      .def("get_dones", &BatchedSimulator::getDones, R"(todo)")
      .def("start_render", &BatchedSimulator::startRender, R"(todo)")
      .def("wait_for_frame", &BatchedSimulator::waitForFrame, R"(todo)")
      // .def("reset", &BatchedSimulator::reset, R"(todo)")
      .def(
          "rgba",
          [](BatchedSimulator& self, const uint32_t groupIdx) {
            return getColorMemory(self, groupIdx);
          },
          R"(todo)");
}

}  // namespace batched_sim
}  // namespace esp
