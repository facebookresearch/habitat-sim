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

py::capsule getDepthMemory(BatchedSimulator& bsim, const uint32_t groupIdx)
{
    return py::capsule(bsim.getBpsRenderer().getDepthPointer(groupIdx));
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
      .def_readwrite("include_depth", &BatchedSimulatorConfig::includeDepth, R"(Todo)")
      .def_readwrite("include_color", &BatchedSimulatorConfig::includeColor, R"(Todo)")
      .def_readwrite("sensor0", &BatchedSimulatorConfig::sensor0, R"(Todo)")
      .def_readwrite("force_random_actions", &BatchedSimulatorConfig::forceRandomActions, R"(Todo)")
      .def_readwrite("do_async_physics_step", &BatchedSimulatorConfig::doAsyncPhysicsStep, R"(Todo)")
      .def_readwrite("num_physics_substeps", &BatchedSimulatorConfig::numSubsteps, R"(Todo)")
      .def_readwrite("do_procedural_episode_set", &BatchedSimulatorConfig::doProceduralEpisodeSet, R"(Todo)")
      .def_readwrite("episode_set_filepath", &BatchedSimulatorConfig::episodeSetFilepath, R"(Todo)");
    
  py::class_<PythonEnvironmentState, PythonEnvironmentState::ptr>(
      m, "EnvironmentState")
      .def(py::init(&PythonEnvironmentState::create<>))
      .def_readwrite("target_obj_idx", &PythonEnvironmentState::target_obj_idx, R"(Todo)")
      .def_readwrite("goal_pos", &PythonEnvironmentState::goal_pos, R"(Todo)")
      .def_readwrite("episode_idx", &PythonEnvironmentState::episode_idx, R"(Todo)")
      .def_readwrite("episode_step_idx", &PythonEnvironmentState::episode_step_idx, R"(Todo)")
      .def_readwrite("robot_position", &PythonEnvironmentState::robot_position, R"(Todo)")
      .def_readwrite("robot_yaw", &PythonEnvironmentState::robot_yaw, R"(Todo)")
      .def_readwrite("ee_pos", &PythonEnvironmentState::ee_pos, R"(Todo)")
      .def_readwrite("did_collide", &PythonEnvironmentState::did_collide, R"(Todo)")
      .def_readwrite("obj_positions", &PythonEnvironmentState::obj_positions, R"(Todo)");


  py::class_<BatchedSimulator, BatchedSimulator::ptr>(m, "BatchedSimulator")
      .def(py::init(&BatchedSimulator::create<const BatchedSimulatorConfig&>))
      .def("reset",
           &BatchedSimulator::reset, R"(todo)")
      .def("get_environment_states",
           &BatchedSimulator::getEnvironmentStates, R"(todo)")
      .def("start_step_physics_or_reset",
           &BatchedSimulator::startStepPhysicsOrReset, R"(todo)")
      .def("wait_step_physics_or_reset", &BatchedSimulator::waitStepPhysicsOrReset, R"(todo)")
      .def("start_render", &BatchedSimulator::startRender, R"(todo)")
      .def("wait_render", &BatchedSimulator::waitRender, R"(todo)")
      .def("close", &BatchedSimulator::close, R"(todo)")
      // .def("reset", &BatchedSimulator::reset, R"(todo)")
      .def(
          "rgba",
          [](BatchedSimulator& self, const uint32_t groupIdx) {
            return getColorMemory(self, groupIdx);
          },
          R"(todo)")
      .def(
          "depth",
          [](BatchedSimulator& self, const uint32_t groupIdx) {
            return getDepthMemory(self, groupIdx);
          },
          R"(todo)")
      .def("get_recent_stats_and_reset", &BatchedSimulator::getRecentStatsAndReset, R"(todo)");
}

}  // namespace batched_sim
}  // namespace esp
