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

py::capsule getDebugColorMemory(BatchedSimulator& bsim, const uint32_t groupIdx) {
  return py::capsule(bsim.getDebugBpsRenderer().getColorPointer(groupIdx));
}

py::capsule getDepthMemory(BatchedSimulator& bsim, const uint32_t groupIdx)
{
    return py::capsule(bsim.getBpsRenderer().getDepthPointer(groupIdx));
}

}  // namespace

void initBatchedSimBindings(py::module& m) {
  py::class_<CameraSensorConfig>(m, "CameraSensorConfig")
      .def_readwrite("width", &CameraSensorConfig::width, R"(Todo)")
      .def_readwrite("height", &CameraSensorConfig::height, R"(Todo)");

  py::class_<EpisodeGeneratorConfig, EpisodeGeneratorConfig::ptr>(
      m, "EpisodeGeneratorConfig")  
      .def(py::init(&EpisodeGeneratorConfig::create<>))
      .def_readwrite("numEpisodes", &EpisodeGeneratorConfig::numEpisodes, R"(Todo)")
      .def_readwrite("seed", &EpisodeGeneratorConfig::seed, R"(Todo)")
      .def_readwrite("num_stage_variations", &EpisodeGeneratorConfig::numStageVariations, R"(Todo)")
      .def_readwrite("num_object_variations", &EpisodeGeneratorConfig::numObjectVariations, R"(Todo)")
      .def_readwrite("min_nontarget_objects", &EpisodeGeneratorConfig::minNontargetObjects, R"(Todo)")
      .def_readwrite("max_nontarget_objects", &EpisodeGeneratorConfig::maxNontargetObjects, R"(Todo)")
      .def_readwrite("used_fixed_robot_start_pos", &EpisodeGeneratorConfig::useFixedRobotStartPos, R"(Todo)")
      .def_readwrite("use_fixed_robot_start_yaw", &EpisodeGeneratorConfig::useFixedRobotStartYaw, R"(Todo)")
      .def_readwrite("use_fixed_robot_joint_start_positions", &EpisodeGeneratorConfig::useFixedRobotJointStartPositions, R"(Todo)");

  py::class_<BatchedSimulatorConfig, BatchedSimulatorConfig::ptr>(
      m, "BatchedSimulatorConfig")
      .def(py::init(&BatchedSimulatorConfig::create<>))
      .def_readwrite("num_envs", &BatchedSimulatorConfig::numEnvs, R"(Todo)")
      .def_readwrite("gpu_id", &BatchedSimulatorConfig::gpuId, R"(Todo)")
      .def_readwrite("include_depth", &BatchedSimulatorConfig::includeDepth, R"(Todo)")
      .def_readwrite("include_color", &BatchedSimulatorConfig::includeColor, R"(Todo)")
      .def_readwrite("sensor0", &BatchedSimulatorConfig::sensor0, R"(Todo)")
      .def_readwrite("num_debug_envs", &BatchedSimulatorConfig::numDebugEnvs, R"(Todo)")
      .def_readwrite("debug_sensor", &BatchedSimulatorConfig::debugSensor, R"(Todo)")
      .def_readwrite("force_random_actions", &BatchedSimulatorConfig::forceRandomActions, R"(Todo)")
      .def_readwrite("do_async_physics_step", &BatchedSimulatorConfig::doAsyncPhysicsStep, R"(Todo)")
      .def_readwrite("num_physics_substeps", &BatchedSimulatorConfig::numSubsteps, R"(Todo)")
      .def_readwrite("do_procedural_episode_set", &BatchedSimulatorConfig::doProceduralEpisodeSet, R"(Todo)")
      .def_readwrite("episode_generator_config", &BatchedSimulatorConfig::episodeGeneratorConfig, R"(Todo)")
      .def_readwrite("episode_set_filepath", &BatchedSimulatorConfig::episodeSetFilepath, R"(Todo)");
    
  py::class_<PythonEnvironmentState, PythonEnvironmentState::ptr>(
      m, "EnvironmentState")
      .def(py::init(&PythonEnvironmentState::create<>))
      .def_readwrite("episode_idx", &PythonEnvironmentState::episode_idx, R"(Todo)")
      .def_readwrite("episode_step_idx", &PythonEnvironmentState::episode_step_idx, R"(Todo)")
      .def_readwrite("target_obj_idx", &PythonEnvironmentState::target_obj_idx, R"(Todo)")
      .def_readwrite("target_obj_start_pos", &PythonEnvironmentState::target_obj_start_pos, R"(Todo)")
      .def_readwrite("robot_start_pos", &PythonEnvironmentState::robot_start_pos, R"(Todo)")
      .def_readwrite("robot_start_rotation", &PythonEnvironmentState::robot_start_rotation, R"(Todo)")
      .def_readwrite("goal_pos", &PythonEnvironmentState::goal_pos, R"(Todo)")
      .def_readwrite("goal_rotation", &PythonEnvironmentState::goal_rotation, R"(Todo)")
      .def_readwrite("robot_pos", &PythonEnvironmentState::robot_pos, R"(Todo)")
      .def_readwrite("robot_rotation", &PythonEnvironmentState::robot_rotation, R"(Todo)")
      .def_readwrite("robot_joint_positions", &PythonEnvironmentState::robot_joint_positions, R"(Todo)")
      .def_readwrite("robot_joint_positions_normalized", &PythonEnvironmentState::robot_joint_positions_normalized, R"(Todo)")
      .def_readwrite("ee_pos", &PythonEnvironmentState::ee_pos, R"(Todo)")
      .def_readwrite("ee_rotation", &PythonEnvironmentState::ee_rotation, R"(Todo)")
      .def_readwrite("did_collide", &PythonEnvironmentState::did_collide, R"(Todo)")
      .def_readwrite("held_obj_idx", &PythonEnvironmentState::held_obj_idx, R"(Todo)")
      .def_readwrite("did_grasp", &PythonEnvironmentState::did_grasp, R"(Todo)")
      .def_readwrite("did_drop", &PythonEnvironmentState::did_drop, R"(Todo)")
      .def_readwrite("drop_height", &PythonEnvironmentState::drop_height, R"(Todo)")
      .def_readwrite("obj_positions", &PythonEnvironmentState::obj_positions, R"(Todo)")
      .def_readwrite("obj_rotations", &PythonEnvironmentState::obj_rotations, R"(Todo)");

  py::class_<BatchedSimulator, BatchedSimulator::ptr>(m, "BatchedSimulator")
      .def(py::init(&BatchedSimulator::create<const BatchedSimulatorConfig&>))
      .def("get_num_episodes",
           &BatchedSimulator::getNumEpisodes, R"(todo)")
      .def("get_num_actions",
           &BatchedSimulator::getNumActions, R"(todo)")
      .def("set_camera",
           &BatchedSimulator::setCamera, R"(todo)")
      .def("enable_debug_sensor",
           &BatchedSimulator::enableDebugSensor, R"(todo)")
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
      .def(
          "debug_rgba",
          [](BatchedSimulator& self, const uint32_t groupIdx) {
            return getDebugColorMemory(self, groupIdx);
          },
          R"(todo)")
      .def("get_recent_stats_and_reset", &BatchedSimulator::getRecentStatsAndReset, R"(todo)");
}

}  // namespace batched_sim
}  // namespace esp
