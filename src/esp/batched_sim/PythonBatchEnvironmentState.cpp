// Copyright (c) Meta Platforms, Inc. All Rights Reserved
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PythonBatchEnvironmentState.h"
#include "BatchedSimAssert.h"

#include <Magnum/Math/Matrix3.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

PythonBatchEnvironmentStateWrapper::PythonBatchEnvironmentStateWrapper(
    int numBatches,
    int numEnvs,
    int numJointPosVars) {
  states.resize(numBatches);

  for (int batchIdx = 0; batchIdx < numBatches; batchIdx++) {
    auto& state = states[batchIdx];

#ifndef DISABLE_BATCHED_SIM_PYBIND
    //  py::array_t<int> episode_idx = -1;       // 0..len(episodes)-1
    state.episode_idx = getArray(intVectors, -1, numEnvs);
    //   py::array_t<int> episode_step_idx = -1;  // will be zero if this env
    //   was just reset
    state.episode_step_idx = getArray(intVectors, -1, numEnvs);
    // py::array_t<int> target_obj_idx = -1;    // see
    state.target_obj_idx = getArray(intVectors, -1, numEnvs);

    //   obj_positions, obj_rotations
    //   // all positions/rotations are relative to the mesh, i.e. some
    //   arbitrary
    //   // coordinate frame
    //   py::array_t<float> target_obj_start_pos; // Magnum::Vector3
    state.target_obj_start_pos = getArray(floatVectors, 0.f, numEnvs, 3);
    //   //Magnum::Quaternion target_obj_start_rotation;
    //   py::array_t<float> target_obj_start_rotation; // Magnum::Matrix3
    //   py::array_t<float> robot_start_pos; // Magnum::Vector3
    state.robot_start_pos = getArray(floatVectors, 0.f, numEnvs, 3);
    //   py::array_t<float> robot_start_rotation; // Magnum::Matrix3
    //   py::array_t<float> goal_pos; // Magnum::Vector3
    state.goal_pos = getArray(floatVectors, 0.f, numEnvs, 3);
    //   // Magnum::Quaternion goal_rotation;

    //   // robot state
    //   py::array_t<float> robot_pos; // Magnum::Vector3
    state.robot_pos = getArray(floatVectors, 0.f, numEnvs, 3);
    //   py::array_t<float> robot_rotation; // Magnum::Matrix3
    state.robot_inv_rotation = getArray(floatVectors, 0.f, numEnvs, 3, 3);
    //   py::array_t<float> robot_joint_positions; // std::vector<float>
    state.robot_joint_positions =
        getArray(floatVectors, 0.f, numEnvs, numJointPosVars);
    //   py::array_t<float> robot_joint_positions_normalized; //
    //   std::vector<float> py::array_t<float> ee_pos; // Magnum::Vector3
    state.ee_pos = getArray(floatVectors, 0.f, numEnvs, 3);
    //   py::array_t<float> ee_rotation; // Magnum::Matrix3
    state.ee_inv_rotation = getArray(floatVectors, 0.f, numEnvs, 3, 3);

    //   py::array_t<bool> did_collide = false;
    //   py::array_t<int> held_obj_idx = -1;
    state.held_obj_idx = getArray(intVectors, -1, numEnvs);
    //   py::array_t<bool> did_attempt_grasp = false;
    //   py::array_t<bool> did_grasp = false;
    //   py::array_t<bool> did_drop = false;
    state.did_drop = getArray<char>(boolVectors, false, numEnvs);
    //   py::array_t<float> drop_height = NAN;

    //   // other env state
    //   //std::vector<Magnum::Vector3> obj_positions;
    //   //std::vector<Magnum::Quaternion> obj_rotations;
    //   py::array_t<float> target_obj_pos; // Magnum::Matrix3
    state.target_obj_pos = getArray(floatVectors, 0.f, numEnvs, 3);
#endif
  }
}

PythonBatchEnvironmentState& PythonBatchEnvironmentStateWrapper::getState(
    int batchIdx) {
  return safeVectorGet(states, batchIdx);
}

const PythonBatchEnvironmentState& PythonBatchEnvironmentStateWrapper::getState(
    int batchIdx) const {
  return safeVectorGet(states, batchIdx);
}

#ifndef DISABLE_BATCHED_SIM_PYBIND
void safePyArraySet(pybind11::array_t<float>& arr,
                    int idx0,
                    const Magnum::Vector3& item) {
  BATCHED_SIM_ASSERT(arr.ndim() == 2);
  BATCHED_SIM_ASSERT(idx0 >= 0 && idx0 < arr.shape(0));
  BATCHED_SIM_ASSERT(arr.shape(1) == 3);
  float* data = arr.mutable_data(idx0);
  for (int i = 0; i < 3; i++) {
    data[i] = item.data()[i];
  }
}

void safePyArraySet(pybind11::array_t<float>& arr,
                    int idx0,
                    const Magnum::Matrix3x3& item) {
  BATCHED_SIM_ASSERT(arr.ndim() == 3);
  BATCHED_SIM_ASSERT(idx0 >= 0 && idx0 < arr.shape(0));
  BATCHED_SIM_ASSERT(arr.shape(1) == 3);
  BATCHED_SIM_ASSERT(arr.shape(2) == 3);
  float* data = arr.mutable_data(idx0);
  for (int i = 0; i < 9; i++) {
    data[i] = item.data()[i];
  }
}

void safePyArraySet(pybind11::array_t<int>& arr, int idx0, int item) {
  BATCHED_SIM_ASSERT(arr.ndim() == 1);
  BATCHED_SIM_ASSERT(idx0 >= 0 && idx0 < arr.shape(0));
  int* data = arr.mutable_data(idx0);
  *data = item;
}

void safePyArraySet(pybind11::array_t<bool>& arr, int idx0, bool item) {
  BATCHED_SIM_ASSERT(arr.ndim() == 1);
  BATCHED_SIM_ASSERT(idx0 >= 0 && idx0 < arr.shape(0));
  bool* data = arr.mutable_data(idx0);
  *data = item;
}

void safePyArraySet(pybind11::array_t<float>& arr,
                    int idx0,
                    int idx1,
                    float item) {
  BATCHED_SIM_ASSERT(arr.ndim() == 2);
  BATCHED_SIM_ASSERT(idx0 >= 0 && idx0 < arr.shape(0));
  BATCHED_SIM_ASSERT(idx1 >= 0 && idx1 < arr.shape(1));
  float* data = arr.mutable_data(idx0);
  data[idx1] = item;
}

#endif

}  // namespace batched_sim
}  // namespace esp
