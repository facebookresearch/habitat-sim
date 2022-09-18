// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_PYTHONBATCHENVIRONMENTSTATE_H_
#define ESP_BATCHEDSIM_PYTHONBATCHENVIRONMENTSTATE_H_

#include "esp/batched_sim/BatchedSimAssert.h"

#include "esp/core/esp.h"

#ifndef DISABLE_BATCHED_SIM_PYBIND
#include <pybind11/numpy.h>
#endif

namespace esp {
namespace batched_sim {

// namespace py = pybind11;

struct PythonBatchEnvironmentState {
#ifndef DISABLE_BATCHED_SIM_PYBIND
  // curr episode
  pybind11::array_t<int> episode_idx;  //  = -1;       // 0..len(episodes)-1
  pybind11::array_t<int>
      episode_step_idx;  //  = -1;  // will be zero if this env was just reset
  pybind11::array_t<int>
      target_obj_idx;  //  = -1;    // see obj_positions, obj_rotations
  // all positions/rotations are relative to the mesh, i.e. some arbitrary
  // coordinate frame
  pybind11::array_t<float> target_obj_start_pos;  // Magnum::Vector3
  // Magnum::Quaternion target_obj_start_rotation;
  pybind11::array_t<float> target_obj_start_rotation;  // Magnum::Matrix3
  pybind11::array_t<float> robot_start_pos;            // Magnum::Vector3
  pybind11::array_t<float> robot_start_rotation;       // Magnum::Matrix3
  pybind11::array_t<float> goal_pos;                   // Magnum::Vector3
  // Magnum::Quaternion goal_rotation;

  // robot state
  pybind11::array_t<float> robot_pos;              // Magnum::Vector3
  pybind11::array_t<float> robot_inv_rotation;     // Magnum::Matrix3
  pybind11::array_t<float> robot_joint_positions;  // std::vector<float>
  pybind11::array_t<float>
      robot_joint_positions_normalized;      // std::vector<float>
  pybind11::array_t<float> ee_pos;           // Magnum::Vector3
  pybind11::array_t<float> ee_inv_rotation;  // Magnum::Matrix3

  pybind11::array_t<bool> did_collide;        //  = false;
  pybind11::array_t<int> held_obj_idx;        // = -1;
  pybind11::array_t<bool> did_attempt_grasp;  // = false;
  pybind11::array_t<bool> did_grasp;          // = false;
  pybind11::array_t<bool> did_drop;           // = false;
  pybind11::array_t<float> drop_height;       // = NAN;

  // other env state
  // std::vector<Magnum::Vector3> obj_positions;
  // std::vector<Magnum::Quaternion> obj_rotations;
  pybind11::array_t<float> target_obj_pos;  // Magnum::Matrix3
#endif
};

class PythonBatchEnvironmentStateWrapper {
 public:
  PythonBatchEnvironmentStateWrapper(int numBatches,
                                     int numEnvs,
                                     int numJoints);

  PythonBatchEnvironmentStateWrapper() = default;
  // disallow copy
  PythonBatchEnvironmentStateWrapper(
      const PythonBatchEnvironmentStateWrapper& rhs) = delete;
  PythonBatchEnvironmentStateWrapper(PythonBatchEnvironmentStateWrapper&& rhs) =
      default;
  PythonBatchEnvironmentStateWrapper& operator=(
      PythonBatchEnvironmentStateWrapper&&) = default;

  PythonBatchEnvironmentState& getState(int batchIdx);
  const PythonBatchEnvironmentState& getState(int batchIdx) const;

 private:
#ifndef DISABLE_BATCHED_SIM_PYBIND
  template <typename T>
  pybind11::array_t<T> getArray(std::vector<std::vector<T>>& vectorPool,
                                T initialVal,
                                int dim0,
                                int dim1 = -1,
                                int dim2 = -1) {
    vectorPool.emplace_back(std::vector<T>());
    auto& vec = vectorPool.back();
    if (dim1 == -1) {
      // simple 1D
      vec.resize(dim0, initialVal);
      return pybind11::array(vec.size(), vec.data());
    } else if (dim2 == -1) {
      // 2D
      vec.resize(dim0 * dim1, initialVal);
      return pybind11::array_t<T>(
          {dim0, dim1},                   // shape
          {dim1 * sizeof(T), sizeof(T)},  // C-style contiguous strides
          vec.data());
    } else {
      // 3D
      vec.resize(dim0 * dim1 * dim2, initialVal);
      return pybind11::array_t<T>({dim0, dim1, dim2},  // shape
                                  {dim1 * dim2 * sizeof(T), dim2 * sizeof(T),
                                   sizeof(T)},  // C-style contiguous strides
                                  vec.data());
    }
  }
#endif

  std::vector<PythonBatchEnvironmentState> states;

  std::vector<std::vector<float>> floatVectors;
  std::vector<std::vector<int>> intVectors;
  std::vector<std::vector<bool>> boolVectors;
};

#ifndef DISABLE_BATCHED_SIM_PYBIND
void safePyArraySet(pybind11::array_t<float>& arr,
                    int idx0,
                    const Magnum::Vector3& item);
void safePyArraySet(pybind11::array_t<float>& arr,
                    int idx0,
                    const Magnum::Matrix3x3& item);
#endif

}  // namespace batched_sim
}  // namespace esp

#endif
