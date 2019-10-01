// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
//
#pragma once

#include "esp/core/esp.h"
#include "esp/core/random.h"

#include "RedwoodNoiseModel.cuh"

namespace esp {
namespace sensor {

/**
 * Provides a CUDA/GPU implementation of the Redwood Noise Model for PrimSense
 Depth sensors
 * provided here: http://redwood-data.org/indoor/dataset.html
 *
 * Please cite the following work if you use this noise model
 * @verbatim
@inproceedings{choi2015robust,
  title={Robust reconstruction of indoor scenes},
  author={Choi, Sungjoon and Zhou, Qian-Yi and Koltun, Vladlen},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and
    Pattern Recognition}, pages={5556--5565}, year={2015}
}
  @endverbatim
 */
struct RedwoodNoiseModelGPUImpl {
  /**
   * @brief Constructor
   * @param model             The distortion model from
   *                          http://redwood-data.org/indoor/data/dist-model.txt
   *                          The 3rd dimensions is assumed to have been
   *                          flattened into the second
   * @param gpuDeviceId       The CUDA device ID to use
   * @param noiseMultiplier   Multiplier for the Guassian random-variables. This
   *                          can be used to increase or decrease the noise
   *                          level
   */
  RedwoodNoiseModelGPUImpl(const Eigen::Ref<const Eigen::RowMatrixXf> model,
                           int gpuDeviceId,
                           float noiseMultiplier);

  /**
   * @brief Simulates noisy depth from clean depth.  The input is assumed to be
   * on the CPU and the output will be on the CPU
   *
   * @param[in] depth  Clean depth, i.e. depth from habitat's depth shader
   * @return Simulated noisy depth
   */
  Eigen::RowMatrixXf simulateFromCPU(
      const Eigen::Ref<const Eigen::RowMatrixXf> depth);

  /**
   * @brief Similar to @ref simulateFromCPU() but the input and output are
   * assumed to be on the GPU
   *
   * @param[in] devDepth        Device pointer to the clean depth
   *                            Assumed to be a continguous array in row-major
   *                            order
   * @param[in] rows            The number of rows in the depth image
   * @param[in] cols            The number of columns
   * @param[out] devNoisyDepth  Device pointer to the memory to write the noisy
   *                            depth
   */
  void simulateFromGPU(const float* devDepth,
                       const int rows,
                       const int cols,
                       float* devNoisyDepth);

  ~RedwoodNoiseModelGPUImpl();

 private:
  int gpuDeviceId_;
  float noiseMultiplier_;
  float* devModel_ = nullptr;
  impl::CurandStates* curandStates_ = nullptr;

  ESP_SMART_POINTERS(RedwoodNoiseModelGPUImpl)
};

}  // namespace sensor
}  // namespace esp
