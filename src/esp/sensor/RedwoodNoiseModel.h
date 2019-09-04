#pragma once

#include "esp/core/esp.h"
#include "esp/core/random.h"

#include "RedwoodNoiseModel.cuh"

namespace esp {
namespace sensor {

struct RedwoodNoiseModelGPUImpl {
  RedwoodNoiseModelGPUImpl(const Eigen::Ref<const Eigen::RowMatrixXf> model,
                           int gpuDeviceId,
                           float noiseMultiplier);

  Eigen::RowMatrixXf simulateFromCPU(
      const Eigen::Ref<const Eigen::RowMatrixXf> depth);
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
