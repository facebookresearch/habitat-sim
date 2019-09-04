#pragma once

#include "esp/core/esp.h"
#include "esp/core/random.h"

#include "RedwoodNoiseModel.cuh"

namespace esp {
namespace sensor {

#ifdef ESP_BUILD_WITH_CUDA

struct RedwoodNoiseModelGPUImpl {
  RedwoodNoiseModelGPUImpl(const Eigen::Ref<const RowMatrixXf> model,
                           int gpuDeviceId);

  RowMatrixXf simulateFromCPU(const Eigen::Ref<const RowMatrixXf> depth);
  void simulateFromGPU(const float* devDepth,
                       const int rows,
                       const int cols,
                       float* devNoisyDepth);

  ~RedwoodNoiseModelGPUImpl();

 private:
  int gpuDeviceId_;
  float* devModel_ = nullptr;
  impl::CurandStates* curandStates_ = nullptr;

  ESP_SMART_POINTERS(RedwoodNoiseModelGPUImpl)
};
#endif

}  // namespace sensor
}  // namespace esp
