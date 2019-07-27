#pragma once

#include "esp/core/esp.h"
#include "esp/core/random.h"

namespace esp {
namespace sensor {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    RowMatrixXf;

struct RedwoodNoiseModelCPUImpl {
  RedwoodNoiseModelCPUImpl(const Eigen::Ref<const RowMatrixXf> model)
      : model_{model} {};

  RowMatrixXf simulate(const Eigen::Ref<const RowMatrixXf> depth);

 private:
  RowMatrixXf model_;
  core::Random prng_;

  ESP_SMART_POINTERS(RedwoodNoiseModelCPUImpl)
};

#ifdef SENSORS_WITH_CUDA

namespace impl {
struct CurandStates;
}

struct RedwoodNoiseModelGPUImpl {
  RedwoodNoiseModelGPUImpl(const Eigen::Ref<const RowMatrixXf> model,
                           int gpuDeviceId);

  RowMatrixXf simulateFromCPU(const Eigen::Ref<const RowMatrixXf> depth);

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
