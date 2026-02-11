// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <cuda_runtime.h>

#include <cstring>

#include "RedwoodNoiseModel.h"

namespace esp {
namespace sensor {

namespace {

struct CudaDeviceContext {
  explicit CudaDeviceContext(const int deviceId) {
    cudaGetDevice(&currentDevice_);
    if (deviceId != currentDevice_) {
      cudaSetDevice(deviceId);
      setDevice_ = true;
    }
  }

  ~CudaDeviceContext() {
    if (setDevice_)
      cudaSetDevice(currentDevice_);
  }

 private:
  bool setDevice_ = false;
  int currentDevice_ = -1;
};

}  // namespace

RedwoodNoiseModelGPUImpl::RedwoodNoiseModelGPUImpl(const float* model,
                                                   const int modelRows,
                                                   const int modelCols,
                                                   const int gpuDeviceId,
                                                   const float noiseMultiplier)
    : gpuDeviceId_{gpuDeviceId},
      maxThreadsPerBlock_{[gpuDeviceId]() -> int {
        int maxThreadsPerBlock;
        cudaDeviceGetAttribute(&maxThreadsPerBlock,
                               cudaDeviceAttr::cudaDevAttrMaxThreadsPerBlock,
                               gpuDeviceId);

        return maxThreadsPerBlock;
      }()},
      warpSize_{[gpuDeviceId]() -> int {
        int warpSize;
        cudaDeviceGetAttribute(&warpSize, cudaDeviceAttr::cudaDevAttrWarpSize,
                               gpuDeviceId);

        return warpSize;
      }()},
      noiseMultiplier_{noiseMultiplier} {
  CudaDeviceContext ctx{gpuDeviceId_};

  const std::size_t modelSize = modelRows * modelCols * sizeof(float);
  cudaMalloc(&devModel_, modelSize);
  cudaMemcpy(devModel_, model, modelSize, cudaMemcpyHostToDevice);
  curandStates_ = impl::getCurandStates();
}

RedwoodNoiseModelGPUImpl::~RedwoodNoiseModelGPUImpl() {
  CudaDeviceContext ctx{gpuDeviceId_};

  if (devModel_ != nullptr)
    cudaFree(devModel_);
  impl::freeCurandStates(curandStates_);
}

Grid2Df RedwoodNoiseModelGPUImpl::simulateFromCPU(const float* depth,
                                                  const int rows,
                                                  const int cols) {
  CudaDeviceContext ctx{gpuDeviceId_};
  Grid2Df noisyDepth(rows, cols);

  impl::simulateFromCPU(maxThreadsPerBlock_, warpSize_, depth, rows, cols,
                        devModel_, curandStates_, noiseMultiplier_,
                        noisyDepth.data());
  return noisyDepth;
}

void RedwoodNoiseModelGPUImpl::simulateFromGPU(const float* devDepth,
                                               const int rows,
                                               const int cols,
                                               float* devNoisyDepth) {
  CudaDeviceContext ctx{gpuDeviceId_};
  impl::simulateFromGPU(maxThreadsPerBlock_, warpSize_, devDepth, rows, cols,
                        devModel_, curandStates_, noiseMultiplier_,
                        devNoisyDepth);
}

}  // namespace sensor
}  // namespace esp
