// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RedwoodNoiseModel.cuh"

#include <algorithm>

#include <cuda_runtime.h>
#include <curand_kernel.h>

namespace {
const int MODEL_N_DIMS = 4;
const int MODEL_N_COLS = 100;

// Read about the noise model here: http://www.alexteichman.com/octo/clams/
// Original source code: http://redwood-data.org/indoor/data/simdepth.py
__device__ float undistort(const int _x,
                           const int _y,
                           const float z,
                           const float* __restrict__ model) {
  const int i2 = (z + 1) / 2;
  const int i1 = i2 - 1;
  const float a = (z - (i1 * 2.0f + 1.0f)) / 2.0f;
  const int x = _x / 8;
  const int y = _y / 6;

  const float f =
      (1.0f - a) *
          model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(max(i1, 0), 4)] +
      a * model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(i2, 4)];

  if (f < 1e-5)
    return 0.0f;
  else
    return z / f;
}

__global__ void redwoodNoiseModelKernel(const float* __restrict__ depth,
                                        const int H,
                                        const int W,
                                        curandState_t* states,
                                        const float* __restrict__ model,
                                        const float noiseMultiplier,
                                        float* __restrict__ noisyDepth) {
  const int TID = threadIdx.x;
  const int BID = blockIdx.x;

  // curandStates are thread-safe, so all threads in a block share the same
  // state. They are NOT block safe however
  curandState_t curandState = states[BID];

  const float ymax = H - 1;
  const float xmax = W - 1;

  for (int j = BID; j < H; j += gridDim.x) {
    for (int i = TID; i < W; i += blockDim.x) {
      // Shuffle pixels
      const int y =
          min(max(j + curand_normal(&curandState) * 0.25f * noiseMultiplier,
                  0.0f),
              ymax) +
          0.5f;
      const int x =
          min(max(i + curand_normal(&curandState) * 0.25f * noiseMultiplier,
                  0.0f),
              xmax) +
          0.5f;

      // downsample
      const float d = depth[(y - y % 2) * W + x - x % 2];
      // If depth is greater than 10m, the sensor will just return a zero
      if (d >= 10.0f) {
        noisyDepth[j * W + i] = 0.0f;
      } else {
        // Distortion
        // The noise model was originally made for a 640x480 sensor,
        // so re-map our arbitrarily sized sensor to that size!
        const float undistorted_d =
            undistort(static_cast<float>(x) / xmax * 639.0f + 0.5f,
                      static_cast<float>(y) / ymax * 479.0f + 0.5f, d, model);

        // quantization and high freq noise
        if (undistorted_d == 0.0f) {
          noisyDepth[j * W + i] = 0.0f;
        } else {
          const float denom = round(
              (35.130f / static_cast<double>(undistorted_d) +
               curand_normal(&curandState) * 0.027778f * noiseMultiplier) *
              8.0f);
          noisyDepth[j * W + i] =
              denom > 1e-5 ? (35.130f * 8.0f / denom) : 0.0f;
        }
      }
    }
  }
}

__global__ void curandStatesSetupKernel(curandState_t* state, int seed, int n) {
  int id = threadIdx.x + blockIdx.x * 64;
  if (id < n) {
    curand_init(seed, id + 1, 0, &state[id]);
  }
}

}  // namespace

namespace esp {
namespace sensor {
namespace impl {

struct CurandStates {
  CurandStates() : devStates(0), n_blocks_(0) {}
  void alloc(const int n_blocks) {
    if (n_blocks > n_blocks_) {
      release();
      cudaMalloc(&devStates, n_blocks * sizeof(curandState_t));
      curandStatesSetupKernel<<<std::max(n_blocks / 64, 1), 64>>>(
          devStates, rand(), n_blocks);
      n_blocks_ = n_blocks;
    }
  }

  void release() {
    if (devStates != 0) {
      cudaFree(devStates);
      devStates = 0;
      n_blocks_ = 0;
    }
  }

  ~CurandStates() { release(); }

  curandState_t* devStates;

 private:
  int n_blocks_;
};

CurandStates* getCurandStates() {
  return new CurandStates();
}
void freeCurandStates(CurandStates* curandStates) {
  if (curandStates != 0)
    delete curandStates;
}

void simulateFromGPU(const float* __restrict__ devDepth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     const float noiseMultiplier,
                     float* __restrict__ devNoisyDepth) {
  const int n_threads = std::min(std::max(W / 4, 1), 256);
  const int n_blocks = std::max(H / 8, 1);

  curandStates->alloc(n_blocks);
  redwoodNoiseModelKernel<<<n_blocks, n_threads>>>(
      devDepth, H, W, curandStates->devStates, devModel, noiseMultiplier,
      devNoisyDepth);
}

void simulateFromCPU(const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     const float noiseMultiplier,
                     float* __restrict__ noisyDepth) {
  float *devDepth, *devNoisyDepth;
  cudaMalloc(&devDepth, H * W * sizeof(float));
  cudaMalloc(&devNoisyDepth, H * W * sizeof(float));

  cudaMemcpy(devDepth, depth, H * W * sizeof(float), cudaMemcpyHostToDevice);

  simulateFromGPU(devDepth, H, W, devModel, curandStates, noiseMultiplier,
                  devNoisyDepth);

  cudaMemcpy(noisyDepth, devNoisyDepth, H * W * sizeof(float),
             cudaMemcpyDeviceToHost);

  cudaFree(devNoisyDepth);
  cudaFree(devDepth);
}
}  // namespace impl
}  // namespace sensor
}  // namespace esp
