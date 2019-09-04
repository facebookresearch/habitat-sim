#include "RedwoodNoiseModel.cuh"

#include <algorithm>

#include <cuda_runtime.h>
#include <curand_kernel.h>

namespace {
const int MODEL_N_DIMS = 5;
const int MODEL_N_COLS = 80;

__device__ float undistort(const int _x,
                           const int _y,
                           const float z,
                           const float* __restrict__ model) {
  const int i2 = (z + 1) / 2;
  const int i1 = i2 - 1;
  const float a = (z - (i1 * 2 + 1)) / 2.0f;
  const int x = _x / 8;
  const int y = _y / 6;

  const float f =
      (1 - a) *
          model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(max(i1, 0), 4)] +
      a * model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(i2, 4)];

  if (f <= 1e-5f)
    return 0;
  else
    return z / f;
}

__global__ void redwoodNoiseModelKernel(const float* __restrict__ depth,
                                        const int H,
                                        const int W,
                                        curandState_t* states,
                                        const float* __restrict__ model,
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
          min(max(j + curand_normal(&curandState) * 0.25f, 0.0f), ymax) + 0.5f;
      const int x =
          min(max(i + curand_normal(&curandState) * 0.25f, 0.0f), xmax) + 0.5f;

      // downsample
      const float d = depth[(y - y % 2) * W + x - x % 2];
      // If depth is greater than 10m, the sensor will just return a zero
      if (d >= 10.0f) {
        noisyDepth[j * W + i] = 0.0f;
      } else {
        // Distortion
        const float undistorted_d =
            undistort(x / xmax * 639.0f, y / ymax * 479.0f, d, model);

        // quantization and high freq noise
        if (undistorted_d == 0.0f)
          noisyDepth[j * W + i] = 0.0f;
        else {
          const float denom = (round(35.130f / undistorted_d +
                                     curand_normal(&curandState) * 0.027778f) *
                               8.0f);
          noisyDepth[j * W + i] = denom > 1e-5 ? (35.130f * 8.0f / denom) : 0.0;
        }
      }
    }
  }
}

}  // namespace

namespace esp {
namespace sensor {

namespace impl {

__global__ void curandStatesSetupKernel(curandState_t* state, int seed, int n) {
  int id = threadIdx.x + blockIdx.x * 64;
  if (id < n) {
    curand_init(seed + id, 1, 0, &state[id]);
  }
}

struct CurandStates {
  curandState_t* devStates = nullptr;
  int n_blocks_ = 0;
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
    if (devStates != nullptr) {
      cudaFree(devStates);
      devStates = nullptr;
    }
  }

  ~CurandStates() { release(); }
};

CurandStates* getCurandStates() {
  return new CurandStates();
}
void freeCurandStates(CurandStates* curandStates) {
  if (curandStates != nullptr)
    delete curandStates;
}

void simulateFromGPU(const float* __restrict__ devDepth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ devNoisyDepth) {
  const int n_threads = std::min(std::max(W / 4, 1), 64);
  const int n_blocks = std::max(H / 8, 1);

  curandStates->alloc(n_blocks);
  redwoodNoiseModelKernel<<<n_blocks, n_threads>>>(
      devDepth, H, W, curandStates->devStates, devModel, devNoisyDepth);
}

void simulateFromCPU(const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ noisyDepth) {
  float *devDepth, *devNoisyDepth;
  cudaMalloc(&devDepth, H * W * sizeof(float));
  cudaMalloc(&devNoisyDepth, H * W * sizeof(float));

  cudaMemcpy(devDepth, depth, H * W * sizeof(float), cudaMemcpyHostToDevice);

  simulateFromGPU(devDepth, H, W, devModel, curandStates, devNoisyDepth);

  cudaMemcpy(noisyDepth, devNoisyDepth, H * W * sizeof(float),
             cudaMemcpyDeviceToHost);

  cudaFree(devNoisyDepth);
  cudaFree(devDepth);
}
}  // namespace impl
}  // namespace sensor
}  // namespace esp
