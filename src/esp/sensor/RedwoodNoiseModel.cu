#include <cuda_runtime.h>

namespace {
constexpr int MODEL_N_DIMS = 5;
constexpr int MODEL_N_ROWS = 80;
constexpr int MODEL_N_COLS = 80;
constexpr int MODEL_N_BYTES =
    MODEL_N_ROWS * MODEL_N_COLS * MODEL_N_DIMS * sizeof(float);
}  // namespace

__device__ float __undistort(const int _x,
                             const int _y,
                             const float z,
                             const float* __restrict__ model) {
  const int i2 = int((z + 1) / 2);
  const int i1 = i2 - 1;
  const float a = (z - (i1 * 2 + 1)) / 2.0;
  const int x = _x / 8;
  const int y = _y / 6;

  const float f =
      (1 - a) *
          model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(max(i1, 0), 4)] +
      a * model[(y * MODEL_N_COLS + x) * MODEL_N_DIMS + min(i2, 4)];

  if (f <= 1e-5)
    return 0;
  else
    return z / f;
}

__global__ void redwoodNoiseModelKernel(const float* __restrict__ depth,
                                        const int H,
                                        const int W,
                                        const float* __restrict__ model,
                                        const unsigned long long seed,
                                        float* __restrict__ noisyDepth) {
  const int TID = threadIdx.x;
  const int BID = blockIdx.x;

  if (TID >= W)
    return;
  if (BID >= H)
    return;

  curandState_t curandState;

  curand_init(seed + BID * W + TID, H * W, 0, *curandState);

  const int ymax = H - 1;
  const int xmax = W - 1;

  const int j = BID;
  for (int i = TID, i < W; i += blockDim.x) {
    // Shuffle pixels
    const int y =
        min(max(j + curand_normal(&curandState) * 0.25, 0.0), ymax) + 0.5;
    const int x =
        min(max(i + curand_normal(&curandState) * 0.25, 0.0), xmax) + 0.5;

    // downsample
    const float d = depth[(y - y % 2) * W + x - x % 2];

    // Distortion
    const float undistorted_d =
        undistort(x / xmax * 639.0 + 0.5, y / ymax * 479.0 + 0.5, d, model);

    // quantization and high freq noise
    if (undistorted_d == 0.0)
      noisyDepth[j * W + i] = 0.0;
    else
      noisyDepth[j * W + i] = 35.130 * 8.0 /
                              (round(35.130 / undistorted_d +
                                     curand_normal(&curandState) * 0.027778) *
                               8.0);
  }
}

namespace esp {
namespace sensor {
namespace impl {
const float* simulatedRedwoodDepthKernelLauncher(
    const float* __restrict__ depth,
    const int H,
    const int W,
    const float* __restrict__ model) {
  float *devDepth, devNoisyDepth;
  cudaMalloc(&devDepth, H * W * sizeof(float));
  cudaMemcpy(devDepth, depth, H * W * sizeof(float), cudaMemcpyHostToDevice);

  cudaMalloc(&devNoisyDepth, H * W * sizeof(float));

  float* devModel;
  cudaMalloc(&devModel, MODEL_N_BYTES);
  cudaMemcpy(devModel, model, MODEL_N_BYTES, cudaMemcpyHostToDevice);

  const int n_threads = std::max(std::min(W / 16, 32), 512);
  redwoodNoiseModelKernel<<<H, n_threads>>>(devDepth, H, W, 0, devModel,
                                            devNoisyDepth);

  float* noisyDepth = new float[H * W];
  cudaMemcpy(noisyDepth, devNoisyDepth, H * W * sizeof(float),
             cudaMemcpyDeviceToHost);

  cudaFree(devModel);
  cudaFree(devNoisyDepth);
  cudaFree(devDepth);

  return noisyDepth;
}
}  // namespace impl
