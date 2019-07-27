#ifdef SENSORS_WITH_CUDA
#include <cuda_runtime.h>
#endif

#include "RedwoodNoiseModel.h"

namespace esp {
namespace sensor {

namespace {
constexpr int MODEL_N_DIMS = 5;
float undistort(const int _x,
                const int _y,
                const float z,
                const RowMatrixXf& model) {
  const int i2 = int((z + 1) / 2);
  const int i1 = i2 - 1;
  const float a = (z - (i1 * 2 + 1)) / 2.0;
  const int x = _x / 8;
  const int y = _y / 6;
  const float f =
      (1 - a) * model(y, x * MODEL_N_DIMS + std::min(std::max(i1, 0), 4)) +
      a * model(y, x * MODEL_N_DIMS + std::min(i2, 4));

  if (f <= 1e-5)
    return 0;
  else
    return z / f;
}
}  // namespace

RowMatrixXf RedwoodNoiseModelCPUImpl::simulate(
    const Eigen::Ref<const RowMatrixXf> depth) {
  RowMatrixXf noisyDepth(depth.rows(), depth.cols());

  const double ymax = depth.rows() - 1, xmax = depth.cols() - 1;

  for (int j = 0; j <= ymax; ++j) {
    for (int i = 0; i <= xmax; ++i) {
      // Shuffle pixels
      const int y =
          std::min(std::max(j + prng_.normal_float_01() * 0.25, 0.0), ymax) +
          0.5;
      const int x =
          std::min(std::max(i + prng_.normal_float_01() * 0.25, 0.0), xmax) +
          0.5;

      // downsample and clip max depth
      const float d = std::min(depth(y - y % 2, x - x % 2), 5.0f);

      // Distortion
      const float undistorted_d =
          undistort(x / xmax * 639.0 + 0.5, y / ymax * 479.0 + 0.5, d, model_);

      // quantization and high freq noise
      if (undistorted_d == 0.0)
        noisyDepth(j, i) = 0.0;
      else {
        const float denom = (round(35.130f / undistorted_d +
                                   prng_.normal_float_01() * 0.027778f) *
                             8.0f);
        noisyDepth(j, i) = denom > 1e-5 ? (35.130f * 8.0f / denom) : 0.0;
      }
    }
  }

  return noisyDepth;
}

#ifdef SENSORS_WITH_CUDA
namespace impl {
CurandStates* getCurandStates();
void freeCurandStates(CurandStates* curandStates);
void simulateFromCPU(const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ noisyDepth);
}  // namespace impl

namespace {
struct CudaDeviceContext {
  CudaDeviceContext(int deviceId) {
    cudaGetDevice(&currentDevice);
    if (deviceId != currentDevice) {
      cudaSetDevice(deviceId);
      setDevice = true;
    }
  }

  ~CudaDeviceContext() {
    if (setDevice)
      cudaSetDevice(currentDevice);
  }

 private:
  bool setDevice = false;
  int currentDevice = -1;
};
}  // namespace

RedwoodNoiseModelGPUImpl::RedwoodNoiseModelGPUImpl(
    const Eigen::Ref<const RowMatrixXf> model,
    int gpuDeviceId)
    : gpuDeviceId_{gpuDeviceId} {
  CudaDeviceContext ctx(gpuDeviceId_);

  cudaMalloc(&devModel_, model.rows() * model.cols() * sizeof(float));
  cudaMemcpy(devModel_, model.data(),
             model.rows() * model.cols() * sizeof(float),
             cudaMemcpyHostToDevice);
  curandStates_ = impl::getCurandStates();
}

RedwoodNoiseModelGPUImpl::~RedwoodNoiseModelGPUImpl() {
  CudaDeviceContext ctx(gpuDeviceId_);

  if (devModel_ != nullptr)
    cudaFree(devModel_);
  impl::freeCurandStates(curandStates_);
}

RowMatrixXf RedwoodNoiseModelGPUImpl::simulateFromCPU(
    const Eigen::Ref<const RowMatrixXf> depth) {
  CudaDeviceContext ctx(gpuDeviceId_);

  RowMatrixXf noisyDepth(depth.rows(), depth.cols());

  impl::simulateFromCPU(depth.data(), depth.rows(), depth.cols(), devModel_,
                        curandStates_, noisyDepth.data());
  return noisyDepth;
}
#endif

}  // namespace sensor
}  // namespace esp
