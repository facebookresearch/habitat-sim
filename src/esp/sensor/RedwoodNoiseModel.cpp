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

      // downsample
      const float d = depth(y - y % 2, x - x % 2);

      // Distortion
      const float undistorted_d =
          undistort(x / xmax * 639.0 + 0.5, y / ymax * 479.0 + 0.5, d, model_);

      // quantization and high freq noise
      if (undistorted_d == 0.0)
        noisyDepth(j, i) = 0.0;
      else
        noisyDepth(j, i) = 35.130 * 8.0 /
                           (std::round(35.130 / undistorted_d +
                                       prng_.normal_float_01() * 0.027778) *
                            8.0);
    }
  }

  return noisyDepth;
}

#ifdef SENSORS_WITH_CUDA
namespace impl {

CurandStates* getCurandStates();
void freeCurandStates(CurandStates* curandStates);
void modelToDev(const float* __restrict__ model, float** devModel);
void releaseDevModel(float* devModel);
void simulateFromCPU(const float* __restrict__ depth,
                     const int H,
                     const int W,
                     const float* __restrict__ devModel,
                     CurandStates* curandStates,
                     float* __restrict__ noisyDepth);
}  // namespace impl

RedwoodNoiseModelGPUImpl::RedwoodNoiseModelGPUImpl(
    const Eigen::Ref<const RowMatrixXf> model) {
  impl::modelToDev(model.data(), &devModel_);
  curandStates_ = impl::getCurandStates();
}

RedwoodNoiseModelGPUImpl::~RedwoodNoiseModelGPUImpl() {
  impl::releaseDevModel(devModel_);
  impl::freeCurandStates(curandStates_);
}

RowMatrixXf RedwoodNoiseModelGPUImpl::simulateFromCPU(
    const Eigen::Ref<const RowMatrixXf> depth) {
  RowMatrixXf noisyDepth(depth.rows(), depth.cols());

  impl::simulateFromCPU(depth.data(), depth.rows(), depth.cols(), devModel_,
                        curandStates_, noisyDepth.data());
  return noisyDepth;
}
#endif

}  // namespace sensor
}  // namespace esp
