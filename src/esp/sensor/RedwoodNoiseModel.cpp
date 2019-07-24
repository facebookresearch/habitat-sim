#include "RedwoodNoiseModel.h"

#include "esp/core/random.h"

namespace esp {
namespace sensor {
namespace impl {
RowMatrixXf simulatedRedwoodDepthKernelLauncher(
    const Eigen::Ref<const RowMatrixXf> depth,
    const Eigen::Ref<const RowMatrixXf> model);
}

namespace {
constexpr int MODEL_N_DIMS = 5;
float undistort(const int _x,
                const int _y,
                const float z,
                const Eigen::Ref<const RowMatrixXf> model) {
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

RowMatrixXf simulateRedwoodDepthCPU(const Eigen::Ref<const RowMatrixXf> depth,
                                    const Eigen::Ref<const RowMatrixXf> model) {
  core::Random prng;

  RowMatrixXf noisyDepth = RowMatrixXf::Zero(depth.rows(), depth.cols());

  const double ymax = depth.rows() - 1, xmax = depth.cols() - 1;

#pragma omp parallel for
  for (int j = 0; j <= ymax; ++j) {
    for (int i = 0; i <= xmax; ++i) {
      // Shuffle pixels
      const int y =
          std::min(std::max(j + prng.normal_float_01() * 0.25, 0.0), ymax) +
          0.5;
      const int x =
          std::min(std::max(i + prng.normal_float_01() * 0.25, 0.0), xmax) +
          0.5;

      // downsample
      const float d = depth(y - y % 2, x - x % 2);

      // Distortion
      const float undistorted_d =
          undistort(x / xmax * 639.0 + 0.5, y / ymax * 479.0 + 0.5, d, model);

      // quantization and high freq noise
      if (undistorted_d == 0.0)
        noisyDepth(j, i) = 0.0;
      else
        noisyDepth(j, i) = 35.130 * 8.0 /
                           (std::round(35.130 / undistorted_d +
                                       prng.normal_float_01() * 0.027778) *
                            8.0);
    }
  }

  return noisyDepth;
}

RowMatrixXf simulateRedwoodDepthGPU(const Eigen::Ref<const RowMatrixXf> depth,
                                    const Eigen::Ref<const RowMatrixXf> model) {
  const float* noisyDepthData = impl::simulatedRedwoodDepthKernelLauncher(
      depth.data(), depth.rows(), depth.cols(), model.data());

  RowMatrixXf noisyDepth = RowMatrixXf::Zero(depth.rows(), depth.cols());
  std::memcpy(noisyDepth.data(), noisyDepthData,
              depth.rows() * depth.cols() * sizeof(float));
}
}  // namespace sensor
}  // namespace esp
