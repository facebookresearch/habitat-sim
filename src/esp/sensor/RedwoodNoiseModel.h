#pragma once

#include "esp/core/esp.h"

namespace esp {
namespace sensor {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    RowMatrixXf;

RowMatrixXf simulateRedwoodDepthCPU(const Eigen::Ref<const RowMatrixXf> depth,
                                    const Eigen::Ref<const RowMatrixXf> model);

RowMatrixXf simulateRedwoodDepthGPU(const Eigen::Ref<const RowMatrixXf> depth,
                                    const Eigen::Ref<const RowMatrixXf> model);
}  // namespace sensor
}  // namespace esp
