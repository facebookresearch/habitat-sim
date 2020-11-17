// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PinholeCamera.h"

namespace esp {
namespace sensor {

void PinholeCamera::setProjectionParameters_TypeSpecific(
    const SensorSpec::ptr& spec) {
  hfov_ = std::atof(spec_->parameters.at("hfov").c_str());
}

Mn::Matrix4 PinholeCamera::recalcBaseProjectionMatrix() {
  const float aspectRatio = static_cast<float>(width_) / height_;
  return Mn::Matrix4::perspectiveProjection(Mn::Deg{hfov_}, aspectRatio, near_,
                                            far_);
}

}  // namespace sensor
}  // namespace esp
