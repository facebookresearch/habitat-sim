// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "OrthographicCamera.h"

namespace esp {
namespace sensor {

void OrthographicCamera::setProjectionParameters_TypeSpecific(
    const SensorSpec::ptr& spec) {
  scale_ = std::atof(spec_->parameters.at("ortho_scale").c_str());
}
Mn::Matrix4 OrthographicCamera::recalcBaseProjectionMatrix() {
  auto size = Mn::Vector2{width_ / (1.0f * height_), 1.0f};
  size /= scale_;
  return Mn::Matrix4::orthographicProjection(size, near_, far_);
}

}  // namespace sensor
}  // namespace esp
