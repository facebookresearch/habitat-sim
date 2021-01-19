// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"

#include <utility>

#include "esp/gfx/RenderTarget.h"

namespace esp {
namespace sensor {
VisualSensor::VisualSensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Sensor{node, std::move(spec)}, tgt_{nullptr} {}

VisualSensor::~VisualSensor() = default;

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

}  // namespace sensor
}  // namespace esp
