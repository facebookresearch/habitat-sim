// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/Optional.h>

#include "esp/core/esp.h"

#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {

// Represents a sensor that provides visual data from the environment to an
// agent
class VisualSensor : public Sensor {
 public:
  using Sensor::Sensor;
  virtual ~VisualSensor() {}

  /**
   * @brief Return the size of the framebuffer corresponding to the sensor's
   * resolution as a [W, H] Vector2i
   */
  Magnum::Vector2i framebufferSize() const {
    // NB: The sensor's resolution is in H x W format as that more cleanly
    // corresponds to the practice of treating images as arrays that is used in
    // modern CV and DL. However, graphics frameworks expect W x H format for
    // frame buffer sizes
    return {spec_->resolution[1], spec_->resolution[0]};
  }

  virtual bool isVisualSensor() override { return true; }

  // visual sensor should implement and override the following functions
  /**
   * @brief set the projection matrix from sensor to the render camera
   * @return Reference to self (for method chaining)
   */
  virtual VisualSensor& setProjectionMatrix(
      CORRADE_UNUSED gfx::RenderCamera& targetCamera) {
    return *this;
  }
  /**
   * @brief set the transform matrix (modelview) from sensor to the render
   * camera
   * @return Reference to self (for method chaining)
   */
  virtual VisualSensor& setTransformationMatrix(
      CORRADE_UNUSED gfx::RenderCamera& targetCamera) {
    return *this;
  }
  /**
   * @brief set the viewport from sensor to the render camera
   * @return Reference to self (for method chaining)
   */
  virtual VisualSensor& setViewport(
      CORRADE_UNUSED gfx::RenderCamera& targetCamera) {
    return *this;
  }

  /**
   * @brief Returns the parameters needed to unproject depth for the sensor.
   *
   * Will always be @ref Corrade::Containers::NullOpt for the base sensor class
   * as it has no projection parameters
   */
  virtual Corrade::Containers::Optional<Magnum::Vector2> depthUnprojection()
      const {
    return Corrade::Containers::NullOpt;
  };

  /**
   * @brief Checks to see if this sensor has a RenderTarget bound or not
   */
  bool hasRenderTarget() const { return tgt_ != nullptr; }

  /**
   * @brief Binds the given given RenderTarget to the sensor.  The sensor takes
   * ownership of the RenderTarget
   */
  void bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
    if (tgt->framebufferSize() != framebufferSize())
      throw std::runtime_error("RenderTarget is not the correct size");
    tgt_ = std::move(tgt);
  }

  /**
   * @brief Returns a reference to the sensors render target
   */
  gfx::RenderTarget& renderTarget() {
    if (!hasRenderTarget())
      throw std::runtime_error("Sensor has no rendering target");
    return *tgt_;
  }

 protected:
  gfx::RenderTarget::uptr tgt_ = nullptr;

  ESP_SMART_POINTERS(VisualSensor)
};

}  // namespace sensor
}  // namespace esp
