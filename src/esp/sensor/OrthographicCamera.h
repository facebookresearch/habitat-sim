// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_ORTHOGRAPHICCAMERA_H_
#define ESP_SENSOR_ORTHOGRAPHICCAMERA_H_

#include "CameraSensor.h"
#include "esp/core/esp.h"

namespace esp {
namespace sensor {
class OrthographicCamera : public CameraSensor {
 public:
  explicit OrthographicCamera(scene::SceneNode& cameraNode,
                              const SensorSpec::ptr& spec)
      : sensor::CameraSensor(cameraNode, spec) {
    setProjectionParameters(spec);
  }
  virtual ~OrthographicCamera() {}

 protected:
  virtual void setProjectionParameters_TypeSpecific(
      const SensorSpec::ptr& spec) override;

  /**
   * @brief Rebuild the base orthographic projection matrix for this @ref
   * OrthographicCamera
   */
  virtual Mn::Matrix4 recalcBaseProjectionMatrix() override;

  /** @brief scaling
   */
  float scale_ = .1f;

 public:
  ESP_SMART_POINTERS(OrthographicCamera)
};  // class PinholeCameara

}  // namespace sensor
}  // namespace esp
#endif  // ESP_SENSOR_ORTHOGRAPHICCAMERA_H_
