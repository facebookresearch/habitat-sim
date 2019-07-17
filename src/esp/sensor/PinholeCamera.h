// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Sensor.h"
#include "esp/core/esp.h"

namespace esp {
namespace sensor {

// TODO:
// in the future, if the system has more than 1 visual sensor,
// Consider to abstract a "VisualSensor" class, which is the subclass
// of Sensor, and base class for the actual visual sensors, such as
// PinholeCamera.
// then the projection parameters, such as width_, height_ etc. cant be stored
// in the "VisualSensor" class

class PinholeCamera : public Sensor {
 public:
  // constructor: the status of the pinhole camera is "valid" after
  // construction;
  // user can use them immediately
  explicit PinholeCamera(scene::SceneNode& pinholeCameraNode,
                         SensorSpec::ptr spec);

  void setProjectionParameters(SensorSpec::ptr spec);

  virtual ~PinholeCamera() {}

  bool isVisualSensor() override { return true; }

  // set the projection parameters to the given render camera
  virtual void setProjectionMatrix(gfx::RenderCamera& targetCamera) override;

 protected:
  // projection parameters
  int width_ = 640;      // canvas width
  int height_ = 480;     // canvas height
  float near_ = 0.001f;  // near clipping plane
  float far_ = 1000.0f;  // far clipping plane
  float hfov_ = 35.0f;   // field of vision (in degrees)

  ESP_SMART_POINTERS(PinholeCamera)
};

}  // namespace sensor
}  // namespace esp
