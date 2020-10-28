// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_PINHOLECAMERA_H_
#define ESP_SENSOR_PINHOLECAMERA_H_

#include "VisualSensor.h"
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

class PinholeCamera : public VisualSensor {
 public:
  // constructor: the status of the pinhole camera is "valid" after
  // construction;
  // user can use them immediately
  explicit PinholeCamera(scene::SceneNode& pinholeCameraNode,
                         const SensorSpec::ptr& spec);

  void setProjectionParameters(const SensorSpec::ptr& spec);

  virtual ~PinholeCamera() {}

  // set the projection matrix to the given render camera
  virtual PinholeCamera& setProjectionMatrix(
      gfx::RenderCamera& targetCamera) override;
  // set the transformation matrix to the given render camera
  virtual PinholeCamera& setTransformationMatrix(
      gfx::RenderCamera& targetCamera) override;
  // set the view port to the given render camera
  virtual PinholeCamera& setViewport(gfx::RenderCamera& targetCamera) override;

  virtual bool getObservation(sim::Simulator& sim, Observation& obs) override;

  virtual bool getObservationSpace(ObservationSpace& space) override;

  virtual bool displayObservation(sim::Simulator& sim) override;

  /**
   * @brief Returns the parameters needed to unproject depth for this sensor's
   * perspective projection model.
   * See @ref gfx::calculateDepthUnprojection
   */
  virtual Corrade::Containers::Optional<Magnum::Vector2> depthUnprojection()
      const override;

  /**
   * @brief Draw an observation to the frame buffer using simulator's renderer
   * @return true if success, otherwise false (e.g., frame buffer is not set)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn
   */
  virtual bool drawObservation(sim::Simulator& sim) override;

 protected:
  // projection parameters
  int width_ = 640;      // canvas width
  int height_ = 480;     // canvas height
  float near_ = 0.001f;  // near clipping plane
  float far_ = 1000.0f;  // far clipping plane
  float hfov_ = 35.0f;   // field of vision (in degrees)

  ESP_SMART_POINTERS(PinholeCamera)

  /**
   * @brief Read the observation that was rendered by the simulator
   * @param[in,out] obs Instance of Observation class in which the observation
   *                    will be stored
   */
  void readObservation(Observation& obs);
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_PINHOLECAMERA_H_
