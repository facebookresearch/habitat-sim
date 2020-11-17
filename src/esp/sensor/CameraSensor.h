// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_CAMERASENSOR_H_
#define ESP_SENSOR_CAMERASENSOR_H_

#include "VisualSensor.h"
#include "esp/core/esp.h"

namespace esp {
namespace sensor {

class CameraSensor : public VisualSensor {
 public:
  // constructor: the status of the camera sensor is "valid" after
  // construction;
  // user can use them immediately
  explicit CameraSensor(scene::SceneNode& cameraNode,
                        const SensorSpec::ptr& spec);
  virtual ~CameraSensor() {}

  void setProjectionParameters(const SensorSpec::ptr& spec);

  // set the projection matrix to the given render camera
  virtual CameraSensor& setProjectionMatrix(
      gfx::RenderCamera& targetCamera) override;
  // set the transformation matrix to the given render camera
  virtual CameraSensor& setTransformationMatrix(
      gfx::RenderCamera& targetCamera) override;
  // set the view port to the given render camera
  virtual CameraSensor& setViewport(gfx::RenderCamera& targetCamera) override;

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

  /**
   * @brief Modify the zoom matrix for perspective and ortho cameras
   * @param factor Modification amount.
   */
  void setZoom(float factor) {
    float before = zoomMatrix_[0][0];
    zoomMatrix_ =
        Magnum::Matrix4::scaling({factor, factor, 1.0f}) * zoomMatrix_;
    recomputeProjectionMatrix();
  }

  /**
   * @brief Sets camera type and calculates appropriate size vector for display.
   */
  void setCameraType(const SensorSubType& _cameraType);

  SensorSubType getCameraType() const { return cameraType_; }

 protected:
  /**
   * @brief Recalculate the base projection matrix, based on camera type and
   * display size. This should be called only when camera type, size or clipping
   * planes change.
   */
  void recomputeBaseProjectionMatrix();

  /**
   * @brief Recalculate the projection Matrix used by this Camera Sensor, which
   * should be recomputeulated @ref zoomMatrix_ or @ref baseProjMatrix_ change.
   */
  void recomputeProjectionMatrix() {
    projectionMatrix_ = zoomMatrix_ * baseProjMatrix_;
  }

  /**
   * @brief Read the observation that was rendered by the simulator
   * @param[in,out] obs Instance of Observation class in which the observation
   * will be stored
   */
  void readObservation(Observation& obs);

  /**
   * @brief This camera's projection matrix. Should be recomputeulated every
   * time size changes.
   */
  Magnum::Matrix4 projectionMatrix_;

  /**
   * @Brief A base projection matrix based on camera's type and display size.
   */
  Magnum::Matrix4 baseProjMatrix_;

  /**
   * @brief A matrix to determine the zoom for the projection.
   */
  Magnum::Matrix4 zoomMatrix_;

  /** @brief projection parameters
   */

  /** @brief Camera type
   */
  SensorSubType cameraType_ = sensor::SensorSubType::Pinhole;

  /** @brief canvas width
   */
  int width_ = 640;

  /** @brief canvas height
   */
  int height_ = 480;

  /** @brief near clipping plane
   */
  float near_ = 0.001f;

  /** @brief far clipping plane
   */
  float far_ = 1000.0f;

  /** @brief size of near plane
   */
  Mn::Vector2 size_;

 public:
  ESP_SMART_POINTERS(CameraSensor)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_CAMERASENSOR_H_
