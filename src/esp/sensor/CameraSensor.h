// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_CAMERASENSOR_H_
#define ESP_SENSOR_CAMERASENSOR_H_

#include <Magnum/Math/ConfigurationValue.h>
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
  void modZoom(float factor) {
    zoomMatrix_ =
        Magnum::Matrix4::scaling({factor, factor, 1.0f}) * zoomMatrix_;
    recomputeProjectionMatrix();
  }

  /**
   * @brief Resets the zoom on this CameraSensor to reflect current sensor spec
   * values.
   */
  void resetZoom() {
    zoomMatrix_ = Magnum::Matrix4(Magnum::Math::IdentityInit);
    recomputeProjectionMatrix();
  }

  // ======== Accessors ========

  /**
   * @brief Sets the FOV for this CameraSensor.  Only consumed by
   * pinhole/perspective cameras.
   * @param FOV desired FOV to set.
   */
  void setFOV(Mn::Deg FOV) {
    spec_->parameters.at("hfov") =
        Corrade::Utility::ConfigurationValue<Mn::Deg>::toString(
            FOV, Corrade::Utility::ConfigurationValueFlags());
    if (spec_->sensorSubType != SensorSubType::Pinhole) {
      LOG(INFO)
          << "CameraSensor::setFOV : Only Perspective-base CameraSensors use "
             "FOV. Specified value saved but will not be consumed by this "
             "CameraSensor.";
    }
    recomputeBaseProjectionMatrix();
  }  // CameraSensor::setFOV

  Mn::Deg getFOV() const {
    float fov = std::atof(spec_->parameters.at("hfov").c_str());
    return Mn::Deg{fov};
  }

  /**
   * @brief Sets camera type and calculates appropriate size vector for
   * display.
   */
  void setCameraType(const SensorSubType& _cameraType) {
    spec_->sensorSubType = _cameraType;
    recomputeBaseProjectionMatrix();
  }  // CameraSensor::setCameraType

  SensorSubType getCameraType() const { return spec_->sensorSubType; }

  /**
   * @brief Sets width of this sensor's view port
   */
  void setWidth(int _width) {
    spec_->resolution[1] = _width;
    width_ = _width;
    recomputeBaseProjectionMatrix();
  }
  int getWidth() const { return width_; }

  /**
   * @brief Sets height of this sensor's view port
   */
  void setHeight(int _height) {
    spec_->resolution[0] = _height;
    height_ = _height;
    recomputeBaseProjectionMatrix();
  }
  int getHeight() const { return height_; }

  /**
   * @brief Sets near plane distance.
   */
  void setNear(float _near) {
    spec_->parameters.at("near") = std::to_string(_near);
    near_ = _near;
    recomputeBaseProjectionMatrix();
  }
  float getNear() { return near_; }

  /**
   * @brief Sets far plane distance.
   */
  void setFar(float _far) {
    spec_->parameters.at("far") = std::to_string(_far);
    far_ = _far;
    recomputeBaseProjectionMatrix();
  }
  float getFar() { return far_; }

 protected:
  /**
   * @brief Recalculate the base projection matrix, based on camera type and
   * display size. This should be called only when camera type, size or
   * clipping planes change.
   */
  void recomputeBaseProjectionMatrix();

  /**
   * @brief Recalculate the projection Matrix used by this Camera Sensor,
   * which should be recomputeulated @ref zoomMatrix_ or @ref baseProjMatrix_
   * change.
   */
  void recomputeProjectionMatrix() {
    projectionMatrix_ = zoomMatrix_ * baseProjMatrix_;
  }

  /**
   * @brief Read the observation that was rendered by the simulator
   * @param[in,out] obs Instance of Observation class in which the observation
   * will be stored
   */
  virtual void readObservation(Observation& obs);

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
  Mn::Vector2 nearPlaneSize_;

 public:
  ESP_SMART_POINTERS(CameraSensor)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_CAMERASENSOR_H_
