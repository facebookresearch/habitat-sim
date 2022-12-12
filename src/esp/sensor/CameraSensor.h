// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_CAMERASENSOR_H_
#define ESP_SENSOR_CAMERASENSOR_H_

#include <Magnum/Math/ConfigurationValue.h>
#include "VisualSensor.h"
#include "esp/core/Esp.h"

namespace esp {
namespace sensor {

struct CameraSensorSpec : public VisualSensorSpec {
  float orthoScale = 0.1f;
  Mn::Deg hfov = 90.0_degf;
  CameraSensorSpec();
  void sanityCheck() const override;
  bool operator==(const CameraSensorSpec& a) const;
  Magnum::Matrix4 projectionMatrix() const;
  ESP_SMART_POINTERS(CameraSensorSpec)
};

class CameraSensor : public VisualSensor {
 public:
  // constructor: the status of the camera sensor is "valid" after
  // construction;
  // user can use them immediately
  explicit CameraSensor(scene::SceneNode& cameraNode,
                        const CameraSensorSpec::ptr& spec);

  /** @brief Updates this sensor's CameraSensorSpec cameraSensorSpec_ to reflect
   * the passed new values
   *  @param[in] spec Instance of CameraSensorSpec that sensor will use to
   * update its own SensorSpec
   */
  void setProjectionParameters(const CameraSensorSpec& spec);

  /** @brief Returns pointer to member RenderCamera that CameraSensor will use
   * for rendering
   */
  gfx::RenderCamera* getRenderCamera() const override;

  /**
   * @brief Returns the parameters needed to unproject depth for this sensor's
   * perspective projection model.
   * See @ref gfx::calculateDepthUnprojection
   */
  Corrade::Containers::Optional<Magnum::Vector2> depthUnprojection()
      const override;

  /**
   * @brief Draw an observation to the frame buffer using simulator's renderer
   * @return true if success, otherwise false (e.g., frame buffer is not set)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn
   */
  bool drawObservation(sim::Simulator& sim) override;

  /**
   * @brief Modify the zoom matrix for perspective and ortho cameras
   * @param factor Modification amount.
   */
  void modifyZoom(float factor) {
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
    hfov_ = FOV;
    if (cameraSensorSpec_->sensorSubType != SensorSubType::Pinhole) {
      ESP_DEBUG()
          << "Only Perspective-base CameraSensors use "
             "FOV. Specified value saved but will not be consumed by this "
             "CameraSensor.";
    }
    recomputeBaseProjectionMatrix();
  }  // CameraSensor::setFOV

  /**
   * @brief Sets camera type and calculates appropriate size vector for
   * display.
   */
  void setCameraType(const SensorSubType& _cameraType) {
    CORRADE_ASSERT(_cameraType == SensorSubType::Pinhole ||
                       _cameraType == SensorSubType::Orthographic,
                   "CameraSensor::setCameraType(): _cameraType is not "
                   "SensorSubType "
                   "Pinhole or Orthographic", );
    cameraSensorSpec_->sensorSubType = _cameraType;
    recomputeBaseProjectionMatrix();
  }  // CameraSensor::setCameraType

  /**
   * @brief Returns the camera type of this Sensor
   */
  SensorSubType getCameraType() const {
    return cameraSensorSpec_->sensorSubType;
  }

  /**
   * @brief Sets width of this sensor's view port
   */
  void setWidth(int width) {
    CORRADE_ASSERT(width > 0,
                   "CameraSensor::setWidth(): resolution height and "
                   "width must be greater than 0", );
    cameraSensorSpec_->resolution[1] = width;
    recomputeBaseProjectionMatrix();
  }

  /**
   * @brief Sets height of this sensor's view port
   */
  void setHeight(int height) {
    CORRADE_ASSERT(height > 0,
                   "CameraSensor::setHeight(): resolution height and "
                   "width must be greater than 0", );
    cameraSensorSpec_->resolution[0] = height;
    recomputeBaseProjectionMatrix();
  }

  /**
   * @brief Sets near plane distance.
   */
  void setNear(float _near) {
    CORRADE_ASSERT(_near > 0,
                   "CameraSensor::setNear(): near plane distance must be "
                   "greater than 0", );
    cameraSensorSpec_->near = _near;
    recomputeBaseProjectionMatrix();
  }

  /**
   * @brief Sets far plane distance.
   */
  void setFar(float _far) {
    CORRADE_ASSERT(
        _far > 0,
        "CameraSensor::setFar(): Far plane distance must be greater than 0", );
    cameraSensorSpec_->far = _far;
    recomputeBaseProjectionMatrix();
  }

  /**
   * @brief Return a pointer to this camera sensor's SensorSpec
   */
  CameraSensorSpec::ptr specification() const { return cameraSensorSpec_; }

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
  void recomputeProjectionMatrix();

  /**
   * @brief Draw the scene graph with the specified camera flag
   * @param[in] sceneGraph scene graph to be drawn
   * @param[in] flags flag for the render camera
   */
  void draw(scene::SceneGraph& sceneGraph, gfx::RenderCamera::Flags flags);

  /**
   * @brief This camera's projection matrix. Should be recomputeulated every
   * time size changes.
   */
  Magnum::Matrix4 projectionMatrix_;

  /**
   * @brief A base projection matrix based on camera's type and display size.
   */
  Magnum::Matrix4 baseProjMatrix_;

  /**
   * @brief A matrix to determine the zoom for the projection.
   */
  Magnum::Matrix4 zoomMatrix_;

  /** @brief size of near plane
   */
  Mn::Vector2 nearPlaneSize_;

  /** @brief raw pointer to member RenderCamera that CameraSensor will use for
   * rendering
   */
  gfx::RenderCamera* renderCamera_;

  CameraSensorSpec::ptr cameraSensorSpec_ =
      std::dynamic_pointer_cast<CameraSensorSpec>(spec_);

 public:
  ESP_SMART_POINTERS(CameraSensor)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_CAMERASENSOR_H_
