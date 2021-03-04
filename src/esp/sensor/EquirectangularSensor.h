// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_EQUIRECTANGULARSENSOR_H_
#define ESP_SENSOR_EQUIRECTANGULARSENSOR_H_

#include <Magnum/Math/ConfigurationValue.h>
#include "VisualSensor.h"
#include "esp/core/esp.h"
#include "esp/gfx/CubeMapCamera.h"

namespace esp {
namespace sensor {

struct EquirectangularSensorSpec : public VisualSensorSpec {
  int channels = 4;
  // description of Sensor observation space as gym.spaces.Dict()
  std::string observationSpace = "";
  EquirectangularSensorSpec();
  void sanityCheck() override;
  bool operator==(const EquirectangularSensorSpec& a) const;
  ESP_SMART_POINTERS(EquirectangularSensorSpec)
};

class EquirectangularSensor : public VisualSensor {
 public:
  // constructor: the status of the camera sensor is "valid" after
  // construction;
  // user can use them immediately
  explicit EquirectangularSensor(scene::SceneNode& cameraNode,
                        const EquirectangularSensorSpec::ptr& spec);
  ~EquirectangularSensor() override { LOG(INFO) << "Deconstructing EquirectangularSensor"; }

  /** @brief Updates this sensor's EquirectangularSensorSpec cameraSensorSpec_ to reflect
   * the passed new values
   *  @param[in] spec Instance of EquirectangularSensorSpec that sensor will use to
   * update its own SensorSpec
   */
  void setProjectionParameters(const EquirectangularSensorSpec& spec);

  /** @brief Returns pointer to member RenderCamera that EquirectangularSensor will use
   * for rendering
   */
  gfx::CubeMapCamera* getRenderCamera() const override;

  /**
   * @brief Draws an observation to the frame buffer using simulator's renderer,
   * then reads the observation to the sensor's memory buffer
   * @return true if success, otherwise false (e.g., failed to draw or read
   * observation)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn, obs Instance of Observation class in which the
   * observation will be stored
   */
  bool getObservation(sim::Simulator& sim, Observation& obs) override;

  /**
   * @brief Updates ObservationSpace space with spaceType, shape, and dataType
   * of this sensor. The information in space is later used to resize the
   * sensor's memory buffer if sensor is resized.
   * @return true if success, otherwise false
   * @param[in] space Instance of ObservationSpace class which will be updated
   * with information from this sensor
   */
  bool getObservationSpace(ObservationSpace& space) override;

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
   * @brief Resets the zoom on this EquirectangularSensor to reflect current sensor spec
   * values.
   */
  void resetZoom() {
    zoomMatrix_ = Magnum::Matrix4(Magnum::Math::IdentityInit);
    recomputeProjectionMatrix();
  }

  // ======== Accessors ========

  /**
   * @brief Returns the camera type of this Sensor
   */
  SensorSubType getCameraType() const {
    return cameraSensorSpec_->sensorSubType;
  }


 protected:

  /**
   * @brief Read the observation that was rendered by the simulator
   * @param[in,out] obs Instance of Observation class in which the observation
   * will be stored
   */
  virtual void readObservation(Observation& obs);


  /** @brief raw pointer to member RenderCamera that EquirectangularSensor will use for
   * rendering
   */
  gfx::CubeMapCamera* renderCamera_;

  EquirectangularSensorSpec::ptr cameraSensorSpec_ =
      std::dynamic_pointer_cast<EquirectangularSensorSpec>(spec_);

 public:
  ESP_SMART_POINTERS(EquirectangularSensor)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_EQUIRECTANGULARSENSOR_H_
