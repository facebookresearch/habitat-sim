// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_VISUALSENSOR_H_
#define ESP_SENSOR_VISUALSENSOR_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/Math/Angle.h>
#include <Magnum/Math/ConfigurationValue.h>

#include "esp/core/Check.h"
#include "esp/core/Esp.h"

#include "esp/gfx/RenderCamera.h"
#include "esp/sensor/Sensor.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {
class RenderTarget;
}  // namespace gfx

namespace sensor {

using Mn::Math::Literals::operator""_degf;

struct VisualSensorSpec : public SensorSpec {
  /**
   * @brief height x width
   */
  vec2i resolution = {128, 128};
  /**
   * @brief Number of components in buffer values, eg. 4 channels for RGBA
   */
  int channels = 4;
  /**
   * @brief True for pytorch tensor support
   */
  bool gpu2gpuTransfer = false;
  /**
   * @brief near clipping plane
   */
  float near = 0.01f;
  /**
   * @brief far clipping plane
   */
  float far = 1000.0f;
  /**
   * @brief color used to clear the framebuffer
   */
  Mn::Color4 clearColor = {0, 0, 0, 1};
  VisualSensorSpec();
  void sanityCheck() const override;
  bool isVisualSensorSpec() const override { return true; }
  bool operator==(const VisualSensorSpec& a) const;
  ESP_SMART_POINTERS(VisualSensorSpec)
};
// Represents a sensor that provides visual data from the environment to an
// agent
class VisualSensor : public Sensor {
 public:
  explicit VisualSensor(scene::SceneNode& node, VisualSensorSpec::ptr spec);

  /**
   * @brief Return the size of the framebuffer corresponding to the sensor's
   * resolution as a [W, H] Vector2i
   */
  Magnum::Vector2i framebufferSize() const {
    // NB: The sensor's resolution is in H x W format as that more cleanly
    // corresponds to the practice of treating images as arrays that is used in
    // modern CV and DL. However, graphics frameworks expect W x H format for
    // frame buffer sizes
    return {visualSensorSpec_->resolution[1], visualSensorSpec_->resolution[0]};
  }

  /* @param[in] sim Instance of Simulator class for which the observation needs
   *                to be displayed
   * @return Whether the display process was successful or not
   */
  bool displayObservation(sim::Simulator& sim) override;

  /**
   * @brief Return whether or not this Sensor is a VisualSensor
   */
  bool isVisualSensor() const override { return true; }

  /**
   * @brief Returns the parameters needed to unproject depth for the sensor.
   */
  virtual Corrade::Containers::Optional<Magnum::Vector2> depthUnprojection()
      const;

  /**
   * @brief Checks to see if this sensor has a RenderTarget bound or not
   */
  bool hasRenderTarget() const { return tgt_ != nullptr; }

  /**
   * @brief Binds the given given RenderTarget to the sensor.  The sensor takes
   * ownership of the RenderTarget
   */
  void bindRenderTarget(std::unique_ptr<gfx::RenderTarget>&& tgt);

  /**
   * @brief Returns a reference to the sensors render target
   */
  gfx::RenderTarget& renderTarget() {
    ESP_CHECK(hasRenderTarget(),
              "VisualSensor::renderTarget(): Sensor has no rendering target");
    return *tgt_;
  }

  /**
   * @brief Draw an observation to the frame buffer using simulator's renderer
   * @return true if success, otherwise false (e.g., frame buffer is not set)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn
   */
  virtual bool drawObservation(CORRADE_UNUSED sim::Simulator& sim) = 0;

  /**
   * @brief Read the observation that was rendered by the simulator
   * @param[in,out] obs Instance of Observation class in which the observation
   * will be stored
   */
  virtual void readObservation(Observation& obs);

  /*
   * @brief Display next observation from Simulator on default frame buffer
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
   * @brief Sets resolution of Sensor's sensorSpec
   */
  void setResolution(int height, int width) {
    CORRADE_ASSERT(height > 0 && width > 0,
                   "VisualSensor::setResolution(): resolution height and "
                   "width must be greater than 0", );
    visualSensorSpec_->resolution = {height, width};
  }

  void setResolution(vec2i resolution) {
    CORRADE_ASSERT(resolution[0] > 0 && resolution[1] > 0,
                   "VisualSensor::setResolution(): resolution height and "
                   "width must be greater than 0", );
    visualSensorSpec_->resolution = {resolution[0], resolution[1]};
  }

  /**
   * @brief Return a pointer to this visual sensor's SensorSpec
   */
  VisualSensorSpec::ptr specification() const { return visualSensorSpec_; }

  /**
   * @brief Return this sensor's projection matrix
   */
  virtual Mn::Matrix4 getProjectionMatrix() const = 0;

  /**
   * @brief Returns RenderCamera
   */
  virtual gfx::RenderCamera* getRenderCamera() const { return nullptr; }

  /**
   * @brief Gets near plane distance.
   */
  float getNear() const { return visualSensorSpec_->near; }

  /**
   * @brief Gets far plane distance.
   */
  float getFar() const { return visualSensorSpec_->far; }

  /**
   * @brief Returns the FOV of this Sensor
   */
  Mn::Deg getFOV() const { return hfov_; }

  /**
   * @brief Return whether or not this Visual Sensor can use the HBAO effect
   */
  bool canUseHBAO() const override {
    // TODO Expand HBAO support to other visual sensors
    return (visualSensorSpec_->sensorSubType == SensorSubType::Pinhole) ||
           (visualSensorSpec_->sensorSubType == SensorSubType::Orthographic);
  }

 protected:
  /** @brief field of view
   */
  Mn::Deg hfov_ = 90.0_degf;

  std::unique_ptr<gfx::RenderTarget> tgt_;
  VisualSensorSpec::ptr visualSensorSpec_ =
      std::dynamic_pointer_cast<VisualSensorSpec>(spec_);

  class MoveSemanticSensorNodeHelper {
   public:
    /**
     * @brief constructor.
     * This function saves the semantic sensor's transformation and parent node,
     * moves it to the semantic scene graph, connects it to the root node, and
     * sets the relative transformation (wrt to the root) to the absolute
     * transformation in the previous scene graph.
     */
    MoveSemanticSensorNodeHelper(VisualSensor& visualSensor,
                                 sim::Simulator& sim);
    /**
     * @brief Destructor.
     * This function moves the semantic sensor back to the regular scene graph,
     * restores its relative transformation to its parent based on the values
     * saved previously.
     */
    ~MoveSemanticSensorNodeHelper();

   protected:
    VisualSensor& visualSensor_;
    sim::Simulator& sim_;
    Corrade::Containers::Optional<Magnum::Matrix4> relativeTransformBackup_ =
        Corrade::Containers::NullOpt;
    scene::SceneNode* semanticSensorParentNodeBackup_ = nullptr;
  };

  ESP_SMART_POINTERS(VisualSensor)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_VISUALSENSOR_H_
