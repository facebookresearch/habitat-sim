// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_EQUIRECTANGULARSENSOR_H_
#define ESP_SENSOR_EQUIRECTANGULARSENSOR_H_

#include "CubeMapSensorBase.h"
#include "VisualSensor.h"

#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include <Magnum/ResourceManager.h>
#include "esp/core/Esp.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/EquirectangularShader.h"
#include "esp/gfx/RenderTarget.h"

namespace esp {

// forward declaration
namespace sim {
class Simulator;
}

namespace sensor {

struct EquirectangularSensorSpec : public CubeMapSensorBaseSpec {
  /**
   * @brief Constructor
   */
  EquirectangularSensorSpec();

  /**
   * @brief check if the specification is legal
   */
  void sanityCheck() const override;
  ESP_SMART_POINTERS(EquirectangularSensorSpec)
};

class EquirectangularSensor : public CubeMapSensorBase {
 public:
  static constexpr const char* EQUIRECTANGULAR_SHADER_KEY_TEMPLATE =
      "equirectangular-flags={}";
  /**
   * @brief constructor
   * NOTE: the status of the camera sensor is "valid" after construction, and
   * user can use them immediately
   */
  explicit EquirectangularSensor(scene::SceneNode& cameraNode,
                                 const EquirectangularSensorSpec::ptr& spec);
  /**
   * @brief destructor
   */
  ~EquirectangularSensor() override = default;

  /**
   * @brief Draw an observation to the frame buffer
   * @return true if success, otherwise false (e.g., frame buffer is not set)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn
   */
  bool drawObservation(sim::Simulator& sim) override;

  /**
   * @brief Return a pointer to this fisheye sensor's SensorSpec
   */
  EquirectangularSensorSpec::ptr specification() const {
    return equirectangularSensorSpec_;
  }

  gfx::RenderCamera* getRenderCamera() = delete;

 protected:
  EquirectangularSensorSpec::ptr equirectangularSensorSpec_ =
      std::dynamic_pointer_cast<EquirectangularSensorSpec>(spec_);
  Magnum::ResourceKey getShaderKey() override;
  ESP_SMART_POINTERS(EquirectangularSensor)
};

}  // namespace sensor
}  // namespace esp

#endif
