// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_EQUIRECTANGULARSENSOR_H_
#define ESP_SENSOR_EQUIRECTANGULARSENSOR_H_

#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include <Magnum/ResourceManager.h>
#include "VisualSensor.h"
#include "esp/core/esp.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/EquirectangularShader.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {



struct EquirectangularSensorSpec : public VisualSensorSpec {
  /**
   * @brief Constructor
   */
  EquirectangularSensorSpec();

  /**
   * @brief operator ==, check if 2 specs are equal
   */
  bool operator==(const EquirectangularSensorSpec& a) const;
  /**
   * @brief check if the specification is legal
   */
  void sanityCheck() override;
  ESP_SMART_POINTERS(EquirectangularSensorSpec)
};

class EquirectangularSensor : public VisualSensor {
 public:
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
   * @brief Returns the parameters needed to unproject depth for the sensor.
   *
   * Will always be @ref Corrade::Containers::NullOpt for the base sensor
   * class as it has no projection parameters
   */
  Corrade::Containers::Optional<Magnum::Vector2> depthUnprojection()
      const override;

  static constexpr const char* FISH_EYE_SHADER_KEY_TEMPLATE =
      "equirectangular-model-type={}-flags={}";

  gfx::RenderCamera* getRenderCamera() = delete;

 protected:
  EquirectangularSensorSpec::ptr equirectangularSensorSpec_ =
      std::dynamic_pointer_cast<EquirectangularSensorSpec>(spec_);
  // raw pointer only, we can create it but let magnum to handle the memory
  // recycling when releasing it.
  gfx::CubeMapCamera* cubeMapCamera_;
  std::unique_ptr<esp::gfx::CubeMap> cubeMap_ = nullptr;

  // Magnum::Resource<gfx::EquirectangularShader> shader_;
  // a big triangles that covers the whole screen
  Magnum::GL::Mesh mesh_;

  gfx::EquirectangularShader* shader_;

  gfx::EquirectangularShader::Flags equirectangularShaderFlags_{};

  ESP_SMART_POINTERS(EquirectangularSensor)
};


}  // namespace sensor
}  // namespace esp

#endif
