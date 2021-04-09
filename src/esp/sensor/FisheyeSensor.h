// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_FISHEYESENSOR_H_
#define ESP_SENSOR_FISHEYESENSOR_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include <Magnum/ResourceManager.h>
#include "VisualSensor.h"
#include "esp/core/esp.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/FisheyeShader.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace sensor {

enum class FisheyeSensorModelType : Magnum::UnsignedInt {

  // Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
  // Camera Model, The International Conference on 3D Vision (3DV), 2018
  DoubleSphere = 0,
  // TODO:
  // User can implement her own model such as:
  // FieldOfView = 1,
  // KannalaBrandt = 2,
};

struct FisheyeSensorSpec : public VisualSensorSpec {
  FisheyeSensorModelType fisheyeModelType;
  /**
   * @brief Focal length, fx, fy, the distance between the pinhole and the image
   * plane.
   * In practice, fx and fy can differ for a number of reasons. See
   * details here: http://ksimek.github.io/2013/08/13/intrinsic/
   */
  Magnum::Vector2 focalLength;
  /**
   * @brief Principal Point Offset in pixel, cx, cy, location of the principal
   * point relative to the image plane's origin.
   */
  Magnum::Vector2 principalPointOffset;

  /**
   * @brief Constructor
   */
  FisheyeSensorSpec();

  /**
   * @brief operator ==, check if 2 specs are equal
   */
  bool operator==(const FisheyeSensorSpec& a) const;

  /**
   * @brief the size of the cubemap
   */
  Corrade::Containers::Optional<int> cubemapSize = Corrade::Containers::NullOpt;

  /**
   * @brief check if the specification is legal
   */
  void sanityCheck() override;
  ESP_SMART_POINTERS(FisheyeSensorSpec)
};

struct FisheyeSensorDoubleSphereSpec : public FisheyeSensorSpec {
  /**
   * @brief alpha and xi are specific to "double sphere" camera model.
   * see details (value ranges) in:
   * Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
   * Camera Model, The International Conference on 3D Vision (3DV), 2018
   */
  float alpha = 0.59;
  float xi = -0.18;
  /**
   * @brief constructor
   */
  FisheyeSensorDoubleSphereSpec() : FisheyeSensorSpec() {}
  /**
   * @brief check if the specification is legal
   */
  void sanityCheck() override;
  ESP_SMART_POINTERS(FisheyeSensorDoubleSphereSpec)
};

// TODO:
// struct FisheyeFieldOfViewSpec : public FisheyeSensorSpec {};
// struct FisheyeKannalaBrandtSpec : public FisheyeSensorSpec {};

class FisheyeSensor : public VisualSensor {
 public:
  /**
   * @brief constructor
   * NOTE: the status of the camera sensor is "valid" after construction, and
   * user can use them immediately
   */
  explicit FisheyeSensor(scene::SceneNode& cameraNode,
                         const FisheyeSensorSpec::ptr& spec);
  /**
   * @brief destructor
   */
  ~FisheyeSensor() override = default;
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
      "fisheye-model-type={}-flags={}";

  gfx::RenderCamera* getRenderCamera() = delete;

  /**
   * @brief Return a pointer to this fisheye sensor's SensorSpec
   */
  FisheyeSensorSpec::ptr specification() const { return fisheyeSensorSpec_; }

 protected:
  FisheyeSensorSpec::ptr fisheyeSensorSpec_ =
      std::dynamic_pointer_cast<FisheyeSensorSpec>(spec_);
  // raw pointer only, we can create it but let magnum to handle the memory
  // recycling when releasing it.
  gfx::CubeMapCamera* cubeMapCamera_;
  Corrade::Containers::Optional<esp::gfx::CubeMap> cubeMap_;

  // fisheye shader resource manager, which manages different shaders such as
  // DoubleSphereCameraShader, FieldOfViewCameraShader (TODO) ...
  Magnum::ResourceManager<gfx::FisheyeShader> fisheyeShaderManager_;
  // a big triangles that covers the whole screen
  Magnum::GL::Mesh mesh_;

  gfx::FisheyeShader::Flags fisheyeShaderFlags_{};

  Magnum::ResourceKey getShaderKey();

  template <typename T>
  Magnum::Resource<gfx::FisheyeShader, T> getShader();

  ESP_SMART_POINTERS(FisheyeSensor)
};

template <typename T>
Magnum::Resource<gfx::FisheyeShader, T> FisheyeSensor::getShader() {
  Magnum::Resource<gfx::FisheyeShader, T> shader =
      fisheyeShaderManager_.get<gfx::FisheyeShader, T>(getShaderKey());
  if (!shader) {
    fisheyeShaderManager_.set<gfx::FisheyeShader>(
        shader.key(), new T{fisheyeShaderFlags_}, Mn::ResourceDataState::Final,
        Mn::ResourcePolicy::ReferenceCounted);
  }

  CORRADE_INTERNAL_ASSERT(shader && shader->flags() == fisheyeShaderFlags_);

  return shader;
}

}  // namespace sensor
}  // namespace esp

#endif
