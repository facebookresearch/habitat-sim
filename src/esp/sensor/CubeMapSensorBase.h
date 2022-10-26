// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_CUBEMAP_SENSOR_BASE_H_
#define ESP_SENSOR_CUBEMAP_SENSOR_BASE_H_
#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include "VisualSensor.h"
#include "esp/core/Esp.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/CubeMapShaderBase.h"

namespace esp {

// forward declaration
namespace sim {
class Simulator;
}

namespace sensor {

struct CubeMapSensorBaseSpec : public VisualSensorSpec {
  /**
   * @brief the size of the cubemap
   */
  Corrade::Containers::Optional<int> cubemapSize = Corrade::Containers::NullOpt;

  /**
   * @brief Constructor
   */
  CubeMapSensorBaseSpec();

  /**
   * @brief operator ==, check if 2 specs are equal
   */
  bool operator==(const CubeMapSensorBaseSpec& a) const;

  /**
   * @brief check if the specification is legal
   */
  void sanityCheck() const override;
  ESP_SMART_POINTERS(CubeMapSensorBaseSpec)
};

class CubeMapSensorBase : public VisualSensor {
 public:
  gfx::RenderCamera* getRenderCamera() = delete;

  /**
   * @brief destructor
   */
  ~CubeMapSensorBase() override = default;

 protected:
  /**
   * @brief constructor
   * NOTE: the status of the camera sensor is "valid" after construction, and
   * user can use them immediately
   */
  explicit CubeMapSensorBase(scene::SceneNode& cameraNode,
                             const CubeMapSensorBaseSpec::ptr& spec);

  CubeMapSensorBaseSpec::ptr cubeMapSensorBaseSpec_ =
      std::dynamic_pointer_cast<CubeMapSensorBaseSpec>(spec_);
  // raw pointer only, we can create it but let magnum to handle the memory
  // recycling when releasing it.
  gfx::CubeMapCamera* cubeMapCamera_;
  Corrade::Containers::Optional<esp::gfx::CubeMap> cubeMap_;

  // a big triangles that covers the whole screen
  Magnum::GL::Mesh mesh_;

  // cubemap shader resource manager, which manages different shaders such as
  // DoubleSphereCameraShader, FieldOfViewCameraShader (TODO),
  // EquiRectangularShader ...
  Magnum::ResourceManager<gfx::CubeMapShaderBase> cubeMapShaderBaseManager_;

  gfx::CubeMapShaderBase::Flags cubeMapShaderBaseFlags_{};

  virtual Magnum::ResourceKey getShaderKey() = 0;

  template <typename T>
  Magnum::Resource<gfx::CubeMapShaderBase, T> getShader();

  /**
   * @brief render the sense into cubemap textures
   * @param[in] sim th simulator instance
   */
  bool renderToCubemapTexture(sim::Simulator& sim);

  /**
   * @brief draw the observation with the shader
   * NOTE: assume the cubemap texture is already generated
   */
  void drawWith(gfx::CubeMapShaderBase& shader);

  ESP_SMART_POINTERS(CubeMapSensorBase)
};

template <typename T>
Magnum::Resource<gfx::CubeMapShaderBase, T> CubeMapSensorBase::getShader() {
  Magnum::Resource<gfx::CubeMapShaderBase, T> shader =
      cubeMapShaderBaseManager_.get<gfx::CubeMapShaderBase, T>(getShaderKey());
  if (!shader) {
    cubeMapShaderBaseManager_.set<gfx::CubeMapShaderBase>(
        shader.key(), new T{cubeMapShaderBaseFlags_},
        Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
  }

  CORRADE_INTERNAL_ASSERT(shader && shader->flags() == cubeMapShaderBaseFlags_);

  return shader;
}

}  // namespace sensor
}  // namespace esp
#endif
