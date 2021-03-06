// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_SENSORINFOVISUALIZER_H_
#define ESP_GFX_SENSORINFOVISUALIZER_H_

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/Magnum.h>
#include <Magnum/ResourceManager.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class DepthVisualizerShader;

enum class SensorInfoType : Magnum::UnsignedInt {
  Depth = 0,
};

/**
@brief helper class to visualize undisplayble info such as depth, semantic ids
from the sensor on a framebuffer
*/
class SensorInfoVisualizer {
 public:
  /**
   * @brief constructor
   */
  explicit SensorInfoVisualizer();
  /**
   * @brief destructor
   */
  ~SensorInfoVisualizer() = default;

  void prepareToDraw(Magnum::Vector2i framebufferSize);

  void draw();

  template <typename T, typename... Targs>
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, T> getShader(
      SensorInfoType type,
      Targs&&... args);
  void draw(Magnum::ResourceKey shaderKey);
  void blitRgbaToDefault();

 protected:
  // framebuffer and renderbuffer for visualizing originally undisplayable
  // info (such as depth, semantic)
  Magnum::GL::Framebuffer frameBuffer_{Magnum::NoCreate};
  Magnum::GL::Renderbuffer colorBuffer_;
  Magnum::GL::Renderbuffer depthBuffer_;

  // a big triangles that covers the whole screen
  Magnum::GL::Mesh mesh_;

  Magnum::ResourceManager<Magnum::GL::AbstractShaderProgram> shaderManager_;

  Magnum::ResourceKey getShaderKey(SensorInfoType type);

  /**
   * @brief recreate the frame buffer and the render buffers,
   * attach renderbuffers (color, depth) as logical buffers of the
   * framebuffer object for visualization
   */
  void resetFramebufferRenderbuffer(Magnum::Vector2i framebufferSize);

  ESP_SMART_POINTERS(SensorInfoVisualizer)
};

template <typename T, typename... Targs>
Magnum::Resource<Magnum::GL::AbstractShaderProgram, T>
SensorInfoVisualizer::getShader(SensorInfoType type, Targs&&... args) {
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, T> shader =
      shaderManager_.get<Magnum::GL::AbstractShaderProgram, T>(
          getShaderKey(type));
  if (!shader) {
    shaderManager_.set<Magnum::GL::AbstractShaderProgram>(
        shader.key(), new T(std::forward<Targs>(args)...),
        Magnum::ResourceDataState::Final,
        Magnum::ResourcePolicy::ReferenceCounted);
  }

  CORRADE_INTERNAL_ASSERT(shader);

  return shaderManager_.get<Magnum::GL::AbstractShaderProgram, T>(
      getShaderKey(type));
}

}  // namespace gfx
}  // namespace esp
#endif
