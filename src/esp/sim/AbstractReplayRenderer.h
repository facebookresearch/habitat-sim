// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_ABSTRACTREPLAYRENDERER_H_
#define ESP_SIM_ABSTRACTREPLAYRENDERER_H_

#include "esp/geo/Geo.h"
#include "esp/gfx/DebugLineRender.h"

namespace esp {

namespace gfx {
namespace replay {
class Player;
}
}  // namespace gfx

namespace sensor {
class SensorSpec;
}

namespace sim {

class ReplayRendererConfiguration {
 public:
  /* De-inlining all constructors and destructors in order to be able to have
     SensorSpec just forward-declared */
  ReplayRendererConfiguration();
  ReplayRendererConfiguration(const ReplayRendererConfiguration&);
  ReplayRendererConfiguration(ReplayRendererConfiguration&&) noexcept;
  ReplayRendererConfiguration& operator=(const ReplayRendererConfiguration&);
  ReplayRendererConfiguration& operator=(
      ReplayRendererConfiguration&&) noexcept;
  ~ReplayRendererConfiguration();

  int numEnvironments = 1;
  //! The system GPU device to use for rendering.
  int gpuDeviceId = 0;
  /**
   * @brief Have the renderer create its own GPU context
   *
   * Set to @cpp false @ce in scenarios where the renderer is meant to draw
   * into a GUI application window.
   */
  bool standalone = true;
  // TODO document that those are obsolete options affecting the classic
  //  renderer only
  bool forceSeparateSemanticSceneGraph = false;
  /**
   * @brief Leave the context with the background thread after finishing draw
   * jobs. This will improve performance as transferring the OpenGL context back
   * and forth takes time but will require the user to manually transfer the
   * context back to the main thread before adding or removing objects.
   */
  bool leaveContextWithBackgroundRenderer = false;

  std::vector<std::shared_ptr<sensor::SensorSpec>> sensorSpecifications;

  ESP_SMART_POINTERS(ReplayRendererConfiguration)
};

class AbstractReplayRenderer {
 public:
  static Magnum::Vector2i environmentGridSize(int environmentCount);

  virtual ~AbstractReplayRenderer();

  void close();

  void preloadFile(Corrade::Containers::StringView filename);

  unsigned environmentCount() const;

  // Assumes there's just one sensor per env
  Magnum::Vector2i sensorSize(unsigned envIndex);

  void clearEnvironment(unsigned envIndex);

  void setEnvironmentKeyframe(unsigned envIndex,
                              const std::string& serKeyframe);

  void setEnvironmentKeyframeUnwrapped(
      unsigned envIndex,
      Corrade::Containers::StringView serKeyframe);

  void setSensorTransform(unsigned envIndex,
                          const std::string& sensorName,
                          const Magnum::Matrix4& transform);

  // You must have done Recorder::addUserTransformToKeyframe(prefix +
  // sensorName, ...) for every sensor in
  // ReplayRendererConfiguration::sensorSpecifications, for the specified
  // environment's keyframe. See also setEnvironmentKeyframe.
  void setSensorTransformsFromKeyframe(unsigned envIndex,
                                       const std::string& prefix);

  // Renders into the specified CPU-resident image view arrays (one image per
  // environment). Waits for the render to finish.
  void render(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                  colorImageViews,
              Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                  depthImageViews);

  // Assumes the framebuffer color & depth is cleared
  void render(Magnum::GL::AbstractFramebuffer& framebuffer);

  // Retrieve the color buffer as a CUDA device pointer. */
  virtual const void* getCudaColorBufferDevicePointer();

  // Retrieve the depth buffer as a CUDA device pointer. */
  virtual const void* getCudaDepthBufferDevicePointer();

  std::shared_ptr<esp::gfx::DebugLineRender> getDebugLineRender(
      unsigned envIndex);

  /**
   * @brief Unproject a 2D viewport point to a 3D ray with origin at camera
   * position. Ray direction is normalized.
   *
   * @param envIndex
   * @param viewportPosition The 2D point on the viewport to unproject
   * ([0,width], [0,height]).
   */
  esp::geo::Ray unproject(unsigned envIndex,
                          const Magnum::Vector2i& viewportPosition);

 protected:
  void checkEnvIndex(unsigned envIndex);

  std::shared_ptr<esp::gfx::DebugLineRender> debugLineRender_;

 private:
  /* Implementation of all public API is in the private do*() functions,
     similarly to how e.g. Magnum plugin interfaces work. The public API does
     all necessary checking (such as ensuring envIndex is in bounds) in order
     to allow the implementations be only about what's actually
     backend-specific, with no unnecessary duplicated code. */

  /* Default implementation does nothing */
  virtual void doPreloadFile(Corrade::Containers::StringView filename);

  virtual void doClose() = 0;

  virtual unsigned doEnvironmentCount() const = 0;

  /* Retrieves a player instance for given environment. Used by
     setSensorTransformsFromKeyframe(). The envIndex is guaranteed to be in
     bounds. */
  virtual esp::gfx::replay::Player& doPlayerFor(unsigned envIndex) = 0;

  /* envIndex is guaranteed to be in bounds */
  virtual Magnum::Vector2i doSensorSize(unsigned envIndex) = 0;

  /* envIndex is guaranteed to be in bounds */
  virtual void doSetSensorTransform(unsigned envIndex,
                                    const std::string& sensorName,
                                    const Magnum::Matrix4& transform) = 0;

  /* envIndex is guaranteed to be in bounds */
  virtual void doSetSensorTransformsFromKeyframe(unsigned envIndex,
                                                 const std::string& prefix) = 0;

  /* imageViews.size() is guaranteed to be same as doEnvironmentCount() */
  virtual void doRender(
      Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
          colorImageViews,
      Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
          depthImageViews) = 0;

  virtual void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) = 0;

  virtual esp::geo::Ray doUnproject(unsigned envIndex,
                                    const Mn::Vector2i& viewportPosition) = 0;

  ESP_SMART_POINTERS(AbstractReplayRenderer)
};

}  // namespace sim
}  // namespace esp

#endif
