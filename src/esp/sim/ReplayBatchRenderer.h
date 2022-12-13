// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_REPLAYBATCHRENDERER_H_
#define ESP_SIM_REPLAYBATCHRENDERER_H_

#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx/replay/Player.h"
#include "esp/gfx_batch/RendererStandalone.h"
#include "esp/scene/SceneManager.h"

namespace esp {
namespace assets {
class ResourceManager;
}  // namespace assets
namespace gfx {
class Renderer;
}
}  // namespace esp

namespace esp {
namespace sim {

class ReplayRendererConfiguration {
 public:
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

  std::vector<sensor::SensorSpec::ptr> sensorSpecifications;

  ESP_SMART_POINTERS(ReplayRendererConfiguration)
};

class AbstractReplayRenderer {
 public:
  static Magnum::Vector2i environmentGridSize(int environmentCount);

  virtual ~AbstractReplayRenderer();

  void preloadFile(Corrade::Containers::StringView filename);

  unsigned environmentCount() const;

  // Assumes there's just one sensor per env
  Magnum::Vector2i sensorSize(unsigned envIndex);

  void clearEnviroment(unsigned envIndex);

  void setEnvironmentKeyframe(unsigned envIndex,
                              const std::string& serKeyframe);

  void setEnvironmentKeyframeUnwrapped(
      unsigned envIndex,
      Corrade::Containers::StringView serKeyframe);

  void setSensorTransform(unsigned envIndex,
                          const std::string& sensorName,
                          const Mn::Matrix4& transform);

  // You must have done Recorder::addUserTransformToKeyframe(prefix +
  // sensorName, ...) for every sensor in
  // ReplayRendererConfiguration::sensorSpecifications, for the specified
  // environment's keyframe. See also setEnvironmentKeyframe.
  void setSensorTransformsFromKeyframe(unsigned envIndex,
                                       const std::string& prefix);

  // Renders and waits for the render to finish
  void render(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                  imageViews);

  // Assumes the framebuffer color & depth is cleared
  void render(Magnum::GL::AbstractFramebuffer& framebuffer);

 private:
  /* Implementation of all public API is in the private do*() functions,
     similarly to how e.g. Magnum plugin interfaces work. The public API does
     all necessary checking (such as ensuring envIndex is in bounds) in order
     to allow the implementations be only about what's actually
     backend-specific, with no unnecessary duplicated code. */

  /* Default implementation does nothing */
  virtual void doPreloadFile(Corrade::Containers::StringView filename);

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
                                    const Mn::Matrix4& transform) = 0;

  /* envIndex is guaranteed to be in bounds */
  virtual void doSetSensorTransformsFromKeyframe(unsigned envIndex,
                                                 const std::string& prefix) = 0;

  /* imageViews.size() is guaranteed to be same as doEnvironmentCount() */
  virtual void doRender(
      Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
          imageViews) = 0;

  virtual void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) = 0;

  ESP_SMART_POINTERS(AbstractReplayRenderer)
};

class ReplayRenderer : public AbstractReplayRenderer {
 public:
  struct EnvironmentRecord {
    explicit EnvironmentRecord(
        std::shared_ptr<gfx::replay::AbstractSceneGraphPlayerImplementation>
            playerImplementation)
        : playerImplementation_{std::move(playerImplementation)},
          player_{playerImplementation_} {}
    std::shared_ptr<gfx::replay::AbstractSceneGraphPlayerImplementation>
        playerImplementation_;
    esp::gfx::replay::Player player_;
    int sceneID_ = ID_UNDEFINED;
    int semanticSceneID_ = ID_UNDEFINED;
    esp::scene::SceneNode* sensorParentNode_ = nullptr;
    std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>
        sensorMap_;
  };

  explicit ReplayRenderer(const ReplayRendererConfiguration& cfg);

  ~ReplayRenderer() override;

  // TODO those are used by bindings at the moment, but should eventually
  //  become private as well
  std::shared_ptr<gfx::Renderer> getRenderer() { return renderer_; }

  esp::scene::SceneGraph& getSceneGraph(unsigned envIndex);
  esp::scene::SceneGraph& getSemanticSceneGraph(unsigned envIndex);

  esp::scene::SceneNode* getEnvironmentSensorParentNode(
      unsigned envIndex) const;
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getEnvironmentSensors(unsigned envIndex);

 private:
  unsigned doEnvironmentCount() const override;

  Magnum::Vector2i doSensorSize(unsigned envIndex) override;

  esp::gfx::replay::Player& doPlayerFor(unsigned envIndex) override;

  void doSetSensorTransform(unsigned envIndex,
                            const std::string& sensorName,
                            const Mn::Matrix4& transform) override;

  void doSetSensorTransformsFromKeyframe(unsigned envIndex,
                                         const std::string& prefix) override;

  void doRender(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                    imageViews) override;

  void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) override;

  gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
      unsigned envIndex,
      const assets::AssetInfo& assetInfo,
      const assets::RenderAssetInstanceCreationInfo& creation);

  std::vector<EnvironmentRecord> envs_;

  std::unique_ptr<assets::ResourceManager> resourceManager_;

  scene::SceneManager::uptr sceneManager_ = nullptr;

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;

  ReplayRendererConfiguration config_;

  ESP_SMART_POINTERS(ReplayRenderer)
};

class ReplayBatchRenderer : public AbstractReplayRenderer {
 public:
  explicit ReplayBatchRenderer(const ReplayRendererConfiguration& cfg);

  ~ReplayBatchRenderer() override;

 private:
  void doPreloadFile(Corrade::Containers::StringView filename) override;

  unsigned doEnvironmentCount() const override;

  Magnum::Vector2i doSensorSize(unsigned envIndex) override;

  esp::gfx::replay::Player& doPlayerFor(unsigned envIndex) override;

  void doSetSensorTransform(unsigned envIndex,
                            const std::string& sensorName,
                            const Mn::Matrix4& transform) override;

  void doSetSensorTransformsFromKeyframe(unsigned envIndex,
                                         const std::string& prefix) override;

  void doRender(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                    imageViews) override;

  void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) override;

  /* If standalone_ is true, renderer_ contains a RendererStandalone. Has to be
     before the EnvironmentRecord array because Player calls
     gfx_batch::Renderer::clear() on destruction. */
  bool standalone_;
  Corrade::Containers::Pointer<esp::gfx_batch::Renderer> renderer_;

  // TODO pimpl all this?
  struct EnvironmentRecord {
    std::shared_ptr<gfx::replay::AbstractPlayerImplementation>
        playerImplementation_;
    gfx::replay::Player player_{playerImplementation_};
  };
  Corrade::Containers::Array<EnvironmentRecord> envs_;

  Corrade::Containers::String theOnlySensorName_;
  Mn::Matrix4 theOnlySensorProjection_;

  ESP_SMART_POINTERS(ReplayBatchRenderer)
};

}  // namespace sim
}  // namespace esp

#endif
