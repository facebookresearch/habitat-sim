// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_RENDERER_H_
#define ESP_GFX_RENDERER_H_

#include "esp/core/Esp.h"
#include "esp/gfx/CubeMap.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneGraph.h"
#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace sim {
class Simulator;
}
namespace gfx {
class Renderer {
 public:
  enum class Flag {
    /**
     * No textures for the meshes.
     * Textures take a lot of GPU memory but they are not needed unless we are
     * doing RGB rendering.
     * Note: Cannot set this flag when doing RGB rendering
     */
    NoTextures = 1 << 0,

    /**
     * When binding the render target to a depth or a sementic sensor,
     * setting this flag will give the render target the ability to visualize
     * the depth, or sementic info
     * see bindRenderTarget for more info.
     */
    VisualizeTexture = 1 << 1,

    /**
     * Use a background renderer to overlap rendering with physics simulation.
     */
    BackgroundRenderer = 1 << 2,

    /**
     * Leave the OpenGL context with the background renderer thread
     * after @ref waitDrawJobs.  Otherwise the context will be reacquired by the
     * main thread. Enabling this improves performance slightly but makes OpenGL
     * context management more challenging.
     *
     * This flag only has effect if habitat-sim was built with the
     * BackgroundRenderer and the BackgroundRenderer flag is true.
     */
    LeaveContextWithBackgroundRenderer = 1 << 3,

    /**
     * Enable HBAO visual effect that adds soft shadows to corners and crevices.
     */
    HorizonBasedAmbientOcclusion = 1 << 4,

  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  static void setupMagnumFeatures();

  /**
   * @brief Constructor
   */
  explicit Renderer(Flags flags = {});

  /**
   * @brief Constructor for when creating a background thread
   */
  explicit Renderer(WindowlessContext* context, Flags flags = {});

  /*
   * @brief draw the scene graph with the camera specified by user
   * @param[in] camera the render camera to render the scene
   * @param[in] sceneGraph the scene to render
   * @param[in] flags flags to control the rendering
   */
  void draw(RenderCamera& camera,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling});
  /**
   * @brief draw the active scene in current sim using the specified visual
   * sensor
   * @param[in] visualSensor, the visual sensor, from which the observation is
   * obtained
   * @param[in] sim, the simulator instance
   */
  void draw(sensor::VisualSensor& visualSensor, sim::Simulator& sim);

  /**
   * @brief visualize the observation of a non-rgb visual sensor, e.g., depth,
   * semantic
   */
  void visualize(sensor::VisualSensor& visualSensor,
                 float colorMapOffset,
                 float colorMapScale);
  /**
   * @brief visualize the observation of a non-rgb visual sensor, e.g., depth,
   * semantic, using already-specified offset and scale.
   */
  void visualize(sensor::VisualSensor& visualSensor);

#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER
  /**
   * @brief Enqueue a async draw job.
   *
   * Jobs are started by a call to @ref startDrawJobs.  Note that after calling
   * @ref startDrawJobs, you must call @ref waitSceneGraph before doing anything
   * that changes the scene graph.
   */
  void enqueueAsyncDrawJob(sensor::VisualSensor& visualSensor,
                           scene::SceneGraph& sceneGraph,
                           const Mn::MutableImageView2D& view,
                           RenderCamera::Flags flags = {
                               RenderCamera::Flag::FrustumCulling});

  /**
   * @brief Begins all the draw jobs enqueued by @ref enqueueAsyncDrawJob.
   *
   * This method implicitly transfers ownership of the OpenGL context and scene
   * graphs to the thread, use @ref waitSceneGraph and @ref acquireGlContext to
   * transfer ownership back
   */
  void startDrawJobs();
  /**
   * @brief Waits on all started
   */
  void waitDrawJobs();
#endif
  /**
   * @brief Sets the colormap for the @ref TextureVisualizerShader used for
   * Semantic Scene rendering. Note, these colors are only used for
   * visualization purposes.
   * @param colormap The colormap to use, where idxs correspond to per-vertex
   * semantic IDs.
   */
  void setSemanticVisualizerColormap(
      Cr::Containers::ArrayView<const Mn::Vector3ub> colorMap);

  /**
   * @brief Acquires ownership of the scene graph from the background render
   * thread. Will block if needed.
   *
   * The blocking waiting is guarded by an atomic, so if the main thread already
   * has ownership of the scene graph, this method is lock-free and very cheap.
   */
  void waitSceneGraph();
  /**
   * @brief Acquires the OpenGL context from the background render thread.  Will
   * block if needed.
   *
   * This method is lock-free if the main thread already has ownership of the
   * OpenGL context.
   */
  void acquireGlContext();

  /**
   * @brief Was the background rendering thread ever initialized. Initialization
   * is lazy and done the first time async render jobs are started.
   */
  bool wasBackgroundRendererInitialized() const;

  /**
   * @brief Binds a @ref RenderTarget to the sensor
   * @param[in] sensor the target sensor
   * @param[in] bindingFlags flags, such as to control the bindings
   */
  void bindRenderTarget(sensor::VisualSensor& sensor, Flags bindingFlags = {});

  /**
   * @brief apply gaussian filtering to source cubemap and store the result in
   * target cubemap
   * @param[in,out] target, the target cubemap
   * @param[in,out] helper, a helper cubemap with the same cube size, and
   * texture type (e.g., color, variance shadow map)
   * @param[in] type cubemap texture type, indicating which texture type the
   * filtering would apply to. It can ONLY be Color
   */
  void applyGaussianFiltering(CubeMap& target,
                              CubeMap& helper,
                              CubeMap::TextureType type);

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(Renderer)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_RENDERER_H_
