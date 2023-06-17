// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BATCH_RENDERER_STANDALONE_H_
#define ESP_GFX_BATCH_RENDERER_STANDALONE_H_

#include "Renderer.h"

// TODO: ideally this would go directly to esp/, so we don't depend on core
#include "esp/core/configure.h"

namespace esp {
namespace gfx_batch {

/**
@brief Global standalone batch renderer flag

@see @ref RendererStandaloneFlags, @ref RendererStandaloneConfiguration::setFlags()
*/
enum class RendererStandaloneFlag {
  /**
   * Suppress Magnum startup log.
   *
   * Useful for testing scenarios where the renderer instance is created
   * several times in a row and the output would cause too much noise in the
   * output.
   *
   * @m_class{m-note m-warning}
   *
   * @par
   *    **Not recommended** to be enabled in end-user applications, as the log
   *    contains vital information for debugging platform-specific issues.
   */
  QuietLog = 1 << 0
};

/**
@brief Global standalone batch renderer flags

@see @ref RendererStandaloneConfiguration::setFlags()
*/
typedef Corrade::Containers::EnumSet<RendererStandaloneFlag>
    RendererStandaloneFlags;

CORRADE_ENUMSET_OPERATORS(RendererStandaloneFlags)

class RendererStandalone;

/**
@brief Global standalone renderer configuration

Passed to @ref RendererStandalone::RendererStandalone(const RendererConfiguration&, const RendererStandaloneConfiguration&).
@see @ref RendererStandalone, @ref RendererConfiguration
*/
struct RendererStandaloneConfiguration {
  explicit RendererStandaloneConfiguration();
  ~RendererStandaloneConfiguration();

  /**
   * @brief Set CUDA device ID
   *
   * By default no ID is set, meaning any GPU --- including a non-NVidia GPU
   * --- is selected.
   * @see @ref Magnum::Platform::WindowlessEglContext::Configuration::setCudaDevice()
   */
  RendererStandaloneConfiguration& setCudaDevice(Magnum::UnsignedInt id);

  /**
   * @brief Set renderer flags
   *
   * By default no flags are set.
   * @see @ref RendererStandalone::standaloneFlags()
   */
  RendererStandaloneConfiguration& setFlags(RendererStandaloneFlags flags);

 private:
  friend RendererStandalone;
  struct State;
  Corrade::Containers::Pointer<State> state;
};

/**
@brief Standalone batch renderer

Wraps @ref Renderer with a corresponding GPU context and a framebuffer for a
standalone use (i.e., in cases where it's not needed to share the framebuffer
with a GUI application).

@section gfx_batch-RendererStandalone-usage Usage

The renderer gets constructed using a @ref RendererStandaloneConfiguration with
desired CUDA device specified (if needed) and a @ref RendererConfiguration with
desired tile size and count. From there onwards, the usage is the same as with
the base class alone, @ref gfx_batch-Renderer-usage "see its documentation for more information".

After @ref draw(), in order to retrieve the framebuffer data, you have two
options:

<ul>
<li>
  Pass it over to CUDA using @ref colorCudaBufferDevicePointer() and
  @ref depthCudaBufferDevicePointer(). These functions internally make a copy
  of the framebuffer to an internal linearized CUDA buffer and return its
  device pointer. The returned data are in a format reported by
  @ref colorFramebufferFormat() and @ref depthFramebufferFormat().
  @attention
    To avoid issues on multi-GPU systems (or laptops with an iGPU), you should
    explicitly pass a CUDA device ID to
    @ref RendererStandaloneConfiguration::setCudaDevice(), and then use the
    same CUDA device ID for the code you pass the data to.
</li>
<li>
  Or download it via @ref colorImage() and @ref depthImage(). Calling these
  functions causes the CPU to wait until the GPU finishes rendering, so it's
  intended mainly for testing and debugging purposes.
</li>
</ul>
*/
class RendererStandalone : public Renderer {
 public:
  /**
   * @brief Constructor
   * @param configuration           Renderer configuration
   * @param standaloneConfiguration Standalone renderer configuration
   */
  explicit RendererStandalone(
      const RendererConfiguration& configuration,
      const RendererStandaloneConfiguration& standaloneConfiguration);

  ~RendererStandalone() override;

  /**
   * @brief Global standalone renderer flags
   *
   * By default, no flags are set.
   * @see @ref RendererStandaloneConfiguration::setFlags(), @ref flags()
   */
  RendererStandaloneFlags standaloneFlags() const;

  /**
   * @brief Color framebuffer format
   *
   * Format in which @ref colorImage() and @ref colorCudaBufferDevicePointer()
   * is returned. At the moment @ref Magnum::PixelFormat::RGBA8Unorm.
   * Framebuffer size is @ref tileSize() multiplied by @ref tileCount().
   * @see @ref Magnum::pixelFormatSize(), @ref Magnum::pixelFormatChannelCount()
   */
  Magnum::PixelFormat colorFramebufferFormat() const;

  /**
   * @brief Depth framebuffer format
   *
   * Format in which @ref depthImage() and @ref colorCudaBufferDevicePointer()
   * is returned. At the moment @ref Magnum::PixelFormat::Depth32F. Framebuffer
   * size is @ref tileSize() multiplied by @ref tileCount().
   * @see @ref Magnum::pixelFormatSize()
   */
  Magnum::PixelFormat depthFramebufferFormat() const;

  /**
   * @brief Draw all scenes
   *
   * Compared to @ref Renderer::draw(Magnum::GL::AbstractFramebuffer&), the
   * scene is drawn to an internal framebuffer. Use @ref colorImage() /
   * @ref depthImage() or @ref colorCudaBufferDevicePointer() /
   * @ref depthCudaBufferDevicePointer() to retrieve the output.
   */
  void draw();

  /**
   * @brief Retrieve the rendered color output
   *
   * Stalls the CPU until the GPU finishes the last @ref draw() and then
   * returns an image in @ref colorFramebufferFormat() and with size being
   * @ref tileSize() multiplied by @ref tileCount().
   */
  Magnum::Image2D colorImage();

  /**
   * @brief Retrieve the rendered color output into a pre-allocated location
   *
   * Expects that @p rectangle is contained in a size defined
   * by @ref tileSize() multiplied by @ref tileCount(), that @p image
   * size corresponds to @p rectangle size and that its format is compatible
   * with @ref colorFramebufferFormat().
   */
  void colorImageInto(const Magnum::Range2Di& rectangle,
                      const Magnum::MutableImageView2D& image);

  /**
   * @brief Retrieve the raw rendered depth output.
   *
   * This returns the depth buffer as-is. To unproject, use @ref unprojectDepth().
   *
   * Stalls the CPU until the GPU finishes the last @ref draw() and then
   * returns an image in @ref depthFramebufferFormat() and with size being
   * @ref tileSize() multiplied by @ref tileCount().
   */
  Magnum::Image2D depthImage();

  /**
   * @brief Retrieve the raw rendered depth output into a pre-allocated
   * location.
   *
   * This returns the depth buffer as-is. To unproject, use @ref unprojectDepth().
   *
   * Expects that @p rectangle is contained in a size defined
   * by @ref tileSize() multiplied by @ref tileCount(), that @p image
   * size corresponds to @p rectangle size and that its format is compatible
   * with @ref depthFramebufferFormat().
   */
  void depthImageInto(const Magnum::Range2Di& rectangle,
                      const Magnum::MutableImageView2D& image);

#if defined(ESP_BUILD_WITH_CUDA) || defined(DOXYGEN_GENERATING_OUTPUT)
  /**
   * @brief Retrieve the rendered color output as a CUDA device pointer
   *
   * Copies the internal framebuffer into a linearized and tightly-packed CUDA
   * buffer of @ref colorFramebufferFormat() and with size given by the
   * @ref Magnum::Math::Vector::product() "product()" of @ref tileSize()
   * and @ref tileCount(), and returns its device pointer.
   */
  const void* colorCudaBufferDevicePointer();

  /**
   * @brief Retrieve the rendered depth output as a CUDA device pointer
   *
   * Copies the internal framebuffer into a linearized and tightly-packed CUDA
   * buffer of @ref depthFramebufferFormat() and with size given by the
   * @ref Magnum::Math::Vector::product() "product()" of @ref tileSize()
   * and @ref tileCount(), and returns its device pointer.
   */
  const void* depthCudaBufferDevicePointer();
#endif

 private:
  struct State;
  Corrade::Containers::Pointer<State> state_;
};

}  // namespace gfx_batch
}  // namespace esp

#endif
