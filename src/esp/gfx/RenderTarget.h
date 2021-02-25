// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_RENDERTARGET_H_
#define ESP_GFX_RENDERTARGET_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/Magnum.h>

#include "esp/core/esp.h"

#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/Renderer.h"

namespace esp {
namespace gfx {

/**
 * Holds a framebuffer and encapsulates the logic of retrieving rendering
 * results of various types (RGB, Depth, ObjectID) from the framebuffer.
 *
 * Reads the rendering results into either CPU or GPU, if compiled with CUDA,
 * memory
 */
class RenderTarget {
 public:
  enum class Flag {
    /**
     * create rgba buffer
     * No need to set it for depth sensor, semantic sensor etc. as it makes
     * the rendering slower
     */
    RgbaBuffer = 1 << 0,
    /**
     * create objectId buffer
     * No need to set it for color sensor, depth sensor etc. as it makes the
     * rendering slower
     */
    ObjectIdBuffer = 1 << 1,
    /**
     * @brief use depth texture, it must be set for the depth sensor.
     * No need to set it for color sensor, objectId sensor etc. as it makes the
     * rendering slower
     */
    DepthTexture = 1 << 2,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  /**
   * @brief Constructor
   * @param size               The size of the underlying framebuffers in WxH
   * @param depthUnprojection  Depth unprojection parameters.  See @ref
   *                           calculateDepthUnprojection()
   * @param depthShader        A DepthShader used to unproject depth on the GPU.
   *                           Unprojects the depth on the CPU if nullptr.
   *                           Must be not nullptr to use @ref
   *                           readFrameDepthGPU()
   * @param flags              The flags of the renderer target
   */
  RenderTarget(const Magnum::Vector2i& size,
               const Magnum::Vector2& depthUnprojection,
               DepthShader* depthShader,
               Flags flags = {Flag::RgbaBuffer | Flag::ObjectIdBuffer |
                              Flag::DepthTexture});

  /**
   * @brief Constructor
   * @param size               The size of the underlying framebuffers in WxH
   * @param depthUnprojection  Depth unrpojection parameters.  See @ref
   *                           calculateDepthUnprojection()
   *
   * Equivalent to calling
   * @ref RenderTarget(size, depthUnprojection, nullptr, {})
   */
  RenderTarget(const Magnum::Vector2i& size,
               const Magnum::Vector2& depthUnprojection)
      : RenderTarget{size, depthUnprojection, nullptr, {}} {}

  ~RenderTarget() = default;

  /**
   * @brief Called before any draw calls that target this RenderTarget
   * Clears the framebuffer and binds it
   */
  void renderEnter();

  /**
   * @brief Prepare for another render pass (e.g., to bind the framebuffer).
   * Compared to @renderEnter, it will NOT clear the framebuffer.
   */
  void renderReEnter();

  /**
   * @brief Called after any draw calls that target this RenderTarget
   */
  void renderExit();

  /**
   * @brief The size of the framebuffer in WxH
   */
  Magnum::Vector2i framebufferSize() const;

  /**
   * @brief Retrieve the RGBA rendering results.
   *
   * @param[in, out] view Preallocated memory that will be populated with the
   * result.  The result will be read as the pixel format of this view.
   */
  void readFrameRgba(const Magnum::MutableImageView2D& view);

  /**
   * @brief Retrieve the depth rendering results.
   *
   * @param[in, out] view Preallocated memory that will be populated with the
   * result.  The PixelFormat of the image must only specify the R channel,
   * generally @ref Magnum::PixelFormat::R32F
   */
  void readFrameDepth(const Magnum::MutableImageView2D& view);

  /**
   * @brief Reads the ObjectID rendering results into the memory specified by
   * view
   *
   * @param[in, out] view Preallocated memory that will be populated with the
   * result.  The PixelFormat of the image must only specify the R channel and
   * be a format which a uint16_t can be interpreted as, generally @ref
   * Magnum::PixelFormat::R32UI, @ref Magnum::PixelFormat::R32I, or @ref
   * Magnum::PixelFormat::R16UI
   */
  void readFrameObjectId(const Magnum::MutableImageView2D& view);

  /**
   * @brief Blits the rgba buffer from internal FBO to default frame buffer
   * which in case of EmscriptenApplication will be a canvas element.
   */
  void blitRgbaToDefault();

  // @brief Delete copy Constructor
  RenderTarget(const RenderTarget&) = delete;
  // @brief Delete copy operator
  RenderTarget& operator=(const RenderTarget&) = delete;

#ifdef ESP_BUILD_WITH_CUDA
  /**
   * @brief Reads the RGBA rendering result directly into CUDA memory. The
   * caller is responsible for allocating memory and ensuring that the OpenGL
   * context and the devPtr are on the same CUDA device.
   *
   * @param[in, out] devPtr CUDA memory pointer that points to a contiguous
   * memory region of at least W*H*sizeof(uint8_t) bytes.
   */
  void readFrameRgbaGPU(uint8_t* devPtr);

  /**
   * @brief Reads the depth rendering result directly into CUDA memory.  See
   * @ref readFrameRgbaGPU()
   *
   * Requires the rendering target to have a valid DepthShader
   *
   * @param[in, out] devPtr CUDA memory pointer that points to a contiguous
   * memory region of at least W*H*sizeof(float) bytes.
   */
  void readFrameDepthGPU(float* devPtr);

  /**
   * @brief Reads the ObjectID rendering result directly into CUDA memory.  See
   * @ref readFrameRgbaGPU()
   *
   * @param[in, out] devPtr CUDA memory pointer that points to a contiguous
   * memory region of at least W*H*sizeof(int32_t) bytes.
   */
  void readFrameObjectIdGPU(int32_t* devPtr);
#endif

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(RenderTarget)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_RENDERTARGET_H_
