// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#ifndef ESP_GFX_CUBEMAP_H_
#define ESP_GFX_CUBEMAP_H_

#include <map>

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Containers/StaticArray.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/ResourceManager.h>
#include <Magnum/Shaders/GenericGL.h>
#include <Magnum/Trade/AbstractImporter.h>
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
// NOLINTNEXTLINE(bugprone-reserved-identifier)
class CubeMap {
 public:
  enum class TextureType : uint8_t {
    /**
     * rgba texture with 8 bits per channel
     */
    Color,
    /**
     * HDR depth texture
     */
    Depth,
    /**
     * object id (uint) texture
     */
    ObjectId,

    // TODO: HDR color

    Count,
  };

  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,

    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     */
    ObjectIdOutput = Magnum::Shaders::GenericGL3D::ObjectIdOutput,

  };

  enum class Flag : Magnum::UnsignedShort {
    /**
     *  create color cubemap
     */
    ColorTexture = 1 << 0,
    /**
     * create depth cubemap
     */
    DepthTexture = 1 << 1,
    /**
     * create ObjectId cubemap
     */
    ObjectIdTexture = 1 << 2,
    /**
     * Build mipmap for cubemap color texture
     * By default, NO mipmap will be built, only 1 level (mip 0)
     * By turning on this option, it will AUTOMATICALLY build the mipmap for the
     * *color* texture if any (for example, after rendering to texture (mip 0),
     * or loading the texture to mip 0.)
     */
    AutoBuildMipmap = 1 << 3,
    /**
     * Reserve the space for the mipmaps of the cubemap color texture. System
     * will NOT automatically build the mipmaps for the user.
     * By default, NO mipmap will be built, only 1 level (mip 0).
     * By turning on this option, it will allocates the space for the mipmaps,
     * but will not fillin the contents. This is useful e.g., in PBR prefiltered
     * environment map computation.
     */
    ManuallyBuildMipmap = 1 << 4,

  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief, Constructor
   * @param imageSize the size of the cubemap texture (each face is size x size)
   */
  explicit CubeMap(int imageSize, Flags flags = Flags{Flag::ColorTexture});

  /**
   * @brief, reset the image size
   * @return true, if image size has been changed, and reset has happened,
   * otherwise false
   */
  bool reset(int imageSize);

  /**
   * @brief Get the cubemap texture based on the texture type
   * @return Reference to the cubemap texture
   */
  Magnum::GL::CubeMapTexture& getTexture(TextureType type);

  /** @brief get cube map size */
  int getCubeMapSize() const { return imageSize_; }

#ifndef MAGNUM_TARGET_WEBGL
  /**
   * ```
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   * ```
   * NOTE: +Y is top
   * @brief save the cubemap texture based on the texture type
   * @param type texture type
   * @param imageFilePrefix the filename prefix
   * The 6 image files then would be:
   * {imageFilePrefix}.{texType}.+X.png
   * {imageFilePrefix}.{texType}.-X.png
   * {imageFilePrefix}.{texType}.+Y.png
   * {imageFilePrefix}.{texType}.-Y.png
   * {imageFilePrefix}.{texType}.+Z.png
   * {imageFilePrefix}.{texType}.-Z.png
   * @return true, if success, otherwise false
   *
   * NOTE:
   * In this version, it will lose precision when depth texture is saved to the
   * disk. This is because the pixel format for depth texture is R32F with 1 bit
   * of sign, 23 bits of mantissa and 8 bits of exponent while the radiance HDR
   * is 8 bits of mantissa and 8 bits of exponent. So when it is saved, only 8
   * bits is saved to the R channel (GB channels just repeat the R channel
   * twice).
   */
  // TODO: save HDR textures in EXR format
  bool saveTexture(TextureType type,
                   const std::string& imageFilePrefix,
                   unsigned int mipLevel = 0);
#endif

  /**
   * ```
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   * ```
   * NOTE: +Y is top
   * @brief load cubemap texture from external images
   * @param type can be "rgba", "depth", or "objectId" (TODO)
   * @param imageFilePrefix the prefix of the image filename
   * @param imageFileExtension the image filename extension (such as "png",
   * "jpg")
   * @return true if succeeded, otherwise false
   * The 6 image files then would be:
   * {imageFilePrefix}.{texType}.+X.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-X.{imageFileExtension}
   * {imageFilePrefix}.{texType}.+Y.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-Y.{imageFileExtension}
   * {imageFilePrefix}.{texType}.+Z.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-Z.{imageFileExtension}
   */
  void loadTexture(TextureType type,
                   const std::string& imageFilePrefix,
                   const std::string& imageFileExtension);

  /**
   * @brief Render to cubemap texture using the camera
   * @param camera a cubemap camera
   * NOTE: It will NOT automatically generate the mipmap for the user
   */
  void renderToTexture(CubeMapCamera& camera,
                       scene::SceneGraph& sceneGraph,
                       const char* drawableGroupName = "",
                       RenderCamera::Flags flags = {
                           RenderCamera::Flag::FrustumCulling |
                           RenderCamera::Flag::ClearColor |
                           RenderCamera::Flag::ClearDepth});

  /**
   * @brief copy the texture from a specified cube face to a given texture
   * (GPU->GPU)
   * NOTE: can only call this function for the textures attaching to the color
   * attachments
   * @param[in] cubeSideIndex, the index of the cube side, can be 0, 1, ..., 5
   * @param[in] type, the texture type
   * NOTE: the type CANNOT be the depth texture
   * @param[in, out] texture, the 2D target texture
   * @param[in] mipLevel, the mipmap level
   */
  void copySubImage(unsigned int cubeSideIndex,
                    TextureType type,
                    Magnum::GL::Texture2D& texture,
                    unsigned int mipLevel = 0);
  /**
   * @brief Prepare to draw to the texture. It will bind the framebuffer, clear
   * color, depth etc.
   * @param[in] cubeSideIndex, the index of the cube side, can be 0,
   * 1, 2, 3, 4, or 5
   * @param[in] flags, the flags to control the rendering
   * @param[in] mipLevel, the mip level of the texture. default value is 0. This
   * is an advanced feature. In most cases user do not have to set it.
   * NOTE:
   * If a non-zero miplevel is specified, it requires:
   * - The cubemap is color texture; (does not make sense to talk about mip
   * level for depth or object ids)
   * - Flag::ManuallyBuildMipmap is set;
   * - value is smaller than the max mip level, which is log2(mip0_size) + 1;
   */
  void prepareToDraw(
      unsigned int cubeSideIndex,
      RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling |
                                   RenderCamera::Flag::ClearColor |
                                   RenderCamera::Flag::ClearDepth},
      unsigned int mipLevel = 0);

  /**
   * @brief Get the mipmap levels
   * NOTE: returns 1 if this is non-colored cubemap, or it is colored cubemap
   * but without mipmap enabled.
   */
  unsigned int getMipmapLevels() const;

  /**
   * @brief generate mipmap
   */
  void generateMipmap(TextureType type);

  /** @brief get flags */
  Flags getFlags() { return flags_; }

 private:
  Flags flags_;
  int imageSize_ = 0;
  unsigned int mipmapLevels_ = 1;

  Magnum::GL::CubeMapTexture textures_[uint8_t(TextureType::Count)];

  Magnum::GL::CubeMapTexture& texture(TextureType type);

  /**
   * @brief Recreate textures
   */
  void recreateTexture();

  // framebuffers (one for every cube side)
  Corrade::Containers::StaticArray<6, Magnum::GL::Framebuffer> frameBuffer_{
      Corrade::DirectInit, Magnum::NoCreate};

  // in case there is no need to output depth texture, we need a depth buffer
  Corrade::Containers::StaticArray<6, Magnum::GL::Renderbuffer>
      optionalDepthBuffer_{Corrade::DirectInit, Magnum::NoCreate};

  /**
   * @brief recreate the frame buffer
   * @param cubeSideIndex the index of the cube side, can be 0,
   * 1, 2, 3, 4, or 5
   * @param[in] framebufferSize, the size of the framebuffer
   */
  void recreateFramebuffer(unsigned int cubeSideIndex, int framebufferSize);

  /**
   * @brief attach renderbuffers (color etc.) as logical buffers of the
   * framebuffer object
   * @param cubeSideIndex the index of the cube side, can be 0,
   * 1, 2, 3, 4, or 5
   * @param[in] mipLevel, the level of the mipmap. See more details at @ref
   * prepareToDraw
   */
  void attachFramebufferRenderbuffer(unsigned int cubeSideIndex,
                                     unsigned int mipLevel = 0);

  /**
   * @brief Map shader output to attachments.
   * @param cubeSideIndex the index of the cube side, can be 0,
   * 1, 2, 3, 4, or 5
   */
  void mapForDraw(unsigned int cubeSideIndex);

  /**
   * @brief bind framebuffer
   * @param[in] cubeSideIndex, the index of the cube side, can be 0,
   * 1, 2, 3, 4, or 5
   */
  void bindFramebuffer(unsigned int cubeSideIndex);
};

CORRADE_ENUMSET_OPERATORS(CubeMap::Flags)

}  // namespace gfx
}  // namespace esp
#endif
