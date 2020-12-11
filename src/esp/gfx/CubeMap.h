// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#ifndef ESP_GFX_CUBEMAP_H_
#define ESP_GFX_CUBEMAP_H_

#include <map>

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/Magnum.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/ResourceManager.h>
#include <Magnum/Trade/AbstractImporter.h>
#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
class CubeMap {
 public:
  static void enableSeamlessCubeMapTexture();
  enum class TextureType : int8_t {
    Color = 0,
    Depth = 1,
    // TODO: ObjectId
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
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief, Constructor
   * @param size, the size of the cubemap texture (each face is size x size)
   */
  CubeMap(int imageSize, Flags flags = Flags{Flag::ColorTexture});

  /**
   * @brief, reset the image size
   */
  void reset(int imageSize);

  /**
   * @brief Get the cubemap texture based on the texture type
   * @return Reference to the cubemap texture
   */
  Magnum::GL::CubeMapTexture& getTexture(TextureType type);
  /**
   * @brief save the cubemap texture based on the texture type
   * @param type, texture type
   * @param imageFilePrefix, the filename prefix
   * The 6 image files then would be:
   * {imageFilePrefix}.{texType}.+X.png
   * {imageFilePrefix}.{texType}.-X.png
   * {imageFilePrefix}.{texType}.+Y.png
   * {imageFilePrefix}.{texType}.-Y.png
   * {imageFilePrefix}.{texType}.+Z.png
   * {imageFilePrefix}.{texType}.-Z.png
   * @return true, if success, otherwise false
   * NOTE: +Y is top
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   */
  bool saveTexture(TextureType type, const std::string& imageFilePrefix);

  /**
   * @brief load cubemap texture from external images
   * @param imageFilePrefix, the prefix of the image filename
   * @param imageFileExtension, the image filename extension (such as "png",
   * "jpg")
   * The 6 image files then would be:
   * {imageFilePrefix}.{texType}.+X.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-X.{imageFileExtension}
   * {imageFilePrefix}.{texType}.+Y.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-Y.{imageFileExtension}
   * {imageFilePrefix}.{texType}.+Z.{imageFileExtension}
   * {imageFilePrefix}.{texType}.-Z.{imageFileExtension}
   *
   * texType can be "rgba", "depth", "objectId"
   * NOTE: +Y is top
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   */
  void loadTexture(TextureType type,
                   const std::string& imageFilePrefix,
                   const std::string& imageFileExtension);

  /**
   * @brief Render to cubemap texture using the camera
   * @param camera, a cubemap camera
   * NOTE: It will NOT automatically generate the mipmap for the user
   */
  void renderToTexture(CubeMapCamera& camera,
                       scene::SceneGraph& sceneGraph,
                       RenderCamera::Flags flags);

 protected:
  Flags flags_;
  int imageSize_ = 0;
  std::map<TextureType, std::unique_ptr<Magnum::GL::CubeMapTexture>> textures_;

  Magnum::ResourceManager<Magnum::Trade::AbstractImporter> imageImporterManger_;

  /**
   * @brief Recreate textures
   */
  void recreateTexture();

  // framebuffer for drawable selection
  Magnum::GL::Framebuffer frameBuffer_{Magnum::NoCreate};

  // in case there is no need to output depth texture, we need a depth buffer
  Magnum::GL::Renderbuffer optionalDepthBuffer_;

  /**
   * @brief Recreate frame buffer
   */
  void recreateFramebuffer();

  /**
   * @brief Prepare to draw to the texture
   */
  void prepareToDraw(int cubeSideIndex);

  /**
   * @brief Map shader output to attachments.
   */
  void mapForDraw();

  /**
   * @brief check if the class instance is created with corresponding texture
   * enabled
   */
  void textureTypeSanityCheck(TextureType type,
                              const std::string& functionNameStr);

  /**
   * @brief convert cube face index to Magnum::GL::CubeMapCoordinate
   */
  Magnum::GL::CubeMapCoordinate convertFaceIndexToCubeMapCoordinate(
      int faceIndex);

  /**
   * @brief get texture type string for texture filename
   */
  std::string getTextureTypeFilenameString(TextureType type);
  /**
   * @brief get the pixel format based on texture type (color, depth objectId
   * etc.)
   */
  Magnum::PixelFormat getPixelFormat(TextureType type);
};

CORRADE_ENUMSET_OPERATORS(CubeMap::Flags)

}  // namespace gfx
}  // namespace esp
#endif
