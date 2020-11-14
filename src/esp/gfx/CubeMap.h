// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#ifndef ESP_GFX_CUBEMAP_H_
#define ESP_GFX_CUBEMAP_H_

#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/Magnum.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "esp/gfx/CubeMapCamera.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
class CubeMap {
 public:
  /**
   * @brief, Constructor
   * @param size, the size of the cubemap texture (each face is size x size)
   */
  CubeMap(int imageSize);

  /**
   * @brief, reset the image size
   */
  void reset(int imageSize);

  /**
   * @brief Get the cubemap color texture
   * @return Reference to the cubemap texture
   */
  Magnum::GL::CubeMapTexture& getColorTexture() { return *colorTexture_; }
  Magnum::GL::CubeMapTexture& getDepthTexture() { return *depthTexture_; }

  /**
   * @brief Render to cubemap texture using the camera
   * @param camera, a cubemap camera
   */
  void renderToTexture(CubeMapCamera& camera,
                       scene::SceneGraph& sceneGraph,
                       RenderCamera::Flags flags);
  /**
   * @brief load cubemap texture from external images
   * @param importer, image importer
   * @param imageFilePrefix, the prefix of the image filename
   * @param imageFileExtension, the image filename extension (such as .png,
   * .jpg) The 6 image files then would be:
   * {imageFilePrefix}+X{imageFileExtension}
   * {imageFilePrefix}-X{imageFileExtension}
   * {imageFilePrefix}+Y{imageFileExtension}
   * {imageFilePrefix}-Y{imageFileExtension}
   * {imageFilePrefix}+Z{imageFileExtension}
   * {imageFilePrefix}-Z{imageFileExtension}
   */
  void loadColorTexture(Mn::Trade::AbstractImporter& importer,
                        const std::string& imageFilePrefix,
                        const std::string& imageFileExtension);

 protected:
  const unsigned int colorAttachment_ = 1;
  int imageSize_ = 0;
  std::unique_ptr<Magnum::GL::CubeMapTexture> colorTexture_ = nullptr;
  std::unique_ptr<Magnum::GL::CubeMapTexture> depthTexture_ = nullptr;
  void recreateTexture();

  // framebuffer for drawable selection
  Magnum::GL::Framebuffer frameBuffer_{Magnum::NoCreate};

  /**
   * @brief recreate frame buffer
   */
  void recreateFramebuffer();

  /**
   * @brief prepare to draw to the texture
   */
  void prepareToDraw(int cubeSideIndex);
};

}  // namespace gfx
}  // namespace esp
#endif
