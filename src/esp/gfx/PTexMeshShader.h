// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PTEXMESHSHADER_H_
#define ESP_GFX_PTEXMESHSHADER_H_

#include <memory>
#include <vector>

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Matrix4.h>

#include "esp/assets/PTexMeshData.h"

namespace esp {

// forward declaration
namespace assets {
class PTexMeshData;
};

namespace gfx {

class PTexMeshShader : public Magnum::GL::AbstractShaderProgram {
 public:
  //! @brief vertex positions
  typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;

  /**
   * @brief Constructor
   */
  explicit PTexMeshShader();

  // ======== texture binding ========
  /**
   * @brief Bind the atlas texture
   * @return Reference to self (for method chaining)
   */
  PTexMeshShader& bindAtlasTexture(Magnum::GL::Texture2D& texture);
  /**
   *  @brief Bind the buffer texture containing the adjacent faces
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& bindAdjFacesBufferTexture(Magnum::GL::BufferTexture& texture);

  // ======== set uniforms ===========
  /**
   *  @brief Set modelview and projection matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setMVPMatrix(const Magnum::Matrix4& matrix);
  /**
   *  @brief Set expsure to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setExposure(float exposure);
  /**
   *  @brief Set gamma to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setGamma(float gamma);
  /**
   *  @brief Set saturation to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setSaturation(float saturation);
  /**
   *  @brief Set the tile size of the atlas texture
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setAtlasTextureSize(Magnum::GL::Texture2D& texture,
                                      uint32_t tileSize);
  /**
   *  @brief Set object id to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PTexMeshShader& setObjectId(unsigned int objectId);

 protected:
  // it hurts the performance to call glGetUniformLocation() every frame due to
  // string operations.
  // therefore, cache the locations in the constructor
  int MVPMatrixUniform_;
  int exposureUniform_;
  int gammaUniform_;
  int saturationUniform_;
  int tileSizeUniform_;
  int widthInTilesUniform_;
  int objectIdUniform_;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PTEXMESHSHADER_H_
