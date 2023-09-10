// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_IBL_H_
#define ESP_GFX_PBR_IBL_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Texture.h>

#include "CubeMap.h"
#include "PbrEquiRectangularToCubeMapShader.h"
#include "PbrPrecomputedMapShader.h"
#include "esp/core/Esp.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

/**
 * @brief This class performs 2 functions. It derives the Irradiance and
 * Precomputed Cubemaps based on an Enironment map texture, and it provides
 * references to all the assets that are then consumed by the PBR shader for IBL
 * functionality.
 */
class PbrIBLHelper {
 public:
  /**
   * @brief constructor
   * @param[in] shaderManager the shader manager that manages all the shaders
   * @param[in] brdfLUT the brdf lookup table texture being used.
   * @param[in] envMapTexture the texture to use to build the environment cube
   * maps.
   */
  explicit PbrIBLHelper(
      ShaderManager& shaderManager,
      const std::shared_ptr<Mn::GL::Texture2D>& brdfLUT,
      const std::shared_ptr<Mn::GL::Texture2D>& envMapTexture);

  /**
   * @brief get the irradiance cube map
   */
  CubeMap& getIrradianceMap();

  /**
   * @brief get the pre-filtered cube map
   */
  CubeMap& getPrefilteredMap();

  /**
   * @brief get the brdf lookup table
   */
  Magnum::GL::Texture2D& getBrdfLookupTable();

 private:
  /**
   * @brief load the equirectangular env map and convert it to an environmental
   * cube map
   * @param[in] envMapTexture The loaded equirectangular texture, which will be
   * converted to an environment cube map
   */
  void convertEquirectangularToCubeMap(
      const std::shared_ptr<Mn::GL::Texture2D>& envMapTexture);

  /**
   * @brief 2D BRDF lookup table, an HDR image (16-bits per channel) that
   * contains BRDF values for roughness and view angle. This is for the indirect
   * specular.
   * See: Karis, Brian. “Real Shading in Unreal Engine 4” (2013).
   */
  std::shared_ptr<Mn::GL::Texture2D> brdfLUT_;

  /**
   * @brief irradiance cube map (default size of each face: 64 x 64 pixels),
   * that stores light radiated from the environment. This is for the
   * indirect diffuse.
   */
  Cr::Containers::Optional<CubeMap> irradianceMap_;
  /**
   *@brief Pre-filtered environment map that stores specular contribution for
   * different roughness values and store the results in the mip-map levels of a
   * cubemap (namely, a mip chain from roughness = 0.0 to roughness = 1.0.)
   * This is sampled for the indirect specular.
   */
  Cr::Containers::Optional<CubeMap> prefilteredMap_;

  ShaderManager& shaderManager_;

  /**
   * @brief precompute the irradiance map
   * @param[in] envCubeMap cube map of environment
   */
  void computeIrradianceMap(Magnum::GL::CubeMapTexture& envCubeMap);

  /**
   * @brief precompute the prefiltered environment map
   * @param[in] envCubeMap cube map of environment
   */
  void computePrefilteredEnvMap(Magnum::GL::CubeMapTexture& envCubeMap);
};

}  // namespace gfx
}  // namespace esp

#endif
