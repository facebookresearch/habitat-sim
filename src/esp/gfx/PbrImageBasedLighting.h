// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_IBL_H_
#define ESP_GFX_PBR_IBL_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Texture.h>

#include "CubeMap.h"
#include "PbrEquiRectangularToCubeMapShader.h"
#include "PbrPrecomputedMapShader.h"
#include "esp/core/Esp.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {
class PbrImageBasedLighting {
 public:
  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * enable indirect diffuse part of the lighting equation
     */
    IndirectDiffuse = 1 << 0,

    /**
     * enable indirect specular part of the lighting equation
     */
    IndirectSpecular = 1 << 1,

    /**
     * use LDR images for irradiance cube map, brdf lookup table, pre-fitered
     * cube map
     */
    UseLDRImages = 1 << 2,
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief constructor
   * @param[in] flags, flags that indicate settings
   * @param[in] shaderManager, the shader manager that manages all the shaders
   * @param[in] hdriImageFilename, the name of the
   * HDRi image (an equirectangular image), that will be converted to a
   * environment cube map
   * NOTE!!! Such an image MUST be SPECIFIED in the
   * ~/habitat-sim/data/pbr/PbrImages.conf
   * and be put in that folder.
   * example image:
   * ~/habitat-sim/data/pbr/lythwood_room_4k.png
   */
  explicit PbrImageBasedLighting(Flags flags,
                                 ShaderManager& shaderManager,
                                 const std::string& hdriImageFilename);

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

  /**
   * @brief get the brdf lookup table
   * @param[in] equirectangularImageFilename, the name (path included) of the
   * equirectangular image, that will be converted to a environment cube map
   */
  void convertEquirectangularToCubeMap(
      const std::string& equirectangularImageFilename);

 private:
  Flags flags_;

  /**
   * @brief the environment map (e.g., a sky box)
   */
  Cr::Containers::Optional<CubeMap> environmentMap_;
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
  /**
   * @brief 2D BRDF lookup table, an HDR image (16-bits per channel) that
   * contains BRDF values for roughness and view angle. This is for the indirect
   * specular.
   * See: Karis, Brian. “Real Shading in Unreal Engine 4” (2013).
   */
  Cr::Containers::Optional<Magnum::GL::Texture2D> brdfLUT_;

  ShaderManager& shaderManager_;
  /**
   * @brief recreate brdf lookup table, irradiance map, prefiltered environment
   * map
   */
  void recreateTextures();

  /** @brief load the brdf LUT from the disk */
  // TODO: HDR
  void loadBrdfLookUpTable();

  enum class PrecomputedMapType : uint8_t {
    IrradianceMap = 0,
    PrefilteredMap = 1,
  };

  /**
   * @brief precompute the irradiance map, prefiltered environment map
   * @param[in] type, see @ref PrecomputedMapType
   */
  void computePrecomputedMap(PrecomputedMapType type);

  enum class PbrIblShaderType : uint8_t {
    IrradianceMap = 0,
    PrefilteredMap = 1,
    // BrdfLookupTable = 2, // TODO
    EquirectangularToCubeMap = 3,
  };
  /**
   * @brief get the shader based on the type
   * @param[in] type, see @ref PbrIblShaderType
   */
  template <typename T>
  Mn::Resource<Mn::GL::AbstractShaderProgram, T> getShader(
      PbrIblShaderType type);
};

CORRADE_ENUMSET_OPERATORS(PbrImageBasedLighting::Flags)

}  // namespace gfx
}  // namespace esp

#endif
