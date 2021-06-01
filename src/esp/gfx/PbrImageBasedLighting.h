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
#include "PbrIrradianceMapShader.h"
#include "esp/core/esp.h"
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
   */
  explicit PbrImageBasedLighting(Flags flags, ShaderManager& shaderManager);

  /**
   * @brief constructor
   * @param[in] flags, flags that indicate settings
   * @param[in] shaderManager, the shader manager that manages all the shaders
   * @param[in] equirectangularImageFilename, the name (path included) of the
   * equirectangular image, that will be converted to a environment cube map
   */
  explicit PbrImageBasedLighting(
      Flags flags,
      ShaderManager& shaderManager,
      const std::string& equirectangularImageFilename);

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
  void loadBrdfLookUpTable();

  ShaderManager& shaderManager_;

  void recreateTextures();
  void computeIrradianceMap();

  enum class PbrIblShaderType : uint8_t {
    IrradianceMap = 0,
    // PrefilteredMap = 1,
    // BrdfLookupTable = 2,
    EquirectangularToCubeMap = 3,
  };
  template <typename T>
  Mn::Resource<Mn::GL::AbstractShaderProgram, T> getShader(
      PbrIblShaderType type) {
    Mn::ResourceKey key;
    switch (type) {
      case PbrIblShaderType::IrradianceMap:
        key = Mn::ResourceKey{"irradianceMap"};
        break;
        /*
          case PbrIblShaderType::PrefilteredMap:
            key = Mn::ResourceKey{"prefilteredMap"};
            break;
        */
      case PbrIblShaderType::EquirectangularToCubeMap:
        key = Mn::ResourceKey{"equirectangularToCubeMap"};
        break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
    Mn::Resource<Mn::GL::AbstractShaderProgram, T> shader =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, T>(key);

    if (!shader) {
      if (type == PbrIblShaderType::IrradianceMap) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(), new PbrIrradianceMapShader(),
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      } else if (type == PbrIblShaderType::EquirectangularToCubeMap) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(), new PbrEquiRectangularToCubeMapShader(),
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      }
      /*
    else if (type == PbrIblShaderType::PrefilteredMap) {
      // TODO: prefilteredMap shader
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader.key(), new PbrPrefilteredMapShader(),
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }
      */
    }
    CORRADE_INTERNAL_ASSERT(shader);

    return shader;
  }
};

CORRADE_ENUMSET_OPERATORS(PbrImageBasedLighting::Flags)

}  // namespace gfx
}  // namespace esp

#endif
