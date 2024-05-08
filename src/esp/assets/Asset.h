// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_ASSET_H_
#define ESP_ASSETS_ASSET_H_

#include <Magnum/Math/Color.h>
#include "esp/core/Esp.h"
#include "esp/geo/CoordinateFrame.h"
#include "esp/metadata/attributes/AttributesEnumMaps.h"

namespace esp {

//! Asset management namespace
namespace assets {

/**
 * @brief stores basic Phong compatible color properties for procedural override
 * material construction
 */
struct PhongMaterialColor {
  /**
   * @brief the ambient color of the Phong material
   */
  Magnum::Color4 ambientColor{0.1};
  /**
   * @brief the diffuse color of the Phong material
   */
  Magnum::Color4 diffuseColor{0.7};
  /**
   * @brief the specular color of the Phong material
   */
  Magnum::Color4 specularColor{0.2};
};
/**
 * @brief Verify @ref PhongMaterialColor equality.
 */
bool operator==(const PhongMaterialColor& a, const PhongMaterialColor& b);
/**
 * @brief Verify @ref PhongMaterialColor inequality.
 */
bool operator!=(const PhongMaterialColor& a, const PhongMaterialColor& b);

/**
 * @brief AssetInfo stores information necessary to identify and load an Asset
 */
struct AssetInfo {
  /**
   * @brief The type of the asset
   */
  metadata::attributes::AssetType type =
      metadata::attributes::AssetType::Unknown;
  /**
   * @brief The path to the asset's source on disk
   */
  std::string filepath = esp::EMPTY_SCENE;  // empty scene
  /**
   * The @ref esp::geo::CoordinateFrame describing the default orientation of the asset
   */
  geo::CoordinateFrame frame{};
  /**
   * @brief Conversion factor from units specified in asset source to meters
   */
  float virtualUnitToMeters = 1.0f;

  /**
   * @brief Whether to force this asset to be flat shaded.
   */
  bool forceFlatShading = true;
  /**
   * @brief Whether supported semantic meshes should be split
   */
  bool splitInstanceMesh = true;  // only applies to AssetType::InstanceMesh

  /**
   * @brief if set, override the asset material with a procedural Phong material
   */
  Cr::Containers::Optional<PhongMaterialColor> overridePhongMaterial =
      Cr::Containers::NullOpt;
  /**
   * @brief Defaults to @ref
   * esp::metadata::attributes::ObjectInstanceShaderType::Unspecified (which
   * means the user has not specified a shader/material to use, and the assets's
   * default material should be used). If set to other value, this specifies the
   * shader type to use for this asset, overriding any other inferred shader
   * types. See @ref esp::metadata::attributes::ObjectInstanceShaderType
   */
  metadata::attributes::ObjectInstanceShaderType shaderTypeToUse =
      metadata::attributes::ObjectInstanceShaderType::Unspecified;

  /**
   * @brief Asset is a semantic mesh that uses textures (instead of vertices)
   * for annotation information.
   */
  bool hasSemanticTextures = false;

  //! Populates a preset AssetInfo by matching against known filepaths
  static AssetInfo fromPath(const std::string& filepath);

  ESP_SMART_POINTERS(AssetInfo)
};
/**
 * @brief Verify @ref AssetInfo equality.
 */
bool operator==(const AssetInfo& a, const AssetInfo& b);
/**
 * @brief Verify @ref AssetInfo equality.
 */
bool operator!=(const AssetInfo& a, const AssetInfo& b);

/**
 * @brief Wrapper for all valid asset types
 */
template <typename T>
struct Asset {
  //! Create Asset wrapper given AssetInfo and created asset
  Asset(AssetInfo& info, T& asset) : info_(info), asset_(asset) {}

  //! Return AssetInfo for this Asset
  const AssetInfo& info() { return info_; }

  //! Return reference to contained Asset
  T& get() { return asset_; }

 protected:
  /**
   * @brief The @ref AssetInfo for this asset
   */
  AssetInfo info_;
  /**
   * @brief the asset
   */
  T& asset_;
  ESP_SMART_POINTERS(Asset)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ASSET_H_
