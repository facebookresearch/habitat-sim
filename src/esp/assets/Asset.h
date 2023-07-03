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

//! Supported Asset types
enum class AssetType {
  UNKNOWN,
  MP3D_MESH,
  INSTANCE_MESH,
  UNKNOWN2,
  NAVMESH,
  PRIMITIVE,
};

// loading and asset info with filepath == EMPTY_SCENE creates a scene graph
// with no scene mesh (ie. an empty scene)
constexpr char EMPTY_SCENE[] = "NONE";

//! stores basic Phong compatible color properties for procedural override
//! material construction
struct PhongMaterialColor {
  Magnum::Color4 ambientColor{0.1};
  Magnum::Color4 diffuseColor{0.7};
  Magnum::Color4 specularColor{0.2};
};

bool operator==(const PhongMaterialColor& a, const PhongMaterialColor& b);
bool operator!=(const PhongMaterialColor& a, const PhongMaterialColor& b);

//! AssetInfo stores information necessary to identify and load an Asset
struct AssetInfo {
  AssetType type = AssetType::UNKNOWN;
  std::string filepath = EMPTY_SCENE;  // empty scene
  geo::CoordinateFrame frame{};
  float virtualUnitToMeters = 1.0f;
  bool forceFlatShading = true;
  bool splitInstanceMesh = true;  // only applies to AssetType::INSTANCE_MESH

  //! if set, override the asset material with a procedural Phong material
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
bool operator==(const AssetInfo& a, const AssetInfo& b);
bool operator!=(const AssetInfo& a, const AssetInfo& b);

//! Wrapper for all valid asset types
template <typename T>
struct Asset {
  //! Create Asset wrapper given AssetInfo and created asset
  Asset(AssetInfo& info, T& asset) : info_(info), asset_(asset) {}

  //! Return AssetInfo for this Asset
  const AssetInfo& info() { return info_; }

  //! Return reference to contained Asset
  T& get() { return asset_; }

 protected:
  AssetInfo info_;
  T& asset_;
  ESP_SMART_POINTERS(Asset)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_ASSET_H_
