// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/geo/CoordinateFrame.h"

namespace esp {
namespace assets {

//! Supported Asset types
enum class AssetType {
  UNKNOWN,
  SUNCG_OBJECT,
  SUNCG_SCENE,
  MP3D_MESH,
  INSTANCE_MESH,
  FRL_PTEX_MESH,
  FRL_INSTANCE_MESH,
  NAVMESH,
};

//! AssetInfo stores information necessary to identify and load an Asset
struct AssetInfo {
  AssetType type = AssetType::UNKNOWN;
  std::string filepath = "";
  geo::CoordinateFrame frame;
  float virtualUnitToMeters = 1.0f;

  //! Populates a preset AssetInfo by matching against known filepaths
  static AssetInfo fromPath(const std::string& filepath);

  ESP_SMART_POINTERS(AssetInfo)
};

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
