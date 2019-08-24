// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Asset.h"

#include <Corrade/Utility/String.h>

namespace esp {
namespace assets {

using namespace Corrade::Utility::String;

AssetInfo AssetInfo::fromPath(const std::string& path) {
  AssetInfo info{AssetType::UNKNOWN, path};

  if (endsWith(path, "_semantic.ply")) {
    info.type = AssetType::INSTANCE_MESH;
  } else if (endsWith(path, "mesh.ply")) {
    info.type = AssetType::FRL_PTEX_MESH;
    info.frame = {quatf::FromTwoVectors(geo::ESP_GRAVITY, -vec3f::UnitZ())};
  } else if (endsWith(path, "house.json")) {
    info.type = AssetType::SUNCG_SCENE;
  } else if (endsWith(path, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    info.type = AssetType::MP3D_MESH;
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    info.frame = {quatf::FromTwoVectors(geo::ESP_GRAVITY, -vec3f::UnitZ())};
  }

  return info;
}

}  // namespace assets
}  // namespace esp
