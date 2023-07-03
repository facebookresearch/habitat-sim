// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Asset.h"

#include <Corrade/Utility/String.h>
#include "esp/geo/CoordinateFrame.h"

namespace esp {
namespace assets {

AssetInfo AssetInfo::fromPath(const std::string& path) {
  using Corrade::Utility::String::endsWith;
  AssetInfo info{AssetType::UNKNOWN, path};

  if (endsWith(path, "_semantic.ply")) {
    info.type = AssetType::INSTANCE_MESH;
  } else if (endsWith(path, ".glb")) {
    // assumes MP3D glb with gravity = -Z
    info.type = AssetType::MP3D_MESH;
    // Create a coordinate for the mesh by rotating the default ESP
    // coordinate frame to -Z gravity
    info.frame = geo::CoordinateFrame(geo::ESP_BACK, geo::ESP_UP);
  }

  return info;
}

bool operator==(const AssetInfo& a, const AssetInfo& b) {
  return a.type == b.type && a.filepath == b.filepath && a.frame == b.frame &&
         a.shaderTypeToUse == b.shaderTypeToUse &&
         a.hasSemanticTextures == b.hasSemanticTextures &&
         a.virtualUnitToMeters == b.virtualUnitToMeters &&
         a.splitInstanceMesh == b.splitInstanceMesh &&
         a.overridePhongMaterial == b.overridePhongMaterial &&
         a.forceFlatShading == b.forceFlatShading;
}

bool operator!=(const AssetInfo& a, const AssetInfo& b) {
  return !(a == b);
}

bool operator==(const PhongMaterialColor& a, const PhongMaterialColor& b) {
  return a.ambientColor == b.ambientColor && a.diffuseColor == b.diffuseColor &&
         a.specularColor == b.specularColor;
}

bool operator!=(const PhongMaterialColor& a, const PhongMaterialColor& b) {
  return !(a == b);
}

}  // namespace assets
}  // namespace esp
