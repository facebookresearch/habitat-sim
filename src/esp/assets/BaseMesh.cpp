// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BaseMesh.h"
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>

namespace Cr = Corrade;
namespace Mn = Magnum;
namespace esp {
namespace assets {

bool BaseMesh::setMeshType(SupportedMeshType type) {
  if (type < SupportedMeshType::NOT_DEFINED ||
      type >= SupportedMeshType::NUM_SUPPORTED_MESH_TYPES) {
    ESP_ERROR() << "Cannot set the mesh type to" << type;
    return false;
  }

  type_ = type;
  return true;
}

void BaseMesh::convertMeshColors(
    const Mn::Trade::MeshData& srcMeshData,
    bool convertToSRGB,
    Cr::Containers::Array<Mn::Color3ub>& meshColors) const {
  /* Assuming colors are 8-bit RGB to avoid expanding them to float and then
     packing back */
  auto colors = srcMeshData.colorsAsArray();
  if (convertToSRGB) {
    for (std::size_t i = 0; i != colors.size(); ++i) {
      meshColors[i] = colors[i].rgb().toSrgb<Mn::UnsignedByte>();
    }
  } else {
    Mn::Math::packInto(
        Cr::Containers::arrayCast<2, float>(stridedArrayView(colors))
            .exceptSuffix({0, 1}),
        Cr::Containers::arrayCast<2, Mn::UnsignedByte>(
            stridedArrayView(meshColors)));
  }
}  // BaseMesh::convertMeshColors

void BaseMesh::buildColorMapToUse(
    Cr::Containers::Array<Magnum::UnsignedInt>& vertIDs,
    const Cr::Containers::Array<Mn::Color3ub>& vertColors,
    bool useVertexColors,
    std::vector<Mn::Vector3ub>& colorMapToUse) const {
  colorMapToUse.clear();
  if (useVertexColors) {
    // removeDuplicates returns array of unique idxs for ids, to
    // be used on meshColors to provide mappings for colorMapToUse
    std::pair<Cr::Containers::Array<Mn::UnsignedInt>, std::size_t> out =
        Mn::MeshTools::removeDuplicates(
            Cr::Containers::arrayCast<2, char>(stridedArrayView(vertIDs)));

    // out holds index array of object IDs.  query

    // color map exists so build colorMapToUse by going through every
    // add 1 to account for no assigned 0 value
    colorMapToUse.resize(out.second + 1);
    // set semanticID 0 == black
    colorMapToUse[0] = Mn::Color3ub{};
    for (const auto& vertIdx : out.first) {
      // find objectID @ vert idx and color @ vert idx
      int semanticID = vertIDs[vertIdx];
      colorMapToUse[semanticID] = vertColors[vertIdx];
    }
  } else {
    colorMapToUse.assign(Mn::DebugTools::ColorMap::turbo().begin(),
                         Mn::DebugTools::ColorMap::turbo().end());
  }

}  // BaseMesh::buildColorMapToUse

}  // namespace assets
}  // namespace esp
