// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BaseMesh.h"
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include "esp/scene/SemanticScene.h"

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
            .except({0, 1}),
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

namespace {
// TODO remove when/if Magnum ever supports this function for Color3ub
constexpr const char Hex[]{"0123456789abcdef"};
}  // namespace
std::string BaseMesh::getColorAsString(Magnum::Color3ub color) const {
  char out[] = "#______";
  out[1] = Hex[(color.r() >> 4) & 0xf];
  out[2] = Hex[(color.r() >> 0) & 0xf];
  out[3] = Hex[(color.g() >> 4) & 0xf];
  out[4] = Hex[(color.g() >> 0) & 0xf];
  out[5] = Hex[(color.b() >> 4) & 0xf];
  out[6] = Hex[(color.b() >> 0) & 0xf];
  return std::string(out);
}

void BaseMesh::buildSemanticOBBs(
    const std::vector<Mn::Vector3>& vertices,
    const std::vector<uint16_t>& vertSemanticIDs,
    const std::vector<std::shared_ptr<esp::scene::SemanticObject>>& ssdObjs,
    const std::string& msgPrefix) const {
  // build per-SSD object vector of known semantic IDs
  std::size_t numSSDObjs = ssdObjs.size();
  std::vector<int> semanticIDToSSOBJidx(numSSDObjs, -1);
  for (int i = 0; i < numSSDObjs; ++i) {
    const auto& ssdObj = *ssdObjs[i];
    int semanticID = ssdObj.semanticID();
    // should not happen unless semantic ids are not sequential
    if (semanticIDToSSOBJidx.size() <= semanticID) {
      semanticIDToSSOBJidx.resize(semanticID + 1);
    }
    semanticIDToSSOBJidx[semanticID] = i;
  }

  // aggegates of per-semantic ID mins and maxes
  std::vector<Mn::Vector3> vertMax(
      semanticIDToSSOBJidx.size(),
      {-Mn::Constants::inf(), -Mn::Constants::inf(), -Mn::Constants::inf()});
  std::vector<Mn::Vector3> vertMin(
      semanticIDToSSOBJidx.size(),
      {Mn::Constants::inf(), Mn::Constants::inf(), Mn::Constants::inf()});
  std::vector<int> vertCounts(semanticIDToSSOBJidx.size());

  // for each vertex, map vert min and max for each known semantic ID
  // Known semantic IDs are expected to be contiguous, and correspond to the
  // number of unique ssdObjs mappings.

  for (int vertIdx = 0; vertIdx < vertSemanticIDs.size(); ++vertIdx) {
    // semantic ID on vertex - valid values are 1->semanticIDToSSOBJidx.size().
    // Invalid/unknown semantic ids are > semanticIDToSSOBJidx.size()
    const auto semanticID = vertSemanticIDs[vertIdx];
    if ((semanticID >= 0) && (semanticID < semanticIDToSSOBJidx.size())) {
      const auto vert = vertices[vertIdx];
      // FOR VERT-BASED OBB CALC
      // only support bbs for known colors that map to semantic objects
      vertMax[semanticID] = Mn::Math::max(vertMax[semanticID], vert);
      vertMin[semanticID] = Mn::Math::min(vertMin[semanticID], vert);
      vertCounts[semanticID] += 1;
    }
  }

  // with mins/maxs per ID, map to objs
  // give each ssdObj the values to build its OBB
  for (int semanticID = 0; semanticID < semanticIDToSSOBJidx.size();
       ++semanticID) {
    int objIdx = semanticIDToSSOBJidx[semanticID];
    if (objIdx == -1) {
      continue;
    }
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];
    Mn::Vector3 center{};
    Mn::Vector3 dims{};

    const std::string debugStr = Cr::Utility::formatString(
        "{} Semantic ID : {} : color : {} tag : {} present in {} "
        "verts | ",
        msgPrefix, semanticID,
        getColorAsString(static_cast<Mn::Color3ub>(ssdObj.getColor())),
        ssdObj.id(), vertCounts[semanticID]);
    if (vertCounts[semanticID] == 0) {
      ESP_DEBUG() << Cr::Utility::formatString(
          "{}No verts have specified Semantic ID.", debugStr);
    } else {
      center = .5f * (vertMax[semanticID] + vertMin[semanticID]);
      dims = vertMax[semanticID] - vertMin[semanticID];
      ESP_DEBUG() << Cr::Utility::formatString(
          "{}BB Center [{},{},{}] Dims [{},{},{}]", debugStr, center.x(),
          center.y(), center.z(), dims.x(), dims.y(), dims.z());
    }
    ssdObj.setObb(Mn::EigenIntegration::cast<esp::vec3f>(center),
                  Mn::EigenIntegration::cast<esp::vec3f>(dims));
  }
}  // BaseMesh::buildSemanticOBBs

}  // namespace assets
}  // namespace esp
