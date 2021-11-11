// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericSemanticMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/GenericGL.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "esp/core/Esp.h"
#include "esp/geo/Geo.h"
#include "esp/io/Json.h"
#include "esp/scene/HM3DSemanticScene.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

namespace {

// TODO: this could instead use Mn::Trade::MeshData directly
struct InstancePlyData {
  std::vector<vec3f> cpu_vbo;
  std::vector<vec3uc> cpu_cbo;
  std::vector<uint32_t> cpu_ibo;
  std::vector<uint16_t> objectIds;
  /**
   * @brief Whether or not the objectIds were provided by the source .ply file.
   * If so then we assume they can be used to provide islands to split the
   * semantic mesh for better frustum culling.
   */
  bool objIdsFromPly = false;
  bool objPartitionsFromSSD = false;

  /**
   * @brief This is used to map every vertex to a particular partition, for
   * culling.
   */
  std::vector<uint16_t> partitionIds;
};

Cr::Containers::Optional<InstancePlyData> parsePly(
    Mn::Trade::AbstractImporter& importer,
    const std::string& plyFile,
    std::vector<Magnum::Vector3ub>& colorMapToUse,
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  /* Open the file. On error the importer already prints a diagnostic message,
     so no need to do that here. The importer implicitly converts per-face
     attributes to per-vertex, so nothing extra needs to be done. */

  Cr::Containers::Optional<Mn::Trade::MeshData> meshData;
  if (!importer.openFile(plyFile) || !(meshData = importer.mesh(0))) {
    ESP_ERROR() << "Unable to import semantic .ply file named :" << plyFile
                << ". Aborting.";
    return Cr::Containers::NullOpt;
  }

  /* Copy attributes to the vectors. Positions and indices can be copied
     directly using the convenience APIs as we store them in the full type.
     The importer always provides an indexed mesh with positions, so no need
     for extra error checking. */
  InstancePlyData data;
  data.cpu_vbo.resize(meshData->vertexCount());
  data.cpu_ibo.resize(meshData->indexCount());
  meshData->positions3DInto(Cr::Containers::arrayCast<Mn::Vector3>(
      Cr::Containers::arrayView(data.cpu_vbo)));
  meshData->indicesInto(data.cpu_ibo);
  const std::string msgPrefix =
      Cr::Utility::formatString("PLY File {} :", plyFile);
  /* Assuming colors are 8-bit RGB to avoid expanding them to float and then
     packing back */
  ESP_CHECK(meshData->hasAttribute(Mn::Trade::MeshAttribute::Color),
            msgPrefix << "has no vertex colors defined, which are required.");

  data.cpu_cbo.resize(meshData->vertexCount());
  const Mn::VertexFormat colorFormat =
      meshData->attributeFormat(Mn::Trade::MeshAttribute::Color);
  Cr::Containers::StridedArrayView1D<const Magnum::Color3ub> meshColors;
  if (colorFormat == Mn::VertexFormat::Vector3ubNormalized) {
    meshColors =
        meshData->attribute<Mn::Color3ub>(Mn::Trade::MeshAttribute::Color);

  } else if (colorFormat == Mn::VertexFormat::Vector4ubNormalized) {
    // retrieve RGB view of RGBA color data and copy into data
    meshColors = Cr::Containers::arrayCast<const Mn::Color3ub>(
        meshData->attribute<Mn::Color4ub>(Mn::Trade::MeshAttribute::Color));

  } else {
    ESP_CHECK(false,
              msgPrefix << "Unexpected vertex color type " << colorFormat);
  }

  Cr::Utility::copy(meshColors, Cr::Containers::arrayCast<Mn::Color3ub>(
                                    Cr::Containers::arrayView(data.cpu_cbo)));

  // Check we actually have object IDs before copying them, and that those are
  // in a range we expect them to be
  data.objectIds.resize(meshData->vertexCount());
  Cr::Containers::Array<Mn::UnsignedInt> objectIds;

  if (meshData->hasAttribute(Mn::Trade::MeshAttribute::ObjectId)) {
    objectIds = meshData->objectIdsAsArray();
    data.objIdsFromPly = true;
    data.objPartitionsFromSSD = false;
    const int maxVal = Mn::Math::max(objectIds);
    ESP_CHECK(
        maxVal <= 65535,
        Cr::Utility::formatString(
            "{}Object IDs can't be stored into 16 bits : Max ID Value : {}",
            msgPrefix, maxVal));
    colorMapToUse.assign(Mn::DebugTools::ColorMap::turbo().begin(),
                         Mn::DebugTools::ColorMap::turbo().end());
  } else {
    // If object IDs are lacking, use vertex color to infer objectIDs, where
    // each unique color corresponds to a unique objectID.
    // These ids should probably not be used to split the mesh into submeshes
    // for culling purposes, since there may be very many different vertex
    // colors.
    data.objIdsFromPly = false;

    // If a vertex color array and array of semantic ids are provided via a
    // semantic scene descriptor, use these to replace the idxs found in using
    // the removeDuplicatesInPlace process and their color mappings
    if ((semanticScene != nullptr) && semanticScene->hasVertColorsDefined()) {
      // built partion IDs from SSD regions, to split mesh for culling, instead
      // of objectIDs
      data.objPartitionsFromSSD = true;

      // from SSD's objects_ array - this is the number of defined semantic
      // descriptors corresponding to object instances.
      // all this tapdancing because of the design of the Semantic Scene.
      const auto& ssdObjs = semanticScene->objects();
      std::size_t numSSDObjs = ssdObjs.size();

      // map of colors as ints as key, where value is objectID and regionID from
      // ssd
      std::unordered_map<uint32_t, std::pair<int, int>>
          tmpColorMapToSSDidAndRegionIndex;
      tmpColorMapToSSDidAndRegionIndex.reserve(numSSDObjs);
      // lambda function to convert a 3-vec representation of a color into an
      // int
      auto colorAsInt = [](const Mn::Vector3ub& color) -> uint32_t {
        return (unsigned(color[0]) << 16) | (unsigned(color[1]) << 8) |
               unsigned(color[2]);
      };

      // build maps of color ints to semantic IDs and color ints to region ids
      // in SSD
      // find max regions present, so we can have an "unassigned"/"unknown"
      // region to move all verts whose region is

      int maxRegion = -1;
      // temporary map keyed by semantic ID and value being actual color for
      // that ID. We need this since we do not know how many, if any, unique
      // colors that do not have Semantic Mappings exist on the mesh
      std::map<int, Mn::Vector3ub> tmpDestSSDColorMap;
      for (int i = 0; i < numSSDObjs; ++i) {
        const auto& ssdObj =
            static_cast<scene::HM3DObjectInstance&>(*ssdObjs[i]);
        const Mn::Vector3ub ssdColor = ssdObj.getColor();
        const uint32_t colorInt = colorAsInt(ssdColor);
        int semanticID = ssdObj.getSemanticID();
        int regionIDX =
            static_cast<scene::HM3DSemanticRegion&>(*ssdObj.region())
                .getIndex();
        tmpColorMapToSSDidAndRegionIndex[colorInt] = {semanticID, regionIDX};

        tmpDestSSDColorMap[semanticID] = ssdColor;

        maxRegion = Mn::Math::max(regionIDX, maxRegion);
      }
      // find largest key in map
      int maxSemanticID = tmpDestSSDColorMap.rbegin()->first;
      // increment maxRegion to use for unknown regions whose colors are not
      // mapped
      ++maxRegion;

      // rebuild objectIDs vector (objectID per vertex) based on
      // per vertex colors mapped
      // 1st semantic ID for colors not found in SSD
      std::size_t nonSSDObjID = maxSemanticID + 1;

      // map regionIDs to affected verts using semantic color provided in
      // SemanticScene, as specified in SSD file.
      data.partitionIds.resize(meshData->vertexCount());
      // map semantic IDs corresponding to colors from SSD file to appropriate
      // verts (via index)
      data.objectIds.resize(meshData->vertexCount());
      // temporary holding structure to hold any non-SSD vert colors, so that
      // the nonSSDObjID for new colors can be incremented appropriately
      // not using set to avoid extra include
      std::unordered_map<uint32_t, int> nonSSDVertColors;

      // only go through all verts one time
      // derive semantic ID and color and region/room ID for culling
      int regionID = 0;
      int semanticID = 0;
      for (int vertIdx = 0; vertIdx < meshData->vertexCount(); ++vertIdx) {
        Mn::Vector3ub meshColor = meshColors[vertIdx];
        // Convert color to an int @ vertex
        const uint32_t meshColorInt = colorAsInt(meshColor);

        std::unordered_map<uint32_t, std::pair<int, int>>::const_iterator
            ssdColorToIDAndRegionIter =
                tmpColorMapToSSDidAndRegionIndex.find(meshColorInt);

        if (ssdColorToIDAndRegionIter !=
            tmpColorMapToSSDidAndRegionIndex.end()) {
          // color is found in ssd mapping, so is legal color
          semanticID = ssdColorToIDAndRegionIter->second.first;
          regionID = ssdColorToIDAndRegionIter->second.second;
        } else {
          // color is not found in ssd mapping, so not legal color
          // check if we've assigned a semantic ID to this color before, and if
          // not do so
          auto nonSSDClrRes =
              nonSSDVertColors.insert({meshColorInt, nonSSDObjID});
          if (nonSSDClrRes.second) {
            // inserted, so increment nonSSDObjID
            ++nonSSDObjID;
          }
          // map holds that color's nonSSDObjID
          semanticID = nonSSDClrRes.first->second;
          regionID = maxRegion;

          // color for given semantic ID - only necessary to add for unknown
          // colors
          tmpDestSSDColorMap[semanticID] = meshColor;
        }
        // semantic ID for vertex
        data.objectIds[vertIdx] = semanticID;
        // partition Ids for each vertex, for multi-mesh construction.
        data.partitionIds[vertIdx] = regionID;
      }

      // color map with first nonSSDObjID-1 elements in proper order to match
      // provided semantic IDs (so that ID is IDX of semantic color in map) and
      // any overflow colors uniquely map 1-to-1 to an unmapped semantic ID as
      // their index.
      colorMapToUse.resize(tmpDestSSDColorMap.rbegin()->first + 1);
      for (const auto& elem : tmpDestSSDColorMap) {
        colorMapToUse[elem.first] = elem.second;
      }

    } else {
      // no remapping provided by SSD so just use what is synthesized
      Cr::Containers::Array<Mn::Color3ub> colorsThatBecomeTheColorMap{
          Mn::NoInit, meshColors.size()};
      // copy the meshColors into colorsThatBecomeTheColorMap
      Cr::Utility::copy(meshColors, colorsThatBecomeTheColorMap);

      // removeDuplicatesInPlace returns array of unique ids for colors, and
      std::pair<Cr::Containers::Array<Mn::UnsignedInt>, std::size_t> out =
          Mn::MeshTools::removeDuplicatesInPlace(
              Cr::Containers::arrayCast<2, char>(
                  stridedArrayView(colorsThatBecomeTheColorMap)));

      objectIds = std::move(out.first);
      auto clrMapView = colorsThatBecomeTheColorMap.prefix(out.second);
      colorMapToUse.assign(clrMapView.begin(), clrMapView.end());

      data.objPartitionsFromSSD = false;
    }
  }
  if (!data.objPartitionsFromSSD) {
    // if data.objPartitionsFromSSD then data.objectIds were populated directly
    Mn::Math::castInto(Cr::Containers::arrayCast<2, Mn::UnsignedInt>(
                           Cr::Containers::stridedArrayView(objectIds)),
                       Cr::Containers::arrayCast<2, Mn::UnsignedShort>(
                           Cr::Containers::stridedArrayView(data.objectIds)));
  }

  // Generic Semantic PLY meshes have -Z gravity
  const quatf T_esp_scene =
      quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

  for (auto& xyz : data.cpu_vbo) {
    xyz = T_esp_scene * xyz;
  }
  return data;
}  // parsePly

}  // namespace

std::vector<std::unique_ptr<GenericSemanticMeshData>>
GenericSemanticMeshData::fromPLY(
    Mn::Trade::AbstractImporter& importer,
    const std::string& plyFile,
    const bool splitMesh,
    std::vector<Magnum::Vector3ub>& colorMapToUse,
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  Cr::Containers::Optional<InstancePlyData> parseResult =
      parsePly(importer, plyFile, colorMapToUse, semanticScene);

  if (!parseResult) {
    return {};
  }

  std::vector<GenericSemanticMeshData::uptr> splitMeshData;
  if (splitMesh &&
      (parseResult->objIdsFromPly || parseResult->objPartitionsFromSSD)) {
    std::vector<uint16_t>& meshPartitionIds = parseResult->objIdsFromPly
                                                  ? parseResult->objectIds
                                                  : parseResult->partitionIds;

    std::unordered_map<uint16_t, PerPartitionIdMeshBuilder>
        partitionIdToObjectData;

    for (size_t i = 0; i < parseResult->cpu_ibo.size(); ++i) {
      const uint32_t globalIndex = parseResult->cpu_ibo[i];
      const uint16_t objectId = parseResult->objectIds[globalIndex];
      const uint16_t partitionId = meshPartitionIds[globalIndex];
      // if not found in map to data create new mesh
      if (partitionIdToObjectData.find(partitionId) ==
          partitionIdToObjectData.end()) {
        auto instanceMesh = GenericSemanticMeshData::create_unique();
        partitionIdToObjectData.emplace(
            partitionId, PerPartitionIdMeshBuilder{*instanceMesh, partitionId});
        splitMeshData.emplace_back(std::move(instanceMesh));
      }
      partitionIdToObjectData.at(partitionId)
          .addVertex(globalIndex, parseResult->cpu_vbo[globalIndex],
                     parseResult->cpu_cbo[globalIndex], objectId);
    }
    // Update collision mesh data for each mesh
    for (size_t i = 0; i < splitMeshData.size(); ++i) {
      splitMeshData[i]->updateCollisionMeshData();
    }
  } else {
    // ply should not be split - ids were synthesized
    auto meshData = GenericSemanticMeshData::create_unique();
    meshData->cpu_vbo_ = std::move(parseResult->cpu_vbo);
    meshData->cpu_cbo_ = std::move(parseResult->cpu_cbo);
    meshData->cpu_ibo_ = std::move(parseResult->cpu_ibo);
    meshData->objectIds_ = std::move(parseResult->objectIds);
    // Construct vertices for collsion meshData
    // Store indices, facd_ids in Magnum MeshData3D format such that
    // later they can be accessed.
    // Note that normal and texture data are not stored
    meshData->collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;
    meshData->updateCollisionMeshData();
    splitMeshData.emplace_back(std::move(meshData));
  }
  return splitMeshData;

}  // GenericSemanticMeshData::fromPLY

void GenericSemanticMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  Mn::GL::Buffer vertices, indices;
  indices.setTargetHint(Mn::GL::Buffer::TargetHint::ElementArray);
  indices.setData(cpu_ibo_, Mn::GL::BufferUsage::StaticDraw);

  vertices.setData(
      Mn::MeshTools::interleave(cpu_vbo_, cpu_cbo_, 1, objectIds_, 2),
      Mn::GL::BufferUsage::StaticDraw);

  renderingBuffer_ =
      std::make_unique<GenericSemanticMeshData::RenderingBuffer>();
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(cpu_ibo_.size())
      .addVertexBuffer(
          std::move(vertices), 0, Mn::Shaders::GenericGL3D::Position{},
          Mn::Shaders::GenericGL3D::Color3{
              Mn::Shaders::GenericGL3D::Color3::DataType::UnsignedByte,
              Mn::Shaders::GenericGL3D::Color3::DataOption::Normalized},
          1,
          Mn::Shaders::GenericGL3D::ObjectId{
              Mn::Shaders::GenericGL3D::ObjectId::DataType::UnsignedShort},
          2)
      .setIndexBuffer(std::move(indices), 0,
                      Mn::GL::MeshIndexType::UnsignedInt);

  updateCollisionMeshData();

  buffersOnGPU_ = true;
}

Magnum::GL::Mesh* GenericSemanticMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

void GenericSemanticMeshData::updateCollisionMeshData() {
  collisionMeshData_.positions = Cr::Containers::arrayCast<Mn::Vector3>(
      Cr::Containers::arrayView(cpu_vbo_));
  collisionMeshData_.indices = Cr::Containers::arrayCast<Mn::UnsignedInt>(
      Cr::Containers::arrayView(cpu_ibo_));
}

void GenericSemanticMeshData::PerPartitionIdMeshBuilder::addVertex(
    uint32_t vertexId,
    const vec3f& position,
    const vec3uc& color,
    int objectId) {
  // if we haven't seen this vertex, add it to the local vertex/color buffer
  auto result = vertexIdToVertexIndex_.emplace(vertexId, data_.cpu_vbo_.size());
  if (result.second) {
    data_.cpu_vbo_.emplace_back(position);
    data_.cpu_cbo_.emplace_back(color);
    data_.objectIds_.emplace_back(objectId);
  }

  // update index buffers with local index of vertex/color
  data_.cpu_ibo_.emplace_back(result.first->second);
}

}  // namespace assets
}  // namespace esp
