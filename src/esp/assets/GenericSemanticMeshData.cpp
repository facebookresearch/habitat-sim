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
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/GenericGL.h>

#include "esp/scene/HM3DSemanticScene.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

std::vector<std::unique_ptr<GenericSemanticMeshData>>
GenericSemanticMeshData::buildSemanticMeshData(
    const Mn::Trade::MeshData& srcMeshData,
    const std::string& semanticFilename,
    const bool splitMesh,
    std::vector<Mn::Vector3ub>& colorMapToUse,
    bool convertToSRGB,
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  // build text prefix used in log messages
  const std::string msgPrefix =
      Cr::Utility::formatString("Parsing Semantic File {} :", semanticFilename);

  // Check for required colors.
  ESP_CHECK(srcMeshData.hasAttribute(Mn::Trade::MeshAttribute::Color),
            msgPrefix << "has no vertex colors defined, which are required for "
                         "semantic meshes.");

  /* Copy attributes to the vectors. Positions and indices can be copied
     directly using the convenience APIs as we store them in the full type.
     The importer always provides an indexed mesh with positions, so no need
     for extra error checking. */

  auto semanticData = GenericSemanticMeshData::create_unique();
  const auto numVerts = srcMeshData.vertexCount();

  semanticData->cpu_ibo_.resize(srcMeshData.indexCount());
  semanticData->cpu_vbo_.resize(numVerts);
  semanticData->cpu_cbo_.resize(numVerts);
  semanticData->objectIds_.resize(numVerts);

  srcMeshData.positions3DInto(Cr::Containers::arrayCast<Mn::Vector3>(
      Cr::Containers::arrayView(semanticData->cpu_vbo_)));

  if (semanticFilename.find(".ply") != std::string::npos) {
    // Generic Semantic PLY meshes have -Z gravity
    const quatf T_esp_scene =
        quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

    for (auto& xyz : semanticData->cpu_vbo_) {
      xyz = T_esp_scene * xyz;
    }
  }

  srcMeshData.indicesInto(semanticData->cpu_ibo_);

  // initialize per-vertex color array
  Cr::Containers::Array<Mn::Color3ub> meshColors{Mn::NoInit, numVerts};
  // build color array from colors present in mesh
  semanticData->buildMeshColors(srcMeshData, convertToSRGB, meshColors);

  Cr::Utility::copy(meshColors,
                    Cr::Containers::arrayCast<Mn::Color3ub>(
                        Cr::Containers::arrayView(semanticData->cpu_cbo_)));

  // Check we actually have object IDs before copying them, and that those are
  // in a range we expect them to be
  Cr::Containers::Array<Mn::UnsignedInt> objectIds;

  // Whether or not the objectIds were provided by the source file. If so then
  // we assume they can be used to provide islands to split the semantic mesh
  // for better frustum culling.
  bool objIdsFromFile = false;

  // Whether or not region partitions are provided in the SSD file. If so then
  // we assume they can be used to provide islands to split the semantic mesh
  // for better frustum culling.
  bool objPartitionsFromSSD = false;

  // This is used to map every vertex to a particular partition, for culling.
  std::vector<uint16_t> partitionIds;

  if (srcMeshData.hasAttribute(Mn::Trade::MeshAttribute::ObjectId)) {
    objectIds = srcMeshData.objectIdsAsArray();
    objIdsFromFile = true;
    objPartitionsFromSSD = false;
    const int maxVal = Mn::Math::max(objectIds);
    ESP_CHECK(maxVal <= 65535, Cr::Utility::formatString(
                                   "{}Object IDs can't be stored into 16 bits "
                                   ": Max ID Value found in data : {}",
                                   msgPrefix, maxVal));
    colorMapToUse.assign(Mn::DebugTools::ColorMap::turbo().begin(),
                         Mn::DebugTools::ColorMap::turbo().end());
  } else {
    // If object IDs are lacking, use vertex color to infer objectIDs, where
    // each unique color corresponds to a unique objectID.
    // These ids should probably not be used to split the mesh into submeshes
    // for culling purposes, since there may be very many different vertex
    // colors.
    objIdsFromFile = false;

    // If a vertex color array and array of semantic ids are provided via a
    // semantic scene descriptor, use these to replace the idxs found in using
    // the removeDuplicatesInPlace process and their color mappings
    if ((semanticScene != nullptr) && semanticScene->hasVertColorsDefined()) {
      // built partion IDs from SSD regions, to split mesh for culling, instead
      // of objectIDs
      objPartitionsFromSSD = true;

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
      auto colorAsInt = [](const Mn::Color3ub& color) -> uint32_t {
        return (unsigned(color[0]) << 16) | (unsigned(color[1]) << 8) |
               unsigned(color[2]);
      };

      // build maps of color ints to semantic IDs and color ints to region ids
      // in SSD find max regions present, so we can have an
      // "unassigned"/"unknown" region to move all verts whose region is

      int maxRegion = -1;
      // build the color map with first maxSemanticID elements in proper order
      // to match provided semantic IDs (so that ID is IDX of semantic color in
      // map) and any overflow colors uniquely map 1-to-1 to an unmapped
      // semantic ID as their index.
      colorMapToUse.resize(numSSDObjs);
      for (int i = 0; i < numSSDObjs; ++i) {
        const auto& ssdObj =
            static_cast<scene::HM3DObjectInstance&>(*ssdObjs[i]);
        const Mn::Vector3ub ssdColor = ssdObj.getColor();
        const uint32_t colorInt = colorAsInt(ssdColor);
        int semanticID = ssdObj.semanticID();
        int regionIDX = ssdObj.region()->getIndex();
        tmpColorMapToSSDidAndRegionIndex[colorInt] = {semanticID, regionIDX};
        if (colorMapToUse.size() <= semanticID) {
          colorMapToUse.resize(semanticID + 1);
        }
        colorMapToUse[semanticID] = ssdColor;
        maxRegion = Mn::Math::max(regionIDX, maxRegion);
      }
      // largest possible known semanticID will be size of colorMapToUse at this
      // point.  If unknown/unmapped colors are present in mesh, colorMapToUse
      // will grow to hold them in subsequent step.
      int maxSemanticID = colorMapToUse.size();
      // increment maxRegion to use largest value as unknown region for objects
      // whose colors are not mapped in given ssd data.
      ++maxRegion;

      // (re)build mesh objectIDs vector (objectID per vertex) based on
      // per vertex colors mapped

      // 1st semantic ID for colors not found in SSD objects
      std::size_t nonSSDObjID = maxSemanticID + 1;

      // map regionIDs to affected verts using semantic color provided in
      // SemanticScene, as specified in SSD file.
      partitionIds.resize(numVerts);
      // map semantic IDs corresponding to colors from SSD file to appropriate
      // verts (via index)
      semanticData->objectIds_.resize(numVerts);
      // temporary holding structure to hold any non-SSD vert colors, so that
      // the nonSSDObjID for new colors can be incremented appropriately
      // not using set to avoid extra include. Key is color, value is semantic
      // ID assigned for unknown color
      std::unordered_map<uint32_t, int> nonSSDVertColors;

      // only go through all verts one time
      // derive semantic ID and color and region/room ID for culling
      int regionID = 0;
      int semanticID = 0;

      for (int vertIdx = 0; vertIdx < numVerts; ++vertIdx) {
        Mn::Color3ub meshColor = meshColors[vertIdx];
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

          // color for given semantic ID
          if (colorMapToUse.size() <= semanticID) {
            colorMapToUse.resize(semanticID + 1);
          }
          colorMapToUse[semanticID] = meshColor;
        }

        // semantic ID for vertex
        semanticData->objectIds_[vertIdx] = semanticID;
        // partition Ids for each vertex, for multi-mesh construction.
        partitionIds[vertIdx] = regionID;
      }  // for each vertex

      // FOR VERT-BASED OBB CALC
      // build semantic OBBs here
      semanticData->buildSemanticOBBs(
          semanticData->cpu_vbo_, semanticData->objectIds_, ssdObjs, msgPrefix);

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
      // per-vertex object IDs reflect per-vertex color assignments
      objectIds = std::move(out.first);
      auto clrMapView = colorsThatBecomeTheColorMap.prefix(out.second);
      colorMapToUse.assign(clrMapView.begin(), clrMapView.end());

      objPartitionsFromSSD = false;
    }
  }
  if (!objPartitionsFromSSD) {
    // if objPartitionsFromSSD then semanticData->objectIds were
    // populated directly
    Mn::Math::castInto(
        Cr::Containers::arrayCast<2, Mn::UnsignedInt>(
            Cr::Containers::stridedArrayView(objectIds)),
        Cr::Containers::arrayCast<2, Mn::UnsignedShort>(
            Cr::Containers::stridedArrayView(semanticData->objectIds_)));
  }

  // build output vector of meshdata unique pointers
  std::vector<GenericSemanticMeshData::uptr> splitMeshData;
  if (splitMesh && (objIdsFromFile || objPartitionsFromSSD)) {
    // use these ids to identify partitions that any particular vert should
    // belong to
    std::vector<uint16_t>& meshPartitionIds =
        objIdsFromFile ? semanticData->objectIds_ : partitionIds;

    std::unordered_map<uint16_t, PerPartitionIdMeshBuilder>
        partitionIdToObjectData;

    for (size_t i = 0; i < semanticData->cpu_ibo_.size(); ++i) {
      const uint32_t globalIndex = semanticData->cpu_ibo_[i];
      const uint16_t objectId = semanticData->objectIds_[globalIndex];
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
          .addVertex(globalIndex, semanticData->cpu_vbo_[globalIndex],
                     semanticData->cpu_cbo_[globalIndex], objectId);
    }
    // Update collision mesh data for each mesh
    for (size_t i = 0; i < splitMeshData.size(); ++i) {
      splitMeshData[i]->updateCollisionMeshData();
    }
  } else {
    // mesh should not be split - ids were synthesized

    // Store indices, facd_ids in Magnum MeshData3D format such that
    // later they can be accessed.
    // Note that normal and texture data are not stored
    semanticData->collisionMeshData_.primitive = Mn::MeshPrimitive::Triangles;
    semanticData->updateCollisionMeshData();
    splitMeshData.emplace_back(std::move(semanticData));
  }
  return splitMeshData;

}  // GenericSemanticMeshData::buildSemanticMeshData

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
  renderingBuffer_->mesh.setPrimitive(Mn::GL::MeshPrimitive::Triangles)
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

Mn::GL::Mesh* GenericSemanticMeshData::getMagnumGLMesh() {
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
