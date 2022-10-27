// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericSemanticMeshData.h"

#include <set>

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

#include "esp/geo/Geo.h"
#include "esp/scene/SemanticScene.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

std::unique_ptr<GenericSemanticMeshData>
GenericSemanticMeshData::buildSemanticMeshData(
    const Mn::Trade::MeshData& srcMeshData,
    const std::string& semanticFilename,
    std::vector<Mn::Vector3ub>& colorMapToUse,
    bool convertToSRGB,
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  // build text prefix used in log messages
  const std::string dbgMsgPrefix = Cr::Utility::formatString(
      "Parsing Semantic File {} w/prim:{} :", semanticFilename,
      static_cast<Mn::UnsignedInt>(srcMeshData.primitive()));

  // Check for required colors - all semantic meshes must have vertex colors
  ESP_CHECK(
      srcMeshData.hasAttribute(Mn::Trade::MeshAttribute::Color),
      dbgMsgPrefix << "has no vertex colors defined, which are required for "
                      "semantic meshes.");

  /* Copy attributes to the vectors. Positions and indices can be copied
     directly using the convenience APIs as we store them in the full type.
     The importer always provides an indexed mesh with positions, so no need
     for extra error checking. */

  auto semanticMeshData = GenericSemanticMeshData::create_unique();
  const auto numVerts = srcMeshData.vertexCount();

  semanticMeshData->cpu_ibo_.resize(srcMeshData.indexCount());
  semanticMeshData->cpu_vbo_.resize(numVerts);
  semanticMeshData->cpu_cbo_.resize(numVerts);
  semanticMeshData->objectIds_.resize(numVerts);

  srcMeshData.positions3DInto(Cr::Containers::arrayCast<Mn::Vector3>(
      Cr::Containers::arrayView(semanticMeshData->cpu_vbo_)));

  if (semanticFilename.find(".ply") != std::string::npos) {
    // Generic Semantic PLY meshes have -Z gravity
    const auto T_esp_scene =
        Mn::Quaternion{quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY)}
            .toMatrix();
    for (auto& xyz : semanticMeshData->cpu_vbo_) {
      xyz = T_esp_scene * xyz;
    }
  }

  srcMeshData.indicesInto(semanticMeshData->cpu_ibo_);

  // initialize per-vertex color array
  Cr::Containers::Array<Mn::Color3ub> meshColors{Mn::NoInit, numVerts};
  // build color array from colors present in mesh
  semanticMeshData->convertMeshColors(srcMeshData, convertToSRGB, meshColors);

  Cr::Utility::copy(meshColors,
                    Cr::Containers::arrayView(semanticMeshData->cpu_cbo_));

  // Check we actually have object IDs before copying them, and that those are
  // in a range we expect them to be
  Cr::Containers::Array<Mn::UnsignedInt> objectIds;

  // Whether or not the objectIds were provided by the source file. If so then
  // we assume they can be used to provide islands to split the semantic mesh
  // for better frustum culling.
  bool objIdsFromSrcMesh = false;

  // Whether or not region partitions are provided in the SSD file. If so then
  // we assume they can be used to provide islands to split the semantic mesh
  // for better frustum culling.
  semanticMeshData->meshUsesSSDPartitionIDs = false;

  // clear holding objects for debug date pertaining to mesh being built.
  semanticMeshData->nonSSDVertColorIDs.clear();
  semanticMeshData->nonSSDVertColorCounts.clear();

  if (srcMeshData.hasAttribute(Mn::Trade::MeshAttribute::ObjectId)) {
    // Per-mesh vertex semantic object ids are provided - this will override any
    // vertex colors provided in file
    objectIds = srcMeshData.objectIdsAsArray();
    objIdsFromSrcMesh = true;
    const int maxVal = Mn::Math::max(objectIds);
    ESP_CHECK(maxVal <= 65535, Cr::Utility::formatString(
                                   "{}Object IDs can't be stored into 16 bits "
                                   ": Max ID Value found in data : {}",
                                   dbgMsgPrefix, maxVal));

    // build colorMapToUse for meshes with pre-populated objectIDs, either using
    // provided vertex colors (NOTE : There must be 1-to-1 map between ID and
    // vert colors - if unsure do not use these) or a magnum color map

    semanticMeshData->buildColorMapToUse(objectIds, meshColors, false,
                                         colorMapToUse);
  } else {
    // No Object IDs defined - assign them based on colors at verts, if they
    // exist

    // If object IDs are lacking, use vertex color to infer objectIDs, where
    // each unique color corresponds to a unique objectID.
    // These ids should probably not be used to split the mesh into submeshes
    // for culling purposes, since there may be very many different vertex
    // colors.
    objIdsFromSrcMesh = false;

    // If a vertex color array and array of semantic ids are provided via a
    // semantic scene descriptor, use these to replace the idxs found in using
    // the removeDuplicatesInPlace process and their color mappings
    if (semanticScene && semanticScene->hasVertColorsDefined()) {
      // built partion IDs from SSD regions, to split mesh for culling, instead
      // of objectIDs
      semanticMeshData->meshUsesSSDPartitionIDs = true;

      // from SSD's objects_ array - this is the number of defined semantic
      // descriptors corresponding to object instances.
      // all this tapdancing because of the design of the Semantic Scene.
      const auto& ssdObjs = semanticScene->objects();
      std::size_t numSSDObjs = ssdObjs.size();

      // map of colors as ints as key, where value is objectID and regionID from
      // ssd
      const std::unordered_map<uint32_t, std::pair<int, int>>
          tmpColorMapToSSDidAndRegionIndex =
              semanticScene->getSemanticColorToIdAndRegionMap();

      // find max regions present, so we can have a specific region to move all
      // verts whose region/color is "unassigned"/"unknown"
      int maxRegion = -1;
      for (int i = 0; i < numSSDObjs; ++i) {
        maxRegion = Mn::Math::max(ssdObjs[i]->region()->getIndex(), maxRegion);
      }
      // If unknown/unmapped colors are present in mesh, colorMapToUse will grow
      // to hold them in subsequent step. 1st semantic ID for colors not found
      // in SSD objects ==> 1 more than the current highest id in the colormap
      std::size_t nonSSDObjID = colorMapToUse.size();

      // increment maxRegion to use as value for unknown region for objects
      // whose colors are not mapped in given ssd data.
      ++maxRegion;

      // (re)build mesh objectIDs vector (objectID per vertex) based on
      // per vertex colors mapped

      // map regionIDs to affected verts using semantic color provided in
      // SemanticScene, as specified in SSD file.
      semanticMeshData->partitionIds_.resize(numVerts);
      // map semantic IDs corresponding to colors from SSD file to appropriate
      // verts (via index)
      semanticMeshData->objectIds_.resize(numVerts);

      // only go through all verts one time
      // derive semantic ID and color and region/room ID for culling
      int regionID = 0;
      int semanticID = 0;

      for (int vertIdx = 0; vertIdx < numVerts; ++vertIdx) {
        Mn::Color3ub meshColor = meshColors[vertIdx];
        // Convert color to an int @ vertex
        const uint32_t meshColorInt = (unsigned(meshColor[0]) << 16) |
                                      (unsigned(meshColor[1]) << 8) |
                                      unsigned(meshColor[2]);

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
          // use currently assigned unknown color's semantic ID for this vertex
          semanticID = nonSSDObjID;
          regionID = maxRegion;

          // check if we've assigned a semantic ID to this color before, and if
          // not do so

          auto nonSSDClrCountRes =
              semanticMeshData->nonSSDVertColorCounts.insert({meshColorInt, 1});
          if (nonSSDClrCountRes.second) {
            // unknown color has not been seen before
            ESP_DEBUG() << dbgMsgPrefix << "Inserted Unknown semantic Color"
                        << meshColor << "in map w/ nonSSDObjID =" << semanticID;
            semanticMeshData->nonSSDVertColorIDs.insert(
                {meshColorInt, nonSSDObjID});
            // inserted, so increment nonSSDObjID tracking expanded semantic IDs
            // for future unknown color
            ++nonSSDObjID;
            // add color for given semantic ID to map
            if (colorMapToUse.size() <= semanticID) {
              colorMapToUse.resize(semanticID + 1);
            }
            colorMapToUse[semanticID] = meshColor;
          } else {
            ++nonSSDClrCountRes.first->second;
          }
        }

        // assign semantic ID for vertex
        semanticMeshData->objectIds_[vertIdx] = semanticID;
        // partition Ids for each vertex, for multi-mesh construction.
        semanticMeshData->partitionIds_[vertIdx] = regionID;
      }  // for each vertex

    } else {
      // Per vertex colors provided, but no semantic scene provided to provide
      // color->id mapping, or no viable mapping from color to semantic ID
      // provided within semantic scene so build a mapping based on colors at
      // vertices - first time we see a color gets first id, etc.

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
    }
  }
  if (!semanticMeshData->meshUsesSSDPartitionIDs) {
    // if meshUsesSSDPartitionIDs then semanticMeshData->objectIds were
    // populated directly while partition IDs were derived based on color
    // mapping
    Mn::Math::castInto(
        Cr::Containers::arrayCast<2, Mn::UnsignedInt>(
            Cr::Containers::stridedArrayView(objectIds)),
        Cr::Containers::arrayCast<2, Mn::UnsignedShort>(
            Cr::Containers::stridedArrayView(semanticMeshData->objectIds_)));
  }

  // Mesh has partition IDXs if they were provided as per-vert values or via SSD
  // color->region mappings
  semanticMeshData->meshHasPartitionIDXs =
      (objIdsFromSrcMesh || semanticMeshData->meshUsesSSDPartitionIDs);

  semanticMeshData->collisionMeshData_.primitive = Mn::MeshPrimitive::Triangles;
  semanticMeshData->updateCollisionMeshData();

  if (semanticScene && (semanticScene->buildBBoxFromVertColors())) {
    float fractionOfMaxBBoxSize = semanticScene->CCFractionToUseForBBox();

    if (fractionOfMaxBBoxSize > 0.0f) {
      // build adj list to use to derive CCs
      // Assumes that index buffer defines triangle polys in sequential groups
      // of 3 vert idxs
      const std::vector<std::set<uint32_t>> adjList = geo::buildAdjList(
          semanticMeshData->cpu_vbo_.size(), semanticMeshData->cpu_ibo_);

      // find all connected components based on adj list and vertex color.
      const std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>
          clrsToComponents =
              geo::findCCsByGivenColor(adjList, semanticMeshData->cpu_cbo_);

      // FOR VERT-BASED OBB CALC build semantic (actually AABBs currently)
      // only use CCs that have some fraction of largest CC's bbox volume.
      // Currently uses only max volume CC bbox for disjoint semantic regions.
      semanticMeshData->unMappedObjectIDXs =
          scene::SemanticScene::buildSemanticOBBsFromCCs(
              semanticMeshData->cpu_vbo_, clrsToComponents, semanticScene,
              fractionOfMaxBBoxSize, dbgMsgPrefix);
    } else {
      // FOR VERT-BASED OBB CALC build semantic (actually AABBs currently)
      // uses all vertex annotations, including disconnected components.
      semanticMeshData->unMappedObjectIDXs =
          scene::SemanticScene::buildSemanticOBBs(
              semanticMeshData->cpu_vbo_, semanticMeshData->objectIds_,
              semanticScene->objects(), dbgMsgPrefix);
    }
  }
  // display or save report denoting presence of semantic object-defined colors
  // in mesh
  return semanticMeshData;
}  // GenericSemanticMeshData::buildSemanticMeshData

std::vector<std::unique_ptr<GenericSemanticMeshData>>
GenericSemanticMeshData::partitionSemanticMeshData(
    const std::unique_ptr<GenericSemanticMeshData>& semanticMeshData) {
  // build output vector of meshdata unique pointers
  std::vector<GenericSemanticMeshData::uptr> splitMeshData;
  std::unordered_map<uint16_t, PerPartitionIdMeshBuilder>
      partitionIdToObjectData;
  const std::vector<uint16_t>& meshPartitionIds =
      semanticMeshData->getPartitionIDs();
  for (size_t i = 0; i < semanticMeshData->cpu_ibo_.size(); ++i) {
    const uint32_t globalIndex = semanticMeshData->cpu_ibo_[i];
    const uint16_t objectId = semanticMeshData->objectIds_[globalIndex];
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
        .addVertex(globalIndex, semanticMeshData->cpu_vbo_[globalIndex],
                   semanticMeshData->cpu_cbo_[globalIndex], objectId);
  }
  // Update collision mesh data for each mesh
  for (size_t i = 0; i < splitMeshData.size(); ++i) {
    splitMeshData[i]->updateCollisionMeshData();
  }
  return splitMeshData;

}  // GenericSemanticMeshData::partitionSemanticMeshData

std::vector<std::string> GenericSemanticMeshData::getVertColorSSDReport(
    const std::string& semanticFilename,
    const std::vector<Mn::Vector3ub>& colorMapToUse,
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  std::vector<std::string> results;
  results.emplace_back(Cr::Utility::formatString(
      "{} : Colors in mesh not found in semantic scene descriptor objects :",
      semanticFilename));
  // colors found on vertices not found in semantic lexicon :
  if (nonSSDVertColorIDs.empty()) {
    results.emplace_back("No unexpected colors found mapped to mesh verts.");
  } else {
    results.reserve(nonSSDVertColorIDs.size() + 1);
    results.emplace_back(Cr::Utility::formatString(
        "{} unexpected/unknown colors found mapped to mesh verts.",
        nonSSDVertColorIDs.size()));
    for (const std::pair<const uint32_t, int>& elem : nonSSDVertColorIDs) {
      results.emplace_back(Cr::Utility::formatString(
          "Color {} | # verts {} | applied Semantic ID {}.",
          geo::getColorAsString(colorMapToUse[elem.second]),
          nonSSDVertColorCounts.at(elem.first), elem.second));
    }
  }
  results.emplace_back(Cr::Utility::formatString(
      "{} : Colors from semantic objects not found on any mesh verts : ",
      semanticFilename));
  if (unMappedObjectIDXs.empty()) {
    results.emplace_back("All semantic object colors are found on mesh.");
    return results;
  }
  /// now process unMappedObjectIDXs
  // get all semantic objects
  const auto& ssdObjs = semanticScene->objects();
  for (int idx = 0; idx < unMappedObjectIDXs.size(); ++idx) {
    int objIdx = unMappedObjectIDXs[idx];
    // get object with given semantic ID
    auto& ssdObj = *ssdObjs[objIdx];
    results.emplace_back(Cr::Utility::formatString(
        "Semantic ID : {} : color : {} tag : {} | No verts have color for "
        "specified Semantic ID.",
        ssdObj.semanticID(), geo::getColorAsString(ssdObj.getColor()),
        ssdObj.id()));
  }
  return results;
}  // GenericSemanticMeshData::getVertColorSSDReport

std::unordered_map<uint32_t, std::vector<scene::CCSemanticObject::ptr>>
GenericSemanticMeshData::buildCCBasedSemanticObjs(
    const std::shared_ptr<scene::SemanticScene>& semanticScene) {
  // build adj list
  std::vector<std::set<uint32_t>> adjList =
      geo::buildAdjList(cpu_vbo_.size(), cpu_ibo_);
  // find all connected components based on vertex color.
  std::unordered_map<uint32_t, std::vector<std::set<uint32_t>>>
      clrsToComponents = geo::findCCsByGivenColor(adjList, cpu_cbo_);

  return scene::SemanticScene::buildCCBasedSemanticObjs(
      cpu_vbo_, clrsToComponents, semanticScene);
}  // GenericSemanticMeshData::buildCCBasedSemanticObjs

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
  collisionMeshData_.positions = Cr::Containers::arrayView(cpu_vbo_);
  collisionMeshData_.indices = Cr::Containers::arrayView(cpu_ibo_);
}

void GenericSemanticMeshData::PerPartitionIdMeshBuilder::addVertex(
    uint32_t vertexId,
    const Mn::Vector3& position,
    const Mn::Color3ub& color,
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
