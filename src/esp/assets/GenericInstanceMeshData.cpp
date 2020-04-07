// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericInstanceMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/Algorithms.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/PackingBatch.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

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
};

Cr::Containers::Optional<InstancePlyData> parsePly(
    Mn::Trade::AbstractImporter& importer,
    const std::string& plyFile) {
  /* Open the file. On error the importer already prints a diagnostic message,
     so no need to do that here. The importer implicitly converts per-face
     attributes to per-vertex, so nothing extra needs to be done. */
  Cr::Containers::Optional<Mn::Trade::MeshData> meshData;
  if (!importer.openFile(plyFile) || !(meshData = importer.mesh(0)))
    return Cr::Containers::NullOpt;

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

  /* Assuming colors are 8-bit RGB to avoid expanding them to float and then
     packing back */
  if (!meshData->hasAttribute(Mn::Trade::MeshAttribute::Color)) {
    LOG(ERROR) << "File has no vertex colors";
    return Cr::Containers::NullOpt;
  }
  if (meshData->attributeFormat(Mn::Trade::MeshAttribute::Color) !=
      Mn::VertexFormat::Vector3ubNormalized) {
    // TODO: output the format enum directly once glog is gone and we use Debug
    LOG(ERROR) << "Unexpected vertex color type"
               << Mn::UnsignedInt(meshData->attributeFormat(
                      Mn::Trade::MeshAttribute::Color));
    return Cr::Containers::NullOpt;
  }
  data.cpu_cbo.resize(meshData->vertexCount());
  Cr::Utility::copy(
      meshData->attribute<Mn::Color3ub>(Mn::Trade::MeshAttribute::Color),
      Cr::Containers::arrayCast<Mn::Color3ub>(
          Cr::Containers::arrayView(data.cpu_cbo)));

  /* Check we actually have object IDs before copying them, and that those are
     in a range we expect them to be */
  if (!meshData->hasAttribute(Mn::Trade::MeshAttribute::ObjectId)) {
    LOG(ERROR) << "File has no object IDs";
    return Cr::Containers::NullOpt;
  }
  Cr::Containers::Array<Mn::UnsignedInt> objectIds =
      meshData->objectIdsAsArray();
  if (Mn::Math::max(objectIds) > 65535) {
    LOG(ERROR) << "Object IDs can't fit into 16 bits";
    return Cr::Containers::NullOpt;
  }
  data.objectIds.resize(meshData->vertexCount());
  Mn::Math::castInto(Cr::Containers::arrayCast<2, Mn::UnsignedInt>(
                         Cr::Containers::stridedArrayView(objectIds)),
                     Cr::Containers::arrayCast<2, Mn::UnsignedShort>(
                         Cr::Containers::stridedArrayView(data.objectIds)));

  // Generic Semantic PLY meshes have -Z gravity
  const quatf T_esp_scene =
      quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

  for (auto& xyz : data.cpu_vbo) {
    xyz = T_esp_scene * xyz;
  }
  return data;
}

}  // namespace

std::vector<std::unique_ptr<GenericInstanceMeshData>>
GenericInstanceMeshData::fromPlySplitByObjectId(
    Mn::Trade::AbstractImporter& importer,
    const std::string& plyFile) {
  Cr::Containers::Optional<InstancePlyData> parseResult =
      parsePly(importer, plyFile);
  if (!parseResult) {
    return {};
  }
  const InstancePlyData& data = *parseResult;

  std::vector<GenericInstanceMeshData::uptr> splitMeshData;
  std::unordered_map<uint16_t, PerObjectIdMeshBuilder> objectIdToObjectData;

  for (size_t i = 0; i < data.cpu_ibo.size(); ++i) {
    const uint32_t globalIndex = data.cpu_ibo[i];
    const uint16_t objectId = data.objectIds[globalIndex];
    if (objectIdToObjectData.find(objectId) == objectIdToObjectData.end()) {
      auto instanceMesh = GenericInstanceMeshData::create_unique();
      objectIdToObjectData.emplace(
          objectId, PerObjectIdMeshBuilder{*instanceMesh, objectId});
      splitMeshData.emplace_back(std::move(instanceMesh));
    }
    objectIdToObjectData.at(objectId).addVertex(
        globalIndex, data.cpu_vbo[globalIndex], data.cpu_cbo[globalIndex]);
  }
  return splitMeshData;
}

std::unique_ptr<GenericInstanceMeshData> GenericInstanceMeshData::fromPLY(
    Mn::Trade::AbstractImporter& importer,
    const std::string& plyFile) {
  Cr::Containers::Optional<InstancePlyData> parseResult =
      parsePly(importer, plyFile);
  if (!parseResult) {
    return nullptr;
  }

  auto data = GenericInstanceMeshData::create_unique();
  data->cpu_vbo_ = std::move(parseResult->cpu_vbo);
  data->cpu_cbo_ = std::move(parseResult->cpu_cbo);
  data->cpu_ibo_ = std::move(parseResult->cpu_ibo);
  data->objectIds_ = std::move(parseResult->objectIds);

  // Construct vertices for collsion meshData
  // Store indices, facd_ids in Magnum MeshData3D format such that
  // later they can be accessed.
  // Note that normal and texture data are not stored
  data->collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;
  data->updateCollisionMeshData();
  return data;
}

void GenericInstanceMeshData::uploadBuffersToGPU(bool forceReload) {
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
      std::make_unique<GenericInstanceMeshData::RenderingBuffer>();
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(cpu_ibo_.size())
      .addVertexBuffer(
          std::move(vertices), 0, Mn::Shaders::Generic3D::Position{},
          Mn::Shaders::Generic3D::Color3{
              Mn::Shaders::Generic3D::Color3::DataType::UnsignedByte,
              Mn::Shaders::Generic3D::Color3::DataOption::Normalized},
          1,
          Mn::Shaders::Generic3D::ObjectId{
              Mn::Shaders::Generic3D::ObjectId::DataType::UnsignedShort},
          2)
      .setIndexBuffer(std::move(indices), 0,
                      Mn::GL::MeshIndexType::UnsignedInt);

  updateCollisionMeshData();

  buffersOnGPU_ = true;
}

Magnum::GL::Mesh* GenericInstanceMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

void GenericInstanceMeshData::updateCollisionMeshData() {
  collisionMeshData_.positions = Cr::Containers::arrayCast<Mn::Vector3>(
      Cr::Containers::arrayView(cpu_vbo_));
  collisionMeshData_.indices = Cr::Containers::arrayCast<Mn::UnsignedInt>(
      Cr::Containers::arrayView(cpu_ibo_));
}

void GenericInstanceMeshData::PerObjectIdMeshBuilder::addVertex(
    uint32_t vertexId,
    const vec3f& position,
    const vec3uc& color) {
  // if we haven't seen this vertex, add it to the local vertex/color buffer
  auto result = vertexIdToVertexIndex_.emplace(vertexId, data_.cpu_vbo_.size());
  if (result.second) {
    data_.cpu_vbo_.emplace_back(position);
    data_.cpu_cbo_.emplace_back(color);
    data_.objectIds_.emplace_back(objectId_);
  }

  // update index buffers with local index of vertex/color
  data_.cpu_ibo_.emplace_back(result.first->second);
}

}  // namespace assets
}  // namespace esp
