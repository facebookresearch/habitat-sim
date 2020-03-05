// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericInstanceMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/MeshTools/CombineIndexedArrays.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/RemoveDuplicates.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/Trade.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <tinyply.h>
#include <unistd.h>
#include <fstream>
#include <sophus/so3.hpp>
#include <sstream>
#include <vector>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/gfx/PrimitiveIDShader.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

namespace {
template <typename T>
void copyTo(std::shared_ptr<tinyply::PlyData> data, std::vector<T>& dst) {
  dst.resize(data->count);
  CHECK_EQ(data->buffer.size_bytes(), sizeof(T) * dst.size());
  std::memcpy(dst.data(), data->buffer.get(), data->buffer.size_bytes());
}

template <typename T>
void copyVec3To(std::shared_ptr<tinyply::PlyData> data, std::vector<T>& dst) {
  dst.resize(data->count * 3);
  CHECK_EQ(data->buffer.size_bytes(), sizeof(T) * dst.size());
  std::memcpy(dst.data(), data->buffer.get(), data->buffer.size_bytes());
}

// TODO(msb) move this specialization to Magnum/MeshTools/CombineIndexedArrays.h
template <class T,
          typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
std::vector<Mn::UnsignedInt> removeDuplicates(std::vector<T>& data) {
  std::vector<Mn::UnsignedInt> resultIndices;
  resultIndices.reserve(data.size());

  std::unordered_map<T, Mn::UnsignedInt> table;
  size_t cursor = 0;
  for (size_t i = 0; i < data.size(); ++i) {
    auto val = data[i];
    auto result = table.emplace(val, cursor);
    if (result.second) {
      data[cursor++] = val;
    }
    resultIndices.push_back(result.first->second);
  }
  data.resize(cursor);

  return resultIndices;
}

struct InstancePlyData {
  std::vector<vec3f> cpu_vbo;
  std::vector<vec3uc> cpu_cbo;
  std::vector<uint32_t> cpu_ibo;
  std::vector<uint16_t> objectIds;
};

Cr::Containers::Optional<InstancePlyData> parsePly(const std::string& plyFile) {
  std::ifstream ifs(plyFile, std::ios::binary);
  if (!ifs.good()) {
    LOG(ERROR) << "Cannot open file at " << plyFile;
    return Cr::Containers::NullOpt;
  }

  tinyply::PlyFile file;
  std::shared_ptr<tinyply::PlyData> vertices, colors, faceVertexIndices,
      objectIds;

  try {
    if (!file.parse_header(ifs)) {
      LOG(ERROR) << "Could not read header";
      return Cr::Containers::NullOpt;
    }

    vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
    colors = file.request_properties_from_element("vertex",
                                                  {"red", "green", "blue"});
    faceVertexIndices =
        file.request_properties_from_element("face", {"vertex_indices"}, 0);
    objectIds = file.request_properties_from_element("face", {"object_id"});
  } catch (const std::exception& e) {
    LOG(ERROR) << "tinyply exception: " << e.what();
    return Cr::Containers::NullOpt;
  }

  file.read(ifs);
  InstancePlyData data{};

  CHECK(vertices->t == tinyply::Type::FLOAT32) << "x,y,z must be floats";
  // copyTo is a helper function to get stuff out of tinyply's format into our
  // format it does the resize and checks to make sure everything will fit
  copyTo(vertices, data.cpu_vbo);
  CHECK(colors->t == tinyply::Type::UINT8) << "r,g,b must be uint8_t's";
  copyTo(colors, data.cpu_cbo);

  int vertexPerFace;
  // We can load int32 index buffers as uint32 index buffers as they are simply
  // indices into an array and thus can't be negative. int32 is really uint31
  // and can be safely reinterpret_casted to uint32
  if (faceVertexIndices->t == tinyply::Type::INT32 ||
      faceVertexIndices->t == tinyply::Type::UINT32) {
    // We can figure out the number of vertices per face by dividing the number
    // of bytes needed to store the faces by the number of faces and the size of
    // each vertex
    vertexPerFace = faceVertexIndices->buffer.size_bytes() /
                    (faceVertexIndices->count * sizeof(uint32_t));
    if (vertexPerFace == 3) {
      copyVec3To(faceVertexIndices, data.cpu_ibo);
    } else if (vertexPerFace == 4) {
      std::vector<vec4ui> tmp;
      copyTo(faceVertexIndices, tmp);
      data.cpu_ibo.reserve(tmp.size() * 2 * 3);
      // create ibo converting quads to tris [0, 1, 2, 3] -> [0, 1, 2],[0, 2, 3]
      for (auto& quad : tmp) {
        constexpr int indices[] = {0, 1, 2, 0, 2, 3};
        for (int i : indices) {
          data.cpu_ibo.push_back(quad[i]);
        }
      }
    } else {
      LOG(ERROR) << "GenericInstanceMeshData: Only triangle or quadmeshs "
                    "supported!";
      return Cr::Containers::NullOpt;
    }
  } else {
    LOG(ERROR) << "Cannot load vertex indices of type "
               << tinyply::PropertyTable[faceVertexIndices->t].str;
    return Cr::Containers::NullOpt;
  }

  VLOG(1) << "vertexPerFace=" << vertexPerFace;

  // If the input mesh is a quad mesh, then we had to double the number of
  // primitives, so we also need to double the size of the object IDs buffer
  int indicesPerFace = 3 * (vertexPerFace == 4 ? 2 : 1);
  data.objectIds.reserve(objectIds->count * indicesPerFace);
  if (objectIds->t == tinyply::Type::INT32 ||
      objectIds->t == tinyply::Type::UINT32) {
    std::vector<int> tmp;
    // This copy will be safe because for 0 <= v <= 2^31 - 1, uint32 and
    // int32 have the same bit patterns.  We currently assume IDs are <= 2^16 -
    // 1, so this assumption is safe.
    copyTo(objectIds, tmp);

    for (auto& id : tmp) {
      // >= 0 to make sure we didn't overflow the int32 with the uint32
      CORRADE_INTERNAL_ASSERT(id >= 0 && id <= ((2 << 16) - 1));
      for (int i = 0; i < indicesPerFace; ++i)
        data.objectIds.push_back(id);
    }
  } else if (objectIds->t == tinyply::Type::UINT16) {
    std::vector<uint16_t> tmp;
    copyTo(objectIds, tmp);

    for (auto& id : tmp) {
      for (int i = 0; i < indicesPerFace; ++i)
        data.objectIds.push_back(id);
    }
  } else {
    LOG(ERROR) << "Cannot load object_id of type "
               << tinyply::PropertyTable[objectIds->t].str;
    return Cr::Containers::NullOpt;
  }

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
GenericInstanceMeshData::fromPlySplitByObjectId(const std::string& plyFile) {
  Cr::Containers::Optional<InstancePlyData> parseResult = parsePly(plyFile);
  if (!parseResult) {
    return {};
  }
  const InstancePlyData& data = *parseResult;

  std::vector<GenericInstanceMeshData::uptr> splitMeshData;
  std::unordered_map<uint16_t, PerObjectIdMeshBuilder> objectIdToObjectData;

  for (size_t i = 0; i < data.objectIds.size(); ++i) {
    const uint16_t objectId = data.objectIds[i];
    const uint32_t globalIndex = data.cpu_ibo[i];
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
    const std::string& plyFile) {
  Cr::Containers::Optional<InstancePlyData> parseResult = parsePly(plyFile);
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

  // TODO(msb) This processing could all be done off-line allowing us to
  // simply mmap a file with the vertex buffer and index buffer.
  std::vector<Mn::UnsignedInt> objectIdIndices = removeDuplicates(objectIds_);
  Mn::GL::Buffer vertices, indices;

  std::vector<Magnum::UnsignedInt> new_indices =
      Mn::MeshTools::combineIndexedArrays(
          std::make_pair(std::cref(objectIdIndices), std::ref(objectIds_)),
          std::make_pair(std::cref(cpu_ibo_), std::ref(cpu_vbo_)),
          std::make_pair(std::cref(cpu_ibo_), std::ref(cpu_cbo_)));

  indices.setTargetHint(Mn::GL::Buffer::TargetHint::ElementArray);
  indices.setData(new_indices, Mn::GL::BufferUsage::StaticDraw);

  vertices.setData(
      Mn::MeshTools::interleave(cpu_vbo_, cpu_cbo_, 1, objectIds_, 2),
      Mn::GL::BufferUsage::StaticDraw);

  renderingBuffer_ =
      std::make_unique<GenericInstanceMeshData::RenderingBuffer>();
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(cpu_ibo_.size())
      .addVertexBuffer(
          std::move(vertices), 0, gfx::PrimitiveIDShader::Position{},
          gfx::PrimitiveIDShader::Color3{
              gfx::PrimitiveIDShader::Color3::DataType::UnsignedByte,
              gfx::PrimitiveIDShader::Color3::DataOption::Normalized},
          1,
          gfx::PrimitiveIDShader::ObjectId{
              gfx::PrimitiveIDShader::ObjectId::DataType::UnsignedShort},
          2)
      .setIndexBuffer(std::move(indices), 0,
                      Mn::GL::MeshIndexType::UnsignedInt);

  cpu_ibo_ = std::move(new_indices);

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
  }

  // update index buffers with local index of vertex/color
  data_.cpu_ibo_.emplace_back(result.first->second);
  data_.objectIds_.emplace_back(objectId_);
}

}  // namespace assets
}  // namespace esp
