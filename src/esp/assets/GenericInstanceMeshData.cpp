// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericInstanceMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/Trade.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <tinyply.h>
#include <unistd.h>
#include <fstream>
#include <sophus/so3.hpp>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/gfx/PrimitiveIDTexturedShader.h"
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
}  // namespace

Mn::GL::Texture2D createInstanceTexture(
    Cr::Containers::ArrayView<std::uint32_t> objectIds) {
  const Mn::Vector2i textureSize{
      gfx::PrimitiveIDTexturedShader::PrimitiveIDTextureWidth,
      int(objectIds.size() +
          gfx::PrimitiveIDTexturedShader::PrimitiveIDTextureWidth - 1) /
          gfx::PrimitiveIDTexturedShader::PrimitiveIDTextureWidth};
  Cr::Containers::Array<Mn::UnsignedShort> packedObjectIds{
      Cr::Containers::NoInit, std::size_t(textureSize.product())};
  for (std::size_t i = 0; i != objectIds.size(); ++i) {
    ASSERT(objectIds[i] <= 65535);
    packedObjectIds[i] = objectIds[i];
  }
  Mn::GL::Texture2D texture;
  texture.setMinificationFilter(Mn::SamplerFilter::Nearest)
      .setMagnificationFilter(Mn::SamplerFilter::Nearest)
      .setWrapping(
          {Mn::SamplerWrapping::ClampToEdge, Mn::SamplerWrapping::ClampToEdge})
      .setStorage(1, Mn::GL::TextureFormat::R16UI, textureSize)
      .setSubImage(0, {},
                   Mn::ImageView2D{Mn::PixelFormat::R16UI, textureSize,
                                   packedObjectIds});

  return texture;
}

bool GenericInstanceMeshData::loadPLY(const std::string& plyFile) {
  cpu_vbo_.clear();
  cpu_cbo_.clear();
  cpu_ibo_.clear();
  objectIds_.clear();

  std::ifstream ifs(plyFile, std::ios::binary);
  if (!ifs.good()) {
    LOG(ERROR) << "Cannot open file at " << plyFile;
    return false;
  }

  tinyply::PlyFile file;
  try {
    if (!file.parse_header(ifs)) {
      LOG(ERROR) << "Could not read header";
      return false;
    }
  } catch (const std::exception& e) {
    LOG(ERROR) << "Tinply error " << e.what();
    return false;
  }

  std::shared_ptr<tinyply::PlyData> vertices, colors, face_inds, object_ids;

  try {
    vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception& e) {
    LOG(ERROR) << "tinyply exception: " << e.what();
    return false;
  }

  try {
    colors = file.request_properties_from_element("vertex",
                                                  {"red", "green", "blue"});
  } catch (const std::exception& e) {
    LOG(ERROR) << "tinyply exception: " << e.what();
    return false;
  }

  try {
    face_inds =
        file.request_properties_from_element("face", {"vertex_indices"}, 0);
  } catch (const std::exception& e) {
    LOG(ERROR) << "tinyply exception: " << e.what();
    return false;
  }

  try {
    object_ids = file.request_properties_from_element("face", {"object_id"});
  } catch (const std::exception& e) {
    LOG(ERROR) << "tinyply exception: " << e.what();
    return false;
  }

  file.read(ifs);

  CHECK(vertices->t == tinyply::Type::FLOAT32) << "x,y,z must be floats";
  // copyTo is a helper function to get stuff out of tinyply's format into our
  // format it does the resize and checks to make sure everything will fit
  copyTo(vertices, cpu_vbo_);
  CHECK(colors->t == tinyply::Type::UINT8) << "r,g,b must be uint8_t's";
  copyTo(colors, cpu_cbo_);

  int vertexPerFace;
  // We can load int32 index buffers as uint32 index buffers as they are simply
  // indices into an array and thus can't be negative. int32 is really uint31
  // and can be safely reinterpret_casted to uint32
  if (face_inds->t == tinyply::Type::INT32 ||
      face_inds->t == tinyply::Type::UINT32) {
    // We can figure out the number of vertices per face by dividing the number
    // of bytes needed to store the faces by the number of faces and the size of
    // each vertex
    vertexPerFace =
        face_inds->buffer.size_bytes() / (face_inds->count * sizeof(uint32_t));
    if (vertexPerFace == 3) {
      copyTo(face_inds, cpu_ibo_);
    } else {
      std::vector<vec4ui> tmp;
      copyTo(face_inds, tmp);
      cpu_ibo_.reserve(tmp.size() * 2);
      // create ibo converting quads to tris [0, 1, 2, 3] -> [0, 1, 2],[0, 2, 3]
      for (auto& quad : tmp) {
        cpu_ibo_.emplace_back(quad[0], quad[1], quad[2]);
        cpu_ibo_.emplace_back(quad[0], quad[2], quad[3]);
      }
    }
  } else {
    LOG(ERROR) << "Cannot load vertex indices of type "
               << tinyply::PropertyTable[face_inds->t].str;
    return false;
  }

  VLOG(1) << "vertexPerFace=" << vertexPerFace;

  // If the input mesh is a quad mesh, then we had to double the number of
  // primitives, so we also need to double the size of the object IDs buffer
  objectIds_.reserve(object_ids->count * (vertexPerFace == 4 ? 2 : 1));
  if (object_ids->t == tinyply::Type::INT32) {
    std::vector<int> tmp;
    copyTo(object_ids, tmp);

    for (auto& id : tmp) {
      objectIds_.emplace_back(id);
      if (vertexPerFace == 4)
        objectIds_.emplace_back(id);
    }
  } else if (object_ids->t == tinyply::Type::UINT16) {
    std::vector<uint16_t> tmp;
    copyTo(object_ids, tmp);

    for (auto& id : tmp) {
      objectIds_.emplace_back(id);
      if (vertexPerFace == 4)
        objectIds_.emplace_back(id);
    }
  } else {
    LOG(ERROR) << "Cannot load object_id of type "
               << tinyply::PropertyTable[object_ids->t].str;
    return false;
  }

  // Generic Semantic PLY meshes have -Z gravity
  const quatf T_esp_scene =
      quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

  for (auto& xyz : cpu_vbo_) {
    xyz = T_esp_scene * xyz;
  }

  // Construct vertices for collsion meshData
  // Store indices, facd_ids in Magnum MeshData3D format such that
  // later they can be accessed.
  // Note that normal and texture data are not stored
  collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;
  collisionMeshData_.positions =
      Corrade::Containers::arrayCast<Magnum::Vector3>(
          Corrade::Containers::arrayView(cpu_vbo_.data(), cpu_vbo_.size()));
  collisionMeshData_.indices =
      Corrade::Containers::arrayCast<Magnum::UnsignedInt>(
          Corrade::Containers::arrayView(cpu_ibo_.data(), cpu_ibo_.size()));

  return true;
}

void GenericInstanceMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ =
      std::make_unique<GenericInstanceMeshData::RenderingBuffer>();

  // convert uchar rgb to float rgb
  std::vector<vec3f> cbo_float_;
  cbo_float_.reserve(cpu_cbo_.size());
  for (const auto& c : cpu_cbo_) {
    cbo_float_.emplace_back(c.cast<float>() / 255.0f);
  }

  /* Pack primitive IDs into a texture. 1D texture won't be large enough so the
     data have to be put into a 2D texture. For simplicity on both the C++ and
     shader side the texture has a fixed width and height is dynamic, and
     addressing is done as (gl_PrimitiveID % width, gl_PrimitiveID / width). */
  ASSERT(objectIds_.size() == cpu_ibo_.size());
  renderingBuffer_->tex = createInstanceTexture(objectIds_);
  renderingBuffer_->vbo.setData(cpu_vbo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cbo_float_,
                                Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(cpu_ibo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(cpu_ibo_.size() * 3)
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       gfx::PrimitiveIDTexturedShader::Position{})
      .addVertexBuffer(renderingBuffer_->cbo, 0,
                       gfx::PrimitiveIDTexturedShader::Color3{})
      .setIndexBuffer(renderingBuffer_->ibo, 0,
                      Magnum::GL::MeshIndexType::UnsignedInt);

  buffersOnGPU_ = true;
}

Magnum::GL::Mesh* GenericInstanceMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

}  // namespace assets
}  // namespace esp
