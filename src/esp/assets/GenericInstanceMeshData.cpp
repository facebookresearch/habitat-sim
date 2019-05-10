// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericInstanceMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/Trade.h>

#include <tinyply.h>
#include <fstream>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <sophus/so3.hpp>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/io/io.h"
#include "esp/io/json.h"

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

/*
 * In modern OpenGL fragment sharder, we can access the ID of the current
 * primitive and thus can index an array.  We can make a 2D texture behave
 * like a 2D array with nearest sampling and edge clamping.
 */
Magnum::GL::Texture2D createInstanceTexture(float* data, const int texSize) {
  /*
   * Create the image that wil be uploaded to the texture.  We
   * can index the original data array
   * with image[primId / texSize, primId % texSize]
   * NB: The indices will have to mapped into [0, 1] in the GLSL code
   */
  Magnum::Image2D image(
      Magnum::PixelFormat::R32F, {texSize, texSize},
      Corrade::Containers::Array<char>(reinterpret_cast<char*>(data),
                                       texSize * texSize * sizeof(data[0])));

  Magnum::GL::Texture2D tex;

  tex.setMinificationFilter(Magnum::SamplerFilter::Nearest)
      .setMagnificationFilter(Magnum::SamplerFilter::Nearest)
      .setWrapping({Magnum::SamplerWrapping::ClampToEdge,
                    Magnum::SamplerWrapping::ClampToEdge})
      .setStorage(1, Magnum::GL::TextureFormat::R32F, image.size())
      .setSubImage(0, {}, image);

  return tex;
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
  std::vector<vec3f> cbo_float;
  cbo_float.reserve(cpu_cbo_.size());
  for (const auto& c : cpu_cbo_) {
    cbo_float.emplace_back(c.cast<float>() / 255.0f);
  }

  /*
   * Textures in OpenGL must be square and must have a sizes that are powers of
   * 2, so first compute the size of the smallest texture that can contain our
   * array.  Then copy the object id's buffer into the texture data buffer as
   * floats.
   */
  const int nPrims = cpu_ibo_.size();
  const int texSize = std::pow(2, std::ceil(std::log2(std::sqrt(nPrims))));
  // The image takes ownership over the obj_id_tex_data array, so there is no
  // delete
  float* obj_id_tex_data = new float[texSize * texSize]();
  for (size_t i = 0; i < nPrims; ++i) {
    obj_id_tex_data[i] = static_cast<float>(objectIds_[i]);
  }

  // Takes ownership of the data pointer
  renderingBuffer_->tex = createInstanceTexture(obj_id_tex_data, texSize);

  renderingBuffer_->vbo.setData(cpu_vbo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cbo_float, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(cpu_ibo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(nPrims * 3)
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       Magnum::GL::Attribute<0, Magnum::Vector3>{})
      .addVertexBuffer(renderingBuffer_->cbo, 0,
                       Magnum::GL::Attribute<1, Magnum::Color3>{})
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
