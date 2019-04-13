// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Mp3dInstanceMeshData.h"

#include <fstream>
#include <sstream>
#include <vector>

#include <sophus/so3.hpp>

#include <Corrade/Containers/Array.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/Trade.h>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/io/io.h"

namespace esp {
namespace assets {

bool Mp3dInstanceMeshData::loadMp3dPLY(const std::string& plyFile) {
  std::ifstream ifs(plyFile);
  if (!ifs.good()) {
    LOG(ERROR) << "Cannot open file at " << plyFile;
    return false;
  }

  std::string line, token;
  std::istringstream iss;
  std::getline(ifs, line);
  if (line != "ply") {
    LOG(ERROR) << "Invalid ply file header";
    return false;
  }
  std::getline(ifs, line);
  if (line != "format binary_little_endian 1.0") {
    LOG(ERROR) << "Invalid ply file header";
    return false;
  }

  // element vertex nVertex
  std::getline(ifs, line);
  iss.str(line);
  iss >> token;
  if (token != "element") {
    LOG(ERROR) << "Invalid element vertex header line";
    return false;
  }
  int nVertex;
  iss >> token >> nVertex;

  // we know the header is fixed so skip until face count
  do {
    std::getline(ifs, line);
  } while ((line.substr(0, 12) != "element face") && !ifs.eof());
  std::stringstream iss2;
  iss2.str(line);
  iss2 >> token;
  if (token != "element") {
    LOG(ERROR) << "Invalid element face header line";
    return false;
  }
  int nFace;
  iss2 >> token >> nFace;

  // ignore rest of header
  do {
    std::getline(ifs, line);
  } while ((line != "end_header") && !ifs.eof());

  cpu_cbo_.clear();
  cpu_cbo_.reserve(nVertex);
  cpu_vbo_.clear();
  cpu_vbo_.reserve(nVertex);
  cpu_ibo_.clear();
  cpu_ibo_.reserve(nFace);

  for (int i = 0; i < nVertex; ++i) {
    vec3f position;
    vec3f normal;
    vec2f texCoords;
    vec3uc rgb;

    ifs.read(reinterpret_cast<char*>(position.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(normal.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(texCoords.data()), 2 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(rgb.data()), 3 * sizeof(uint8_t));
    cpu_vbo_.emplace_back(position);
    cpu_cbo_.emplace_back(rgb);
  }

  for (int i = 0; i < nFace; ++i) {
    uint8_t nIndices;
    vec3i indices;
    int32_t materialId;
    int32_t segmentId;
    int32_t categoryId;

    ifs.read(reinterpret_cast<char*>(&nIndices), sizeof(nIndices));
    ASSERT(nIndices == 3);
    ifs.read(reinterpret_cast<char*>(indices.data()), 3 * sizeof(int));
    ifs.read(reinterpret_cast<char*>(&materialId), sizeof(materialId));
    ifs.read(reinterpret_cast<char*>(&segmentId), sizeof(segmentId));
    ifs.read(reinterpret_cast<char*>(&categoryId), sizeof(categoryId));
    cpu_ibo_.emplace_back(indices);
    materialIds_.emplace_back(materialId);
    segmentIds_.emplace_back(segmentId);
    categoryIds_.emplace_back(categoryId);
  }

  return true;
}

Magnum::GL::Mesh* Mp3dInstanceMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

void Mp3dInstanceMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ = std::make_unique<Mp3dInstanceMeshData::RenderingBuffer>();

  // create uint32 ibo
  const size_t nTris = cpu_ibo_.size();
  std::vector<uint32_t> tri_ibo(nTris * 3);
  for (int iTri = 0; iTri < nTris; ++iTri) {
    const int iBase = 3 * iTri;
    const vec3i& indices = cpu_ibo_[iTri];
    tri_ibo[iBase + 0] = static_cast<uint32_t>(indices[0]);
    tri_ibo[iBase + 1] = static_cast<uint32_t>(indices[1]);
    tri_ibo[iBase + 2] = static_cast<uint32_t>(indices[2]);
  }

  // convert uchar rgb to float rgb
  std::vector<float> cbo_float(cpu_cbo_.size() * 3);
  for (int iVert = 0; iVert < cpu_cbo_.size(); ++iVert) {
    const uint32_t idx = 3 * iVert;
    cbo_float[idx + 0] = cpu_cbo_[iVert][0] / 255.0f;
    cbo_float[idx + 1] = cpu_cbo_[iVert][1] / 255.0f;
    cbo_float[idx + 2] = cpu_cbo_[iVert][2] / 255.0f;
  }

  /* uint32_t max_obj_id = 0;
  for (auto& id : objectIds_) {
    max_obj_id = std::max(id, max_obj_id);
  }
  auto bgr_walk = [max_obj_id](float id) {
    const float r = id / static_cast<float>(max_obj_id);
    vec3f rgb = vec3f::Zero();
    if (r < 0.5) {
      rgb[1] = 2 * r;
      rgb[0] = 1.0 - rgb[1];
    } else {
      rgb[1] = 2 * (1.0 - r);
      rgb[2] = 1.0 - rgb[1];
    }
    return rgb;
  }; */

  const int texSize = std::pow(2, std::ceil(std::log2(std::sqrt(nTris))));
  float* obj_id_tex_data = new float[texSize * texSize]();
  for (size_t i = 0; i < nTris; ++i) {
    obj_id_tex_data[i] = objectIds_[i];
  }

  Magnum::Image2D image(Magnum::PixelFormat::R32F, {texSize, texSize},
                        Corrade::Containers::Array<char>(
                            reinterpret_cast<char*>(obj_id_tex_data),
                            texSize * texSize * sizeof(obj_id_tex_data[0])));

  renderingBuffer_->vbo.setData(cpu_vbo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cbo_float, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(tri_ibo, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(tri_ibo.size())
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       Magnum::GL::Attribute<0, Magnum::Vector3>{})
      .addVertexBuffer(renderingBuffer_->cbo, 0,
                       Magnum::GL::Attribute<1, Magnum::Color3>{})
      .setIndexBuffer(renderingBuffer_->ibo, 0,
                      Magnum::GL::MeshIndexType::UnsignedInt);

  renderingBuffer_->tex.setMinificationFilter(Magnum::SamplerFilter::Nearest)
      .setMagnificationFilter(Magnum::SamplerFilter::Nearest)
      .setStorage(1, Magnum::GL::TextureFormat::R32F, image.size())
      .setSubImage(0, {}, image);

  buffersOnGPU_ = true;
}

bool Mp3dInstanceMeshData::saveSemMeshPLY(
    const std::string& plyFile,
    const std::unordered_map<int, int>& segmentIdToObjectIdMap) {
  const int nVertex = cpu_vbo_.size();
  const int nFace = cpu_ibo_.size();

  std::ofstream f(plyFile, std::ios::out | std::ios::binary);
  f << "ply" << std::endl;
  f << "format binary_little_endian 1.0" << std::endl;
  f << "element vertex " << nVertex << std::endl;
  f << "property float x" << std::endl;
  f << "property float y" << std::endl;
  f << "property float z" << std::endl;
  f << "property uchar red" << std::endl;
  f << "property uchar green" << std::endl;
  f << "property uchar blue" << std::endl;
  f << "element face " << nFace << std::endl;
  f << "property list uchar int vertex_indices" << std::endl;
  f << "property int object_id" << std::endl;
  f << "end_header" << std::endl;

  for (int iVertex = 0; iVertex < nVertex; ++iVertex) {
    const vec3f& xyz = cpu_vbo_[iVertex].head<3>();
    const vec3uc& rgb = cpu_cbo_[iVertex];
    f.write(reinterpret_cast<const char*>(xyz.data()), 3 * sizeof(float));
    f.write(reinterpret_cast<const char*>(rgb.data()), 3 * sizeof(uint8_t));
  }

  for (int iFace = 0; iFace < cpu_ibo_.size(); ++iFace) {
    const uint8_t nIndices = 3;
    const vec3i& indices = cpu_ibo_[iFace];
    // The materialId corresponds to the segmentId from the .house file
    const int32_t segmentId = materialIds_[iFace];
    int32_t objectId = ID_UNDEFINED;
    if (segmentId >= 0) {
      objectId = segmentIdToObjectIdMap.at(segmentId);
    }
    f.write(reinterpret_cast<const char*>(&nIndices), sizeof(nIndices));
    f.write(reinterpret_cast<const char*>(indices.data()),
            3 * sizeof(uint32_t));
    f.write(reinterpret_cast<const char*>(&objectId), sizeof(objectId));
  }
  f.close();

  return true;
}

bool Mp3dInstanceMeshData::loadSemMeshPLY(const std::string& plyFile) {
  std::ifstream ifs(plyFile);
  if (!ifs.good()) {
    LOG(ERROR) << "Cannot open file at " << plyFile;
    return false;
  }

  std::string line, token;
  std::istringstream iss;
  std::getline(ifs, line);
  if (line != "ply") {
    LOG(ERROR) << "Invalid ply file header";
    return false;
  }
  std::getline(ifs, line);
  if (line != "format binary_little_endian 1.0") {
    LOG(ERROR) << "Invalid ply file header";
    return false;
  }

  // element vertex nVertex
  std::getline(ifs, line);
  iss.str(line);
  iss >> token;
  if (token != "element") {
    LOG(ERROR) << "Invalid element vertex header line";
    return false;
  }
  int nVertex;
  iss >> token >> nVertex;

  // we know the header is fixed so skip until face count
  do {
    std::getline(ifs, line);
  } while ((line.substr(0, 12) != "element face") && !ifs.eof());
  std::stringstream iss2;
  iss2.str(line);
  iss2 >> token;
  if (token != "element") {
    LOG(ERROR) << "Invalid element face header line";
    return false;
  }
  int nFace;
  iss2 >> token >> nFace;

  // ignore rest of header
  do {
    std::getline(ifs, line);
  } while ((line != "end_header") && !ifs.eof());

  cpu_cbo_.clear();
  cpu_cbo_.reserve(nVertex);
  cpu_vbo_.clear();
  cpu_vbo_.reserve(nVertex);
  cpu_ibo_.clear();
  cpu_ibo_.reserve(nFace);
  objectIds_.clear();
  objectIds_.reserve(nFace);

  for (int iVertex = 0; iVertex < nVertex; ++iVertex) {
    vec3f position;
    vec3uc rgb;

    ifs.read(reinterpret_cast<char*>(position.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(rgb.data()), 3 * sizeof(uint8_t));
    cpu_vbo_.emplace_back(position);
    cpu_cbo_.emplace_back(rgb);
  }

  for (int iFace = 0; iFace < nFace; ++iFace) {
    uint8_t nIndices;
    vec3i indices;
    int32_t objectId;
    ifs.read(reinterpret_cast<char*>(&nIndices), sizeof(nIndices));
    ASSERT(nIndices == 3);
    ifs.read(reinterpret_cast<char*>(indices.data()), 3 * sizeof(int));
    cpu_ibo_.emplace_back(indices);
    ifs.read(reinterpret_cast<char*>(&objectId), sizeof(objectId));
    objectIds_.emplace_back(objectId);

    // store objectId in position[3] of each vertex
    /* for (int iVertex : indices) {
      vec4f& position = cpu_vbo_[iVertex];
      position[3] = static_cast<float>(objectId);
    } */
  }

  // MP3D semantic PLY meshes have -Z gravity
  const quatf T_esp_scene =
      quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

  for (auto& xyzid : cpu_vbo_) {
    const vec3f xyz_scene = xyzid.head<3>();
    const vec3f xyz_esp = T_esp_scene * xyz_scene;
    xyzid.head<3>() = xyz_esp;
  }
  return true;
}

}  // namespace assets
}  // namespace esp
