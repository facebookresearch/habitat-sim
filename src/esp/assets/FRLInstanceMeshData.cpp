// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FRLInstanceMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
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

namespace esp {
namespace assets {

bool FRLInstanceMeshData::loadPLY(const std::string& ply_file) {
  std::ifstream ifs(ply_file, std::ios::in);
  if (!ifs.good()) {
    return false;
  }

  /* Read the header and make sure it is what we expect */
  std::string line, token;
  std::istringstream iss;
  std::getline(ifs, line);
  if (line != "ply") {
    return false;
  }

  std::getline(ifs, line);
  if (line != "comment etw-instance-mesh-format v1") {
    return false;
  }

  // Trash format line, element mappings, and the two mappings lines
  for (int i = 0; i < 4; ++i) {
    std::getline(ifs, line);
  }

  std::getline(ifs, line);
  iss.str(line);
  iss >> token;
  if (token != "element") {
    return false;
  }
  int nVertex;
  iss >> token >> nVertex;

  // For now, just trash the header because I am lazy and I know what it is
  while (true) {
    std::getline(ifs, line);
    if (line == "end_header") {
      break;
    }
  }

  int num_instances;
  ifs.read(reinterpret_cast<char*>(&num_instances), sizeof(num_instances));
  id_to_node.resize(num_instances);
  ifs.read(reinterpret_cast<char*>(id_to_node.data()),
           num_instances * sizeof(int));

  ifs.read(reinterpret_cast<char*>(&num_instances), sizeof(num_instances));
  id_to_label.resize(num_instances);
  ifs.read(reinterpret_cast<char*>(id_to_label.data()),
           num_instances * sizeof(int));

  cpu_cbo_.clear();
  cpu_cbo_.reserve(nVertex);
  cpu_vbo_.clear();
  cpu_vbo_.reserve(nVertex);

  for (int i = 0; i < nVertex; ++i) {
    vec3f xyz;
    vec3uc rgb;

    ifs.read(reinterpret_cast<char*>(xyz.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(rgb.data()), 3 * sizeof(uint8_t));

    int instance_id;
    ifs.read(reinterpret_cast<char*>(&instance_id), sizeof(instance_id));

    cpu_cbo_.emplace_back(rgb);

    vec4f xyzid;
    xyzid.head<3>() = xyz;
    xyzid[3] = static_cast<float>(instance_id);

    cpu_vbo_.emplace_back(xyzid);
  }

  int grav_size;
  ifs.read(reinterpret_cast<char*>(&grav_size), sizeof(grav_size));
  ifs.read(reinterpret_cast<char*>(gravity_dir.data()),
           sizeof(float) * grav_size);

  this->orig_gravity_dir = this->gravity_dir;

  const Sophus::SO3f T_esp_scene(
      quatf::FromTwoVectors(this->gravity_dir, geo::ESP_GRAVITY));

  this->gravity_dir = esp::geo::ESP_GRAVITY;
  for (auto& xyzid : this->cpu_vbo_) {
    const vec3f xyz_scene = xyzid.head<3>();
    const vec3f xyz_esp = T_esp_scene * xyz_scene;

    xyzid.head<3>() = xyz_esp;
  }

  // Store vertex buffer without instance id
  cpu_vbo_3_ = new std::vector<vec3f>(cpu_vbo_.size());
  for (int i = 0; i < cpu_vbo_.size(); ++i) {
    (*cpu_vbo_3_)[i] = cpu_vbo_[i].head<3>();
  }

  // create ibo converting quads to tris [0, 1, 2, 3] -> [0, 1, 2],[0,2,3]
  const size_t numQuads = cpu_vbo_.size() / 4;
  tri_ibo_ = new std::vector<uint32_t>(numQuads * 6);
  for (uint32_t iQuad = 0; iQuad < numQuads; ++iQuad) {
    const uint32_t triIdx = 6 * iQuad;
    const uint32_t quadIdx = 4 * iQuad;
    (*tri_ibo_)[triIdx + 0] = quadIdx + 0;
    (*tri_ibo_)[triIdx + 1] = quadIdx + 1;
    (*tri_ibo_)[triIdx + 2] = quadIdx + 2;
    (*tri_ibo_)[triIdx + 3] = quadIdx + 0;
    (*tri_ibo_)[triIdx + 4] = quadIdx + 2;
    (*tri_ibo_)[triIdx + 5] = quadIdx + 3;
  }

  // Construct collision meshData
  collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;
  collisionMeshData_.positions =
      Corrade::Containers::arrayCast<Magnum::Vector3>(
          Corrade::Containers::arrayView(cpu_vbo_3_->data(),
                                         cpu_vbo_3_->size()));
  collisionMeshData_.indices =
      Corrade::Containers::arrayCast<Magnum::UnsignedInt>(
          Corrade::Containers::arrayView(tri_ibo_->data(), tri_ibo_->size()));

  return true;
}

void FRLInstanceMeshData::to_ply(const std::string& ply_file) const {
  const int nVertex = cpu_vbo_.size();

  std::ofstream f(ply_file, std::ios::out | std::ios::binary);
  f << "ply" << std::endl;
  // Mark this as my own format as its kinda weird
  f << "comment etw-instance-mesh-format v1" << std::endl;

  f << "format binary_little_endian 1.0" << std::endl;

  // Let's stick these mappings into the ply file as lists
  f << "element mappings 2" << std::endl;
  f << "property list int int id_to_node" << std::endl;
  f << "property list int int id_to_label" << std::endl;

  // Here is a normal mesh.  That ID feild is different tho...
  f << "element vertex " << nVertex << std::endl;
  f << "property float x" << std::endl;
  f << "property float y" << std::endl;
  f << "property float z" << std::endl;
  f << "property uchar red" << std::endl;
  f << "property uchar green" << std::endl;
  f << "property uchar blue" << std::endl;
  f << "property int id" << std::endl;

  // Let's also stick gravity in the ply file while are at it!
  f << "element gravity 1" << std::endl;
  f << "property list int float gravity" << std::endl;

  f << "end_header" << std::endl;

  const int num_instances = id_to_node.size();
  f.write(reinterpret_cast<const char*>(&num_instances), sizeof(num_instances));
  f.write(reinterpret_cast<const char*>(id_to_node.data()),
          num_instances * sizeof(int));

  f.write(reinterpret_cast<const char*>(&num_instances), sizeof(num_instances));
  f.write(reinterpret_cast<const char*>(id_to_label.data()),
          num_instances * sizeof(int));

  for (int i = 0; i < nVertex; ++i) {
    vec3f xyz = cpu_vbo_[i].head<3>();
    auto& rgb = cpu_cbo_[i];

    f.write(reinterpret_cast<const char*>(xyz.data()), 3 * sizeof(float));
    f.write(reinterpret_cast<const char*>(rgb.data()), 3 * sizeof(uint8_t));

    const int instance_id = std::floor(cpu_vbo_[i][3] * id_to_node.size());
    f.write(reinterpret_cast<const char*>(&instance_id), sizeof(instance_id));
  }

  const int grav_size = 3;
  f.write(reinterpret_cast<const char*>(&grav_size), sizeof(grav_size));
  f.write(reinterpret_cast<const char*>(gravity_dir.data()),
          sizeof(float) * grav_size);
}

Magnum::GL::Mesh* FRLInstanceMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

void FRLInstanceMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ = std::make_unique<FRLInstanceMeshData::RenderingBuffer>();

  const size_t numQuads = cpu_vbo_.size() / 4;

  /* Extract object IDs from the fourth component -- it's originally a float
     so we have to allocate instead of using a strided array view :( */
  Cr::Containers::Array<uint32_t> objectIds{numQuads * 2};
  for (size_t i = 0; i < numQuads; ++i) {
    objectIds[2 * i] = cpu_vbo_[4 * i][3];
    objectIds[2 * i + 1] = cpu_vbo_[4 * i][3];
  }
  renderingBuffer_->tex = createInstanceTexture(objectIds);

  renderingBuffer_->vbo.setData(*cpu_vbo_3_,
                                Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cpu_cbo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(*tri_ibo_, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(tri_ibo_->size())  // Set vertex/index count (numQuads * 6)
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       gfx::PrimitiveIDTexturedShader::Position{})
      .addVertexBuffer(
          renderingBuffer_->cbo, 0,
          gfx::PrimitiveIDTexturedShader::Color3{
              gfx::PrimitiveIDTexturedShader::Color3::DataType::UnsignedByte})
      .setIndexBuffer(renderingBuffer_->ibo, 0,
                      Magnum::GL::MeshIndexType::UnsignedInt);

  buffersOnGPU_ = true;
}

}  // namespace assets
}  // namespace esp
