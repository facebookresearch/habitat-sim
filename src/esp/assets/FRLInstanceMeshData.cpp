// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FRLInstanceMeshData.h"

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

  cpu_cbo.clear();
  cpu_cbo.reserve(nVertex);
  cpu_vbo.clear();
  cpu_vbo.reserve(nVertex);

  for (int i = 0; i < nVertex; ++i) {
    vec3f xyz;
    vec3uc rgb;

    ifs.read(reinterpret_cast<char*>(xyz.data()), 3 * sizeof(float));
    ifs.read(reinterpret_cast<char*>(rgb.data()), 3 * sizeof(uint8_t));

    int instance_id;
    ifs.read(reinterpret_cast<char*>(&instance_id), sizeof(instance_id));

    cpu_cbo.emplace_back(rgb);

    vec4f xyzid;
    xyzid.head<3>() = xyz;
    xyzid[3] = static_cast<float>(instance_id);

    cpu_vbo.emplace_back(xyzid);
  }

  int grav_size;
  ifs.read(reinterpret_cast<char*>(&grav_size), sizeof(grav_size));
  ifs.read(reinterpret_cast<char*>(gravity_dir.data()),
           sizeof(float) * grav_size);

  this->orig_gravity_dir = this->gravity_dir;

  const Sophus::SO3f T_esp_scene(
      quatf::FromTwoVectors(this->gravity_dir, geo::ESP_GRAVITY));

  this->gravity_dir = esp::geo::ESP_GRAVITY;
  for (auto& xyzid : this->cpu_vbo) {
    const vec3f xyz_scene = xyzid.head<3>();
    const vec3f xyz_esp = T_esp_scene * xyz_scene;

    xyzid.head<3>() = xyz_esp;
  }

  return true;
}

void FRLInstanceMeshData::to_ply(const std::string& ply_file) const {
  const int nVertex = cpu_vbo.size();

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
    vec3f xyz = cpu_vbo[i].head<3>();
    auto& rgb = cpu_cbo[i];

    f.write(reinterpret_cast<const char*>(xyz.data()), 3 * sizeof(float));
    f.write(reinterpret_cast<const char*>(rgb.data()), 3 * sizeof(uint8_t));

    const int instance_id = std::floor(cpu_vbo[i][3] * id_to_node.size());
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

  // create ibo converting quads to tris [0, 1, 2, 3] -> [0, 1, 2],[0,2,3]
  const size_t numQuads = cpu_vbo.size() / 4;
  std::vector<uint32_t> tri_ibo(numQuads * 6);
  for (uint32_t iQuad = 0; iQuad < numQuads; ++iQuad) {
    const uint32_t triIdx = 6 * iQuad;
    const uint32_t quadIdx = 4 * iQuad;
    tri_ibo[triIdx + 0] = quadIdx + 0;
    tri_ibo[triIdx + 1] = quadIdx + 1;
    tri_ibo[triIdx + 2] = quadIdx + 2;
    tri_ibo[triIdx + 3] = quadIdx + 0;
    tri_ibo[triIdx + 4] = quadIdx + 2;
    tri_ibo[triIdx + 5] = quadIdx + 3;
  }
  // std::vector<vec3ui> cbotest(data.cpu_cbo.size(), vec3ui());
  // Magnum::Containers::ArrayView<const float> v{&data.cpu_vbo[0][0],
  // data.cpu_vbo.size() * 4}; Magnum::Containers::ArrayView<const uint8_t>
  // c{&data.cpu_cbo[0][0], data.cpu_cbo.size() * 3};

  std::vector<float> cbo_float(cpu_cbo.size() * 3);
  for (int iVert = 0; iVert < cpu_cbo.size(); ++iVert) {
    const uint32_t idx = 3 * iVert;
    cbo_float[idx + 0] = cpu_cbo[iVert][0] / 255.0f;
    cbo_float[idx + 1] = cpu_cbo[iVert][1] / 255.0f;
    cbo_float[idx + 2] = cpu_cbo[iVert][2] / 255.0f;
  }

  std::vector<vec3f> xyz_vbo(cpu_vbo.size());
  for (int i = 0; i < cpu_vbo.size(); ++i) {
    xyz_vbo[i] = cpu_vbo[i].head<3>();
  }

  // See code for GenericInstanceMeshData::uploadBuffersToGPU for comments about
  // what's going on with this image/texture
  const size_t numTris = numQuads * 2;
  const int texSize = std::pow(2, std::ceil(std::log2(std::sqrt(numTris))));
  float* obj_id_tex_data = new float[texSize * texSize]();

  for (size_t i = 0; i < numQuads; ++i) {
    obj_id_tex_data[2 * i] = cpu_vbo[4 * i][3];
    obj_id_tex_data[2 * i + 1] = cpu_vbo[4 * i][3];
  }

  renderingBuffer_->tex = createInstanceTexture(obj_id_tex_data, texSize);

  renderingBuffer_->vbo.setData(xyz_vbo, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cbo_float, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(tri_ibo, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(tri_ibo.size())  // Set vertex/index count (numQuads * 6)
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       Magnum::GL::Attribute<0, Magnum::Vector3>{})
      .addVertexBuffer(renderingBuffer_->cbo, 0,
                       Magnum::GL::Attribute<1, Magnum::Color3>{})
      .setIndexBuffer(renderingBuffer_->ibo, 0,
                      Magnum::GL::MeshIndexType::UnsignedInt);

  buffersOnGPU_ = true;
}

}  // namespace assets
}  // namespace esp
