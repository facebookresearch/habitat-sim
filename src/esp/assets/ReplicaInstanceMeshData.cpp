// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplicaInstanceMeshData.h"

#include <tinyply.h>
#include <fstream>
#include <sstream>
#include <vector>

#include <sophus/so3.hpp>

#include "esp/core/esp.h"
#include "esp/geo/geo.h"
#include "esp/io/io.h"

namespace esp {
namespace assets {

namespace {
template <typename T>
void copyTo(const std::shared_ptr<tinyply::PlyData>& data,
            std::vector<T>& dst) {
  dst.resize(data->count);
  std::memcpy(dst.data(), data->buffer.get(), data->buffer.size_bytes());
}
}  // namespace

bool ReplicaInstanceMeshData::loadPLY(const std::string& plyFile) {
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
        file.request_properties_from_element("face", {"vertex_indices"}, 4);
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

  copyTo(face_inds, cpu_ibo_);
  copyTo(vertices, cpu_vbo_);
  copyTo(colors, cpu_cbo_);
  copyTo(object_ids, cpu_object_ids_);

  // Replica semantic PLY meshes have -Z gravity
  const quatf T_esp_scene =
      quatf::FromTwoVectors(-vec3f::UnitZ(), geo::ESP_GRAVITY);

  for (auto& xyz : cpu_vbo_) {
    xyz = T_esp_scene * xyz;
  }

  return true;
}

Magnum::GL::Mesh* ReplicaInstanceMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }
  return &(renderingBuffer_->mesh);
}

void ReplicaInstanceMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ =
      std::make_unique<ReplicaInstanceMeshData::RenderingBuffer>();

  // create ibo converting quads to tris [0, 1, 2, 3] -> [0, 1, 2],[0, 2, 3]
  const size_t numQuads = cpu_ibo_.size();
  std::vector<uint32_t> tri_ibo(numQuads * 6);
  for (uint32_t iQuad = 0; iQuad < numQuads; ++iQuad) {
    const uint32_t triIdx = 6 * iQuad;
    const auto& quad = cpu_ibo_[iQuad];
    tri_ibo[triIdx + 0] = quad[0];
    tri_ibo[triIdx + 1] = quad[1];
    tri_ibo[triIdx + 2] = quad[2];
    tri_ibo[triIdx + 3] = quad[0];
    tri_ibo[triIdx + 4] = quad[2];
    tri_ibo[triIdx + 5] = quad[3];
  }

  /* uint16_t max_obj_id = 0;
  for (auto& id : cpu_object_ids_) {
    max_obj_id = std::max(id, max_obj_id);
  }
  LOG(INFO) << max_obj_id;
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

  std::vector<vec4f> labeled_vbo;
  std::vector<vec3f> cbo_float;
  for (int i = 0; i < tri_ibo.size(); ++i) {
    const int idx = tri_ibo[i];
    vec4f xyzid;
    xyzid.head<3>() = cpu_vbo_[idx];
    xyzid[3] = cpu_object_ids_[i / 6];
    labeled_vbo.emplace_back(xyzid);

    cbo_float.emplace_back(cpu_cbo_[idx].cast<float>() / 255.0f);

    // cbo_float.emplace_back(bgr_walk(xyzid[3]));
  }

  for (uint32_t i = 0; i < tri_ibo.size(); ++i) {
    tri_ibo[i] = i;
  }

  renderingBuffer_->vbo.setData(labeled_vbo,
                                Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->cbo.setData(cbo_float, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->ibo.setData(tri_ibo, Magnum::GL::BufferUsage::StaticDraw);
  renderingBuffer_->mesh.setPrimitive(Magnum::GL::MeshPrimitive::Triangles)
      .setCount(tri_ibo.size())
      .addVertexBuffer(renderingBuffer_->vbo, 0,
                       Magnum::GL::Attribute<0, Magnum::Vector4>{})
      .addVertexBuffer(renderingBuffer_->cbo, 0,
                       Magnum::GL::Attribute<1, Magnum::Color3>{})
      .setIndexBuffer(renderingBuffer_->ibo, 0,
                      Magnum::GL::MeshIndexType::UnsignedInt);

  buffersOnGPU_ = true;
}

}  // namespace assets
}  // namespace esp
