// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "GenericInstanceMeshData.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

/*
 * SurrealSim instance segmented mesh.
 * Holds a vbo where each vertex is (x, y, z, id).
 * id_to_label and id_to_node map face id to instance and node ids
 * Faces are assumed to be quads
 */
class FRLInstanceMeshData : public GenericInstanceMeshData {
 public:
  FRLInstanceMeshData()
      : GenericInstanceMeshData(SupportedMeshType::INSTANCE_MESH){};
  ~FRLInstanceMeshData(){
      delete xyz_vbo;
      delete tri_ibo;
      delete cbo_float;
      delete obj_id_tex_data;
      delete tri_ibo_i;
  };

  void to_ply(const std::string& ply_file) const;
  virtual bool loadPLY(const std::string& plyFile) override;

  std::vector<vec4f>& getVertexBufferObjectCPU() { return cpu_vbo; }
  std::vector<vec3uc>& getColorBufferObjectCPU() { return cpu_cbo; }

  // overloaded function, in case object passed as a const parameter to the
  // function
  const std::vector<vec4f>& getVertexBufferObjectCPU() const { return cpu_vbo; }
  const std::vector<vec3uc>& getColorBufferObjectCPU() const { return cpu_cbo; }

  // ==== rendering ====
  virtual void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

  const std::vector<vec3f> get_vbo();
  const std::vector<int> get_ibo();

 protected:
  std::vector<vec4f> cpu_vbo;
  std::vector<vec3uc> cpu_cbo;

  std::vector<vec3f>* xyz_vbo     = nullptr;
  std::vector<uint32_t>* tri_ibo  = nullptr;
  std::vector<int>* tri_ibo_i     = nullptr;
  std::vector<float>* cbo_float   = nullptr;
  float* obj_id_tex_data          = nullptr;

  vecXi id_to_label;
  vecXi id_to_node;

  // Current gravity direction within the mesh
  vec3f gravity_dir;

  // Gravity direction of the mesh, this is a STATIC
  vec3f orig_gravity_dir;

  void compute_buffer();
};

}  // namespace assets
}  // namespace esp
