// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>

#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

/*
 * SurrealSim instance segmented mesh.
 * Holds a vbo where each vertex is (x, y, z, id).
 * id_to_label and id_to_node map face id to instance and node ids
 * Faces are assumed to be quads
 */
class InstanceMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
    Magnum::GL::Buffer vbo;
    Magnum::GL::Buffer cbo;
    Magnum::GL::Buffer ibo;
  };

  InstanceMeshData() : BaseMesh(SupportedMeshType::INSTANCE_MESH){};
  virtual ~InstanceMeshData(){};

  bool from_ply(const std::string& ply_file);
  void to_ply(const std::string& ply_file) const;

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

 protected:
  std::vector<vec4f> cpu_vbo;
  std::vector<vec3uc> cpu_cbo;

  vecXi id_to_label;
  vecXi id_to_node;

  // Current gravity direction within the mesh
  vec3f gravity_dir;

  // Gravity direction of the mesh, this is a STATIC
  vec3f orig_gravity_dir;

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
};

}  // namespace assets
}  // namespace esp
