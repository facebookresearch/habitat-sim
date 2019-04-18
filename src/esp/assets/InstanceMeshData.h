// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>

#include <Magnum/GL/Texture.h>
#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

class GenericInstanceMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
    Magnum::GL::Buffer vbo;
    Magnum::GL::Buffer cbo;
    Magnum::GL::Buffer ibo;
    Magnum::GL::Texture2D tex;
  };

  explicit GenericInstanceMeshData(SupportedMeshType type) : BaseMesh{type} {};
  explicit GenericInstanceMeshData()
      : GenericInstanceMeshData{SupportedMeshType::INSTANCE_MESH} {};

  virtual bool loadPLY(const std::string& plyFile);

  virtual Magnum::GL::Texture2D* getSemanticTexture() {
    return &renderingBuffer_->tex;
  };

  // ==== rendering ====
  virtual void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

  const std::vector<vec3f>& getVertexBufferObjectCPU() const {
    return cpu_vbo_;
  }
  const std::vector<vec3uc>& getColorBufferObjectCPU() const {
    return cpu_cbo_;
  }

  const std::vector<vec3ui> getIndexBufferObjectCPU() const { return cpu_ibo_; }

 protected:
  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<vec3ui> cpu_ibo_;
  std::vector<uint32_t> objectIds_;
};

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
  virtual ~FRLInstanceMeshData(){};

  bool from_ply(const std::string& ply_file);
  void to_ply(const std::string& ply_file) const;
  virtual bool loadPLY(const std::string& plyFile) override {
    return from_ply(plyFile);
  };

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
};

}  // namespace assets
}  // namespace esp
