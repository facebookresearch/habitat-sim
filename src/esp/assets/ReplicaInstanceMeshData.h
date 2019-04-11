// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>

#include "BaseMesh.h"
#include "esp/assets/InstanceMeshData.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

/*
 * MP3D object instance segmented mesh
 * Holds a vbo where each vertex is (x, y, z, objectId)
 */
class ReplicaInstanceMeshData : public InstanceMeshBase {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
    Magnum::GL::Buffer vbo;
    Magnum::GL::Buffer cbo;
    Magnum::GL::Buffer ibo;
  };

  ReplicaInstanceMeshData()
      : InstanceMeshBase(SupportedMeshType::INSTANCE_MESH) {}
  virtual ~ReplicaInstanceMeshData() {}

  virtual bool loadPLY(const std::string& plyFile) override;

  std::vector<vec3f>& getVertexBufferObjectCPU() { return cpu_vbo_; }
  std::vector<vec3uc>& getColorBufferObjectCPU() { return cpu_cbo_; }
  std::vector<vec4ui>& getIndexBufferObjectCPU() { return cpu_ibo_; }

  // overloaded function, in case object passed as a const parameter to the
  // function
  const std::vector<vec3f>& getVertexBufferObjectCPU() const {
    return cpu_vbo_;
  }
  const std::vector<vec3uc>& getColorBufferObjectCPU() const {
    return cpu_cbo_;
  }
  const std::vector<vec4ui>& getIndexBufferObjectCPU() const {
    return cpu_ibo_;
  }

  // ==== rendering ====
  virtual void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

 protected:
  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<vec4ui> cpu_ibo_;
  std::vector<uint16_t> cpu_object_ids_;

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
};

}  // namespace assets
}  // namespace esp
