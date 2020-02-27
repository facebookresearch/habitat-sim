// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/MeshData3D.h>
#include <memory>
#include <string>
#include <vector>
#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

class GenericInstanceMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
  };

  explicit GenericInstanceMeshData(SupportedMeshType type) : BaseMesh{type} {};
  explicit GenericInstanceMeshData()
      : GenericInstanceMeshData{SupportedMeshType::INSTANCE_MESH} {};

  virtual ~GenericInstanceMeshData(){};

  virtual bool loadPLY(const std::string& plyFile);

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

  const std::vector<uint32_t>& getIndexBufferObjectCPU() const {
    return cpu_ibo_;
  }

 protected:
  void updateCollisionMeshData();

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<uint32_t> cpu_ibo_;
  std::vector<uint16_t> objectIds_;
};
}  // namespace assets
}  // namespace esp
