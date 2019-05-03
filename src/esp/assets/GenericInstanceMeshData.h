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

/*
 * In modern OpenGL fragment sharder, we can access the ID of the current
 * primitive and thus can index an array.  We can make a 2D texture behave
 * like a 2D array with nearest sampling and edge clamping.
 */
Magnum::GL::Texture2D createInstanceTexture(float* data, const int texSize);

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

  const std::vector<vec3ui>& getIndexBufferObjectCPU() const {
    return cpu_ibo_;
  }

 protected:
  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<vec3ui> cpu_ibo_;
  std::vector<uint32_t> objectIds_;
};
}  // namespace assets
}  // namespace esp
