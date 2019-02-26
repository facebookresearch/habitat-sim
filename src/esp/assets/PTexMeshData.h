// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/BufferTexture.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>

#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

class PTexMeshData : public BaseMesh {
 public:
  struct MeshData {
    std::vector<vec4f> vbo;
    std::vector<vec4f> nbo;
    std::vector<vec4uc> cbo;
    std::vector<uint32_t> ibo;
  };

  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
    Magnum::GL::Texture2D tex;
    Magnum::GL::Buffer vbo;
    Magnum::GL::Buffer ibo;
    Magnum::GL::Buffer abo;
    Magnum::GL::BufferTexture adjTex;
  };

  PTexMeshData() : BaseMesh(SupportedMeshType::PTEX_MESH) {}
  virtual ~PTexMeshData() {}

  // ==== geometry ====
  void load(const std::string& meshFile, const std::string& atlasFolder);
  float exposure() const;
  void setExposure(const float& val);
  uint32_t tileSize() const { return tileSize_; }

  const std::vector<MeshData>& meshes() const;
  std::string atlasFolder() const;
  void resize(size_t n) { submeshes_.resize(n); }

  int getSize() { return submeshes_.size(); }

  static void parsePLY(const std::string& filename, MeshData& meshData);
  static void calculateAdjacency(const MeshData& mesh,
                                 std::vector<uint32_t>& adjFaces);

  // ==== rendering ====
  RenderingBuffer* getRenderingBuffer(int submeshID);
  virtual void uploadBuffersToGPU(bool forceReload = false) override;
  virtual Magnum::GL::Mesh* getMagnumGLMesh(int submeshID) override;

 protected:
  void loadMeshData(const std::string& meshFile);

  float splitSize_ = 0.0f;
  uint32_t tileSize_ = 0;
  float exposure_ = 1.0f;
  std::string atlasFolder_;
  std::vector<MeshData> submeshes_;

  // ==== rendering ====
  // we will have to use smart pointer here since each item within the structure
  // (e.g., Magnum::GL::Mesh) does NOT have copy constructor
  std::vector<std::unique_ptr<RenderingBuffer>> renderingBuffers_;
};

}  // namespace assets
}  // namespace esp
