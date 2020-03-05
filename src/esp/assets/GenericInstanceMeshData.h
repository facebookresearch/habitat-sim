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
#include <unordered_map>
#include <vector>

#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

struct ParsedPlyData {
  std::vector<vec3f> cpu_vbo;
  std::vector<vec3uc> cpu_cbo;
  std::vector<uint32_t> cpu_ibo;
  std::vector<uint16_t> objectIds;
};

class GenericInstanceMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
  };

  explicit GenericInstanceMeshData(SupportedMeshType type) : BaseMesh{type} {};
  explicit GenericInstanceMeshData()
      : GenericInstanceMeshData{SupportedMeshType::INSTANCE_MESH} {};

  GenericInstanceMeshData(GenericInstanceMeshData&&) = default;

  virtual ~GenericInstanceMeshData(){};

  /**
   * @brief loads a PLY file and splits it by objectIDs into multiple different
   * mesh's
   *
   * @param plyFile
   * @return std::vector<GenericInstanceMeshData>
   */
  static Corrade::Containers::Optional<std::vector<GenericInstanceMeshData>>
  loadPlySplitByObjectId(const std::string& plyFile);

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

  const std::vector<uint16_t>& getObjectIdsBufferObjectCPU() const {
    return objectIds_;
  }

 protected:
  struct PerObjectIDData {
    size_t meshDataIndex;
    uint16_t objectId;
    std::unordered_map<uint32_t, uint32_t> globalIndexToPerObjectIndex;
  };

  void updateCollisionMeshData();

  void addIndexToMeshData(uint32_t globalIndex,
                          const ParsedPlyData& globalData,
                          PerObjectIDData& localData);

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<uint32_t> cpu_ibo_;
  std::vector<uint16_t> objectIds_;
};
}  // namespace assets
}  // namespace esp
