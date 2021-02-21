// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_GENERICINSTANCEMESHDATA_H_
#define ESP_ASSETS_GENERICINSTANCEMESHDATA_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <memory>
#include <string>
#include <unordered_map>
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

  ~GenericInstanceMeshData() override = default;

  /**
   * @brief Split a .ply file by objectIDs into different meshes
   *
   * @param plyFile .ply file to load and split
   * @return Mesh data split by objectID
   */
  static std::vector<std::unique_ptr<GenericInstanceMeshData>>
  fromPlySplitByObjectId(Magnum::Trade::AbstractImporter& importer,
                         const std::string& plyFile);

  /**
   * @brief Load from a .ply file
   *
   * @param plyFile .ply file to load
   */
  static std::unique_ptr<GenericInstanceMeshData> fromPLY(
      Magnum::Trade::AbstractImporter& importer,
      const std::string& plyFile);

  // ==== rendering ====
  void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  Magnum::GL::Mesh* getMagnumGLMesh() override;

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
  class PerObjectIdMeshBuilder {
   public:
    PerObjectIdMeshBuilder(GenericInstanceMeshData& data, uint16_t objectId)
        : data_{data}, objectId_{objectId} {}

    void addVertex(uint32_t vertexId, const vec3f& vertex, const vec3uc& color);

   private:
    GenericInstanceMeshData& data_;
    uint16_t objectId_;
    std::unordered_map<uint32_t, size_t> vertexIdToVertexIndex_;
  };

  void updateCollisionMeshData();

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<uint32_t> cpu_ibo_;
  std::vector<uint16_t> objectIds_;

  ESP_SMART_POINTERS(GenericInstanceMeshData)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_GENERICINSTANCEMESHDATA_H_
