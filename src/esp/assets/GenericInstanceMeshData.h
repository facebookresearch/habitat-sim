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
#include "esp/core/Esp.h"

namespace esp {
namespace scene {
class SemanticScene;
}  // namespace scene
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
   * @brief Load a .ply file into one ore more @ref GenericInstanceMeshData,
   * splitting the result into multiples if specified and if the source file's
   * objectIds are compatibly configured to do so.
   * @param importer The importer to use to load the .ply file
   * @param plyFile Fully qualified filename of .ply file to load
   * @param splitMesh Whether or not the resultant mesh should be split into
   * multiple components based on objectIds, for frustum culling.
   * @param semanticScene The SSD for the instance mesh being loaded.
   * @return vector holding one or more mesh results from the .ply file.
   */

  static std::vector<std::unique_ptr<GenericInstanceMeshData>> fromPLY(
      Magnum::Trade::AbstractImporter& importer,
      const std::string& plyFile,
      bool splitMesh,
      std::vector<Magnum::Vector3ub>& colorMapToUse,
      const std::shared_ptr<scene::SemanticScene>& semanticScene = nullptr);

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
  class PerPartitionIdMeshBuilder {
   public:
    PerPartitionIdMeshBuilder(GenericInstanceMeshData& data,
                              uint16_t partitionId)
        : data_{data}, partitionId{partitionId} {}

    void addVertex(uint32_t vertexId,
                   const vec3f& vertex,
                   const vec3uc& color,
                   int objectId);

   private:
    GenericInstanceMeshData& data_;
    uint16_t partitionId;
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
