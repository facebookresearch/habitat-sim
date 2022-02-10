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

/**
 * @brief Mesh data storage and loading for ply format assets used primarily for
 * Semantic Scene meshes, including manage vertex colors and vertex IDs for
 * semantic visualization and rendering. See @ref
 * ResourceManager::loadRenderAssetIMesh.
 */
class GenericSemanticMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
  };

  explicit GenericSemanticMeshData(SupportedMeshType type) : BaseMesh{type} {};
  explicit GenericSemanticMeshData()
      : GenericSemanticMeshData{SupportedMeshType::INSTANCE_MESH} {};

  ~GenericSemanticMeshData() override = default;

  /**
   * @brief Build one ore more @ref GenericSemanticMeshData based on the
   * contents of the passed @p meshData, splitting the result into multiples if
   * specified and if the source file's objectIds are compatibly configured to
   * do so.
   * @param meshData The imported meshData.
   * @param semanticFilename Path-less Filename of source mesh.
   * @param splitMesh Whether or not the resultant mesh should be split into
   * multiple components based on objectIds, for frustum culling.
   * @param [out] colorMapToUse An array holding the semantic colors to use for
   * visualization or matching to semantic IDs.
   * @param convertToSRGB Whether the source vertex colors from the @p meshData
   * should be converted to SRGB
   * @param semanticScene The SSD for the instance mesh being loaded.
   * @return vector holding one or more mesh results from the semantic asset
   * file.
   */

  static std::vector<std::unique_ptr<GenericSemanticMeshData>>
  buildSemanticMeshData(
      const Magnum::Trade::MeshData& meshData,
      const std::string& semanticFilename,
      bool splitMesh,
      std::vector<Magnum::Vector3ub>& colorMapToUse,
      bool convertToSRGB,
      const std::shared_ptr<scene::SemanticScene>& semanticScene = nullptr);

  // ==== rendering ====
  void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  Magnum::GL::Mesh* getMagnumGLMesh() override;

  const std::vector<Mn::Vector3>& getVertexBufferObjectCPU() const {
    return cpu_vbo_;
  }
  const std::vector<Mn::Color3ub>& getColorBufferObjectCPU() const {
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
    PerPartitionIdMeshBuilder(GenericSemanticMeshData& data,
                              uint16_t partitionId)
        : data_{data}, partitionId{partitionId} {}

    void addVertex(uint32_t vertexId,
                   const Mn::Vector3& vertex,
                   const Mn::Color3ub& color,
                   int objectId);

   private:
    GenericSemanticMeshData& data_;
    uint16_t partitionId;
    std::unordered_map<uint32_t, size_t> vertexIdToVertexIndex_;
  };

  void updateCollisionMeshData();

  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
  std::vector<Mn::Vector3> cpu_vbo_;
  std::vector<Mn::Color3ub> cpu_cbo_;
  std::vector<uint32_t> cpu_ibo_;
  std::vector<uint16_t> objectIds_;

  ESP_SMART_POINTERS(GenericSemanticMeshData)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_GENERICINSTANCEMESHDATA_H_
