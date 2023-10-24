// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_GENERICSEMANTICMESHDATA_H_
#define ESP_ASSETS_GENERICSEMANTICMESHDATA_H_

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "BaseMesh.h"
#include "esp/core/Esp.h"
#include "esp/scene/SemanticScene.h"

namespace esp {
namespace scene {
class SemanticScene;
}  // namespace scene
namespace assets {

/**
 * @brief Mesh data storage and loading for ply format assets used primarily for
 * Semantic Scene meshes, including manage vertex colors and vertex IDs for
 * semantic visualization and rendering.
 */
class GenericSemanticMeshData : public BaseMesh {
 public:
  /**
   * @brief Stores render data for the mesh.
   */
  struct RenderingBuffer {
    /**
     * @brief Compiled openGL render data for the mesh.
     */
    Magnum::GL::Mesh mesh;
  };

  /**
   * @brief Constructor. Builds semantic mesh data, and sets type to passed
   * SupportedMeshType
   */
  explicit GenericSemanticMeshData(SupportedMeshType type) : BaseMesh{type} {};
  /**
   * @brief Constructor. Builds semantic mesh data, and sets type to
   * SupportedMeshType::INSTANCE_MESH
   */
  explicit GenericSemanticMeshData()
      : GenericSemanticMeshData{SupportedMeshType::INSTANCE_MESH} {};

  ~GenericSemanticMeshData() override = default;

  /**
   * @brief Build the @ref GenericSemanticMeshData based on the
   * contents of the passed @p meshData,
   * @param meshData The imported meshData.
   * @param semanticFilename Path-less Filename of source mesh.
   * @param [out] colorMapToUse An array holding the semantic colors to use for
   * visualization or matching to semantic IDs.
   * @param convertToSRGB Whether the source vertex colors from the @p meshData
   * should be converted to SRGB
   * @param semanticScene The SSD for the semantic mesh being loaded.
   * @return reference to the @ref GenericSemanticMeshData.
   */
  static std::unique_ptr<GenericSemanticMeshData> buildSemanticMeshData(
      const Magnum::Trade::MeshData& meshData,
      const std::string& semanticFilename,
      std::vector<Magnum::Vector3ub>& colorMapToUse,
      bool convertToSRGB,
      const std::shared_ptr<scene::SemanticScene>& semanticScene = nullptr);

  /**
   * @brief Partition the passed @ref GenericSemanticMeshData to facilitate culling.
   * @param semanticMeshData
   * @return vector holding one or more @ref GenericSemanticMeshData
   */
  static std::vector<std::unique_ptr<GenericSemanticMeshData>>
  partitionSemanticMeshData(
      const std::unique_ptr<GenericSemanticMeshData>& semanticMeshData);

  /**
   * @brief Build a per-color/per-semantic ID map of all bounding boxes for each
   * CC found in the mesh, and the count of verts responsible for each.
   * @param semanticScene The SSD for the current semantic mesh.  Used to query
   * semantic objs. If nullptr, this function returns hex-color-keyed map,
   * otherwise returns SemanticID-keyed map.
   */
  std::unordered_map<uint32_t,
                     std::vector<std::shared_ptr<scene::CCSemanticObject>>>
  buildCCBasedSemanticObjs(
      const std::shared_ptr<scene::SemanticScene>& semanticScene);

  // ==== rendering ====
  /**
   * @brief Upload the mesh data to GPU memory.
   */
  void uploadBuffersToGPU(bool forceReload = false) override;

  /**
   * @brief Retrieve a pointer to the rendering buffer for this @ref GenericSemanticMeshData.
   */
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  /**
   * @brief Retrieve a pointer to the Magnum GL Mesh behind this @ref GenericSemanticMeshData.
   */
  Magnum::GL::Mesh* getMagnumGLMesh() override;

  /**
   * @brief Retrive a reference to this @ref GenericSemanticMeshData 's vertex buffer
   */
  const std::vector<Mn::Vector3>& getVertexBufferObjectCPU() const {
    return cpu_vbo_;
  }

  /**
   * @brief Retrive a reference to this @ref GenericSemanticMeshData 's color buffer
   */
  const std::vector<Mn::Color3ub>& getColorBufferObjectCPU() const {
    return cpu_cbo_;
  }

  /**
   * @brief Retrive a reference to this @ref GenericSemanticMeshData 's index buffer
   */
  const std::vector<uint32_t>& getIndexBufferObjectCPU() const {
    return cpu_ibo_;
  }

  /**
   * @brief Retrive a reference to this @ref GenericSemanticMeshData 's object id buffer
   */
  const std::vector<uint16_t>& getObjectIdsBufferObjectCPU() const {
    return objectIds_;
  }

  /**
   * @brief Either return separate partition IDs or objectIDs, depending on
   * which were available when mesh was initially configured.  These are used to
   * partition mesh for culling.
   */
  const std::vector<uint16_t>& getPartitionIDs() const {
    if (meshUsesSSDPartitionIDs) {
      return partitionIds_;
    }
    return objectIds_;
  }
  /**
   * @brief This mesh can be partitioned - either object IDs were found in
   * vertices or per-vert region partition values were found from semantic
   * descriptor file.
   */
  bool meshCanBePartitioned() const { return meshHasPartitionIDXs; }

  /**
   * @brief build a string array holding mapping information for colors found on
   * verts and colors found in semantic scene descriptor aggregated during load.
   */
  std::vector<std::string> getVertColorSSDReport(
      const std::string& semanticFilename,
      const std::vector<Mn::Vector3ub>& colorMapToUse,
      const std::shared_ptr<scene::SemanticScene>& semanticScene);

 protected:
  /**
   * @brief temporary holding structures to hold any non-SSD vert color IDs, so
   * that the nonSSDObjID for new colors can be incremented appropriately not
   * using set to avoid extra include. Key is color, value is semantic ID
   * assigned for unknown color
   */
  std::unordered_map<uint32_t, int> nonSSDVertColorIDs{};
  /**
   * @brief temporary holding structures to hold any non-SSD vert color counts.
   * Key is color, value is semantic ID assigned for unknown color
   */
  std::unordered_map<uint32_t, int> nonSSDVertColorCounts{};

  /**
   * @brief record of semantic object IDXs with no presence in any verts
   */
  std::vector<uint32_t> unMappedObjectIDXs{};

  /**
   * @brief Whether or not this mesh can be partitioned - either object IDs were
   * found in vertices or per-vert region partition values were found from
   * semantic descriptor file.
   */
  bool meshHasPartitionIDXs = false;

  /**
   * @brief This mesh has separate partition IDs, provided by Semantic Scene
   * Descriptor file. If false, uses the objectIDs for the partitioning, if true
   * means region IDs were provided in semantic scene descriptor.
   */
  bool meshUsesSSDPartitionIDs = false;

  /**
   * @brief This class is intended to provide a concise interface to build the
   * appropriate mesh data
   */
  class PerPartitionIdMeshBuilder {
   public:
    /**
     * @brief Build the per-partition @ref GenericSemanticMeshData structure
     */
    PerPartitionIdMeshBuilder(GenericSemanticMeshData& data,
                              uint16_t partitionId)
        : data_{data}, partitionId{partitionId} {}

    /**
     * @brief Add a vertex to the @ref GenericSemanticMeshData
     * @param vertexId The Id of the vertex
     * @param vertex The location of the vertex
     * @param color The vertex color
     * @param objectId The object/semantic Id of the vertex
     */
    void addVertex(uint32_t vertexId,
                   const Mn::Vector3& vertex,
                   const Mn::Color3ub& color,
                   int objectId);

   private:
    GenericSemanticMeshData& data_;
    uint16_t partitionId;
    std::unordered_map<uint32_t, size_t> vertexIdToVertexIndex_;
  };

  /**
   * @brief update the meshdata positions and indices with the data from the
   * member vectors
   */
  void updateCollisionMeshData();

 private:
  // ==== rendering ====
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
  std::vector<Mn::Vector3> cpu_vbo_;
  std::vector<Mn::Color3ub> cpu_cbo_;
  std::vector<uint32_t> cpu_ibo_;
  std::vector<uint16_t> objectIds_;
  // only for mesh paritioning - either points to objectIds_ or to new data
  std::vector<uint16_t> partitionIds_{};

  ESP_SMART_POINTERS(GenericSemanticMeshData)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_GENERICSEMANTICMESHDATA_H_
