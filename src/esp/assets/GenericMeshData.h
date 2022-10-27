// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_GENERICMESHDATA_H_
#define ESP_ASSETS_GENERICMESHDATA_H_

/** @file
 * @brief Class @ref esp::assets::GenericMeshData, Class @ref
 * esp::assets::GenericMeshData::RenderingBuffer
 */

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/AbstractImporter.h>

#include "BaseMesh.h"
#include "esp/core/Esp.h"

namespace esp {
namespace assets {

/**
 * @brief Mesh data storage and loading for gltf format assets. See @ref
 * ResourceManager::loadGeneralMeshData.
 */
class GenericMeshData : public BaseMesh {
 public:
  /**
   * @brief Stores render data for the mesh necessary for gltf format.
   */
  struct RenderingBuffer {
    /**
     * @brief Compiled openGL render data for the mesh.
     */
    Magnum::GL::Mesh mesh;
  };

  /** @brief Constructor. Sets @ref SupportedMeshType::GENERIC_MESH to identify
   * the asset type.*/
  explicit GenericMeshData(bool needsNormals = true)
      : BaseMesh(SupportedMeshType::GENERIC_MESH),
        needsNormals_{needsNormals} {};

  /** @brief Destructor */
  ~GenericMeshData() override = default;

  /**
   * @brief Compile the @ref renderingBuffer_ if first upload or forceReload is
   * true.
   * @param forceReload If true, recompiles the @ref renderingBuffer_ (e.g. in
   * the case of data change after initial compilation).
   */
  void uploadBuffersToGPU(bool forceReload = false) override;

  /**
   * @brief Set mesh data from external source, and sets the @ref collisionMesh_
   * references.  Can be used for meshDatas that are manually synthesized, such
   * as NavMesh. Sets the @ref collisionMesh_ references.
   * @param meshData the meshData to be assigned.
   */
  void setMeshData(Magnum::Trade::MeshData&& meshData);

  /**
   * @brief Load mesh data from a pre-parsed importer for a specific mesh
   * component ID. Sets the @ref collisionMeshData_ references.
   * @param importer The importer pre-loaded with asset data from file, or a
   * Primitive Importer.
   * @param meshID The local identifier of a specific mesh component of the
   * asset.
   */
  void importAndSetMeshData(Magnum::Trade::AbstractImporter& importer,
                            int meshID);
  /**
   * @brief Load mesh data from a pre-parsed importer for a specific mesh
   * component name. Sets the @ref collisionMeshData_ references.
   * @param importer The importer pre-loaded with asset data from file, or a
   * Primitive Importer.
   * @param meshName The string identifier of a specific mesh - i.e. with
   * PrimitiveImporter to denote which Primitive to instantiate.
   */
  void importAndSetMeshData(Magnum::Trade::AbstractImporter& importer,
                            const std::string& meshName);

  /**
   * @brief Returns a pointer to the compiled render data storage structure.
   * @return Pointer to the @ref renderingBuffer_.
   */
  virtual RenderingBuffer* getRenderingBuffer() {
    return renderingBuffer_.get();
  }

  /**
   * @brief Returns a pointer to the compiled render mesh data stored in the
   * @ref renderingBuffer_.
   * @return Pointer to the compiled render mesh data.
   */
  Magnum::GL::Mesh* getMagnumGLMesh() override;

 protected:
  /**
   * @brief Storage structure for compiled render data. We will use a smart
   * pointer here since each item within the structure (e.g., Magnum::GL::Mesh)
   * does NOT have copy constructor. See @ref uploadBuffersToGPU.
   */
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;

  bool needsNormals_ = true;

 private:
  /* Internal; can store data referenced by positions / indices if the original
     MeshData doesn't have them in desired type */
  Corrade::Containers::Array<Magnum::Vector3> positionData_;
  Corrade::Containers::Array<Magnum::UnsignedInt> indexData_;
};
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_GENERICMESHDATA_H_
