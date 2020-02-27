// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::assets::GltfMeshData, Class @ref
 * esp::assets::GltfMeshData::RenderingBuffer
 */

#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/MeshData3D.h>

#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

/**
 * @brief Mesh data storage and loading for gltf format assets. See @ref
 * ResourceManager::loadGeneralMeshData.
 */
class GltfMeshData : public BaseMesh {
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

  /** @brief Constructor. Sets @ref SupportedMeshType::GLTF_MESH to identify the
   * asset type.*/
  GltfMeshData() : BaseMesh(SupportedMeshType::GLTF_MESH){};

  /** @brief Destructor */
  virtual ~GltfMeshData(){};

  /**
   * @brief Compile the @ref renderingBuffer_ if first upload or forceReload is
   * true.
   * @param forceReload If true, recompiles the @ref renderingBuffer_ (e.g. in
   * the case of data change after initial compilation).
   */
  virtual void uploadBuffersToGPU(bool forceReload = false) override;

  /**
   * @brief Load mesh data from a pre-parsed importer for a specific mesh
   * component. Sets the @ref collisionMeshData_ references.
   * @param importer The importer pre-loaded with asset data from file.
   * @param meshID The local identifier of a specific mesh component of the
   * asset.
   */
  void setMeshData(Magnum::Trade::AbstractImporter& importer, int meshID);

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
  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

 protected:
  /**
   * @brief Storage structure for compiled render data. We will use a smart
   * pointer here since each item within the structure (e.g., Magnum::GL::Mesh)
   * does NOT have copy constructor. See @ref uploadBuffersToGPU.
   */
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
};
}  // namespace assets
}  // namespace esp
