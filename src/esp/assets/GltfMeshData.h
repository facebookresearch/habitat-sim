// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Corrade/Containers/Optional.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/MeshData3D.h>

#include "BaseMesh.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {
class GltfMeshData : public BaseMesh {
 public:
  struct RenderingBuffer {
    Magnum::GL::Mesh mesh;
  };
  GltfMeshData() : BaseMesh(SupportedMeshType::GLTF_MESH){};

  virtual ~GltfMeshData(){};

  virtual void uploadBuffersToGPU(bool forceReload = false) override;

  void setMeshData(Magnum::Trade::AbstractImporter& importer, int meshID);
  Corrade::Containers::Optional<Magnum::Trade::MeshData3D>& getMeshData() {
    return meshData_;
  }

  virtual RenderingBuffer* getRenderingBuffer() {
    return renderingBuffer_.get();
  }

  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

 protected:
  Corrade::Containers::Optional<Magnum::Trade::MeshData3D> meshData_;
  // we will have to use smart pointer here since each item within the structure
  // (e.g., Magnum::GL::Mesh) does NOT have copy constructor
  std::unique_ptr<RenderingBuffer> renderingBuffer_ = nullptr;
};
}  // namespace assets
}  // namespace esp
