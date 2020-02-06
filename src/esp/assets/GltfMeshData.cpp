// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GltfMeshData.h"
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/MeshTools/Compile.h>

namespace esp {
namespace assets {
void GltfMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ = std::make_unique<GltfMeshData::RenderingBuffer>();
  // position, normals, uv, colors are bound to corresponding attributes
  renderingBuffer_->mesh = Magnum::MeshTools::compile(
      *meshData_, Magnum::MeshTools::CompileFlag::GenerateSmoothNormals);
  buffersOnGPU_ = true;
}

Magnum::GL::Mesh* GltfMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }

  return &(renderingBuffer_->mesh);
}

void GltfMeshData::setMeshData(Magnum::Trade::AbstractImporter& importer,
                               int meshID) {
  ASSERT(0 <= meshID && meshID < importer.mesh3DCount());
  meshData_ = importer.mesh3D(meshID);

  collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;
  collisionMeshData_.positions = meshData_->positions(0);
  collisionMeshData_.indices = meshData_->indices();
}

}  // namespace assets
}  // namespace esp
