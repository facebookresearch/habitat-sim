// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GltfMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Interleave.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

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
  Magnum::MeshTools::CompileFlags compileFlags{};
  if (needsNormals_ &&
      !meshData_->hasAttribute(Mn::Trade::MeshAttribute::Normal)) {
    compileFlags |= Magnum::MeshTools::CompileFlag::GenerateSmoothNormals;
  }
  // position, normals, uv, colors are bound to corresponding attributes
  renderingBuffer_->mesh = Magnum::MeshTools::compile(*meshData_, compileFlags);

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
  ASSERT(0 <= meshID && meshID < importer.meshCount());
  /* Interleave the mesh, if not already. This makes the GPU happier (better
     cache locality for vertex fetching) and is a no-op if the source data is
     already interleaved, so doesn't hurt to have it there always. */
  Cr::Containers::Optional<Mn::Trade::MeshData> meshData =
      importer.mesh(meshID);
  if (meshData)
    meshData_ = Mn::MeshTools::interleave(*std::move(meshData));
  else
    meshData_ = Cr::Containers::NullOpt;

  collisionMeshData_.primitive = Magnum::MeshPrimitive::Triangles;

  /* For collision data we need positions as Vector3 in a contiguous array.
     There's little chance the data are stored like that in MeshData, so unpack
     them to an array. */
  collisionMeshData_.positions = positionData_ =
      meshData_->positions3DAsArray();

  /* For collision data we need indices as UnsignedInt. If the mesh already has
     those, just make the collision data reference them. If not, unpack them
     and store them here. */
  if (meshData_->indexType() == Mn::MeshIndexType::UnsignedInt)
    collisionMeshData_.indices = meshData_->mutableIndices<Mn::UnsignedInt>();
  else
    collisionMeshData_.indices = indexData_ = meshData_->indicesAsArray();
}

}  // namespace assets
}  // namespace esp
