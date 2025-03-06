// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericMeshData.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/DebugStl.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/GenerateIndices.h>
#include <Magnum/MeshTools/Interleave.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace assets {

void GenericMeshData::uploadBuffersToGPU(bool forceReload) {
  if (forceReload) {
    buffersOnGPU_ = false;
  }
  if (buffersOnGPU_) {
    return;
  }

  renderingBuffer_.reset();
  renderingBuffer_ = std::make_unique<GenericMeshData::RenderingBuffer>();
  Magnum::MeshTools::CompileFlags compileFlags{};
  if (needsNormals_ &&
      !meshData_->hasAttribute(Mn::Trade::MeshAttribute::Normal)) {
    compileFlags |= Magnum::MeshTools::CompileFlag::GenerateSmoothNormals;
  }
  // position, normals, uv, colors are bound to corresponding attributes
  renderingBuffer_->mesh = Magnum::MeshTools::compile(*meshData_, compileFlags);

  buffersOnGPU_ = true;
}

Magnum::GL::Mesh* GenericMeshData::getMagnumGLMesh() {
  if (renderingBuffer_ == nullptr) {
    return nullptr;
  }

  return &(renderingBuffer_->mesh);
}

void GenericMeshData::setMeshData(Magnum::Trade::MeshData&& meshData) {
  /* TODO: Address that non-triangle meshes will have their collisionMeshData_
   * incorrectly calculated */

  meshData_ = std::move(meshData);

  collisionMeshData_.primitive = meshData_->primitive();

  /* For collision data we need positions as Vector3 in a contiguous array.
     There's little chance the data are stored like that in MeshData, so unpack
     them to an array. */
  collisionMeshData_.positions = positionData_ =
      meshData_->positions3DAsArray();

  /* For collision data we need indices as UnsignedInt. If the mesh already has
     those, and we can get a mutable view on them, just make the collision data
     reference them. If not, unpack them and store them here. And if there is
     no index buffer at all, generate a trivial one (0, 1, 2, 3...).

     TODO why is a mutable view needed at all?! If not, make the views const
     and remove the indexDataFlags() check below. */
  if (!meshData_->isIndexed()) {
    collisionMeshData_.indices = indexData_ = Mn::MeshTools::generateTrivialIndices(meshData_->vertexCount());
  } else if (meshData_->indexType() == Mn::MeshIndexType::UnsignedInt && meshData_->indexDataFlags() & Mn::Trade::DataFlag::Mutable) {
    collisionMeshData_.indices =
        meshData_->mutableIndices<Mn::UnsignedInt>().asContiguous();
  } else {
    collisionMeshData_.indices = indexData_ = meshData_->indicesAsArray();
  }
}  // setMeshData

void GenericMeshData::importAndSetMeshData(
    Magnum::Trade::AbstractImporter& importer,
    int meshID) {
  /* Guarantee mesh instance success */
  Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer.mesh(meshID);
  CORRADE_INTERNAL_ASSERT(mesh);
  setMeshData(*std::move(mesh));
}  // importAndSetMeshData

void GenericMeshData::importAndSetMeshData(
    Magnum::Trade::AbstractImporter& importer,
    const std::string& meshName) {
  /* Guarantee mesh instance success */
  Cr::Containers::Optional<Mn::Trade::MeshData> mesh = importer.mesh(meshName);
  CORRADE_INTERNAL_ASSERT(mesh);
  setMeshData(*std::move(mesh));
}  // importAndSetMeshData

}  // namespace assets
}  // namespace esp
