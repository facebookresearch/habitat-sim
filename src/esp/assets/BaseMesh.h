// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/MeshData3D.h>
#include "esp/core/esp.h"

namespace esp {
namespace assets {

enum SupportedMeshType {
  NOT_DEFINED = -1,
  INSTANCE_MESH = 0,
  PTEX_MESH = 1,
  GLTF_MESH = 2,
  NUM_SUPPORTED_MESH_TYPES = 3,
};

class BaseMesh {
 public:
  explicit BaseMesh(SupportedMeshType type) : type_(type){};

  virtual ~BaseMesh(){};

  bool setMeshType(SupportedMeshType type);
  SupportedMeshType getMeshType() { return type_; }

  virtual void uploadBuffersToGPU(bool forceReload = false){};

  virtual Magnum::GL::Mesh* getMagnumGLMesh() { return nullptr; }
  virtual Magnum::GL::Mesh* getMagnumGLMesh(int submeshID) { return nullptr; }

  // Accessing non-GL mesh data for physics simulation
  virtual Corrade::Containers::Optional<Magnum::Trade::MeshData3D>& getMeshData();

 protected:
  SupportedMeshType type_ = SupportedMeshType::NOT_DEFINED;
  bool buffersOnGPU_ = false;
};
}  // namespace assets
}  // namespace esp
