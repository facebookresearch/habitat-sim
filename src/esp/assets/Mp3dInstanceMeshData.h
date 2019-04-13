// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>

#include "BaseMesh.h"
#include "esp/assets/InstanceMeshData.h"
#include "esp/core/esp.h"

namespace esp {
namespace assets {

/*
 * MP3D object instance segmented mesh
 * Holds a vbo where each vertex is (x, y, z, objectId)
 */
class Mp3dInstanceMeshData : public InstanceMeshBase {
 public:
  Mp3dInstanceMeshData() : InstanceMeshBase(SupportedMeshType::INSTANCE_MESH) {}
  virtual ~Mp3dInstanceMeshData() {}

  //! Loads an MP3D house segmentations PLY file
  bool loadMp3dPLY(const std::string& plyFile);

  //! Saves semantic mesh PLY with object ids per-vertex
  bool saveSemMeshPLY(
      const std::string& plyFile,
      const std::unordered_map<int, int>& segmentIdToObjectIdMap);

  //! Loads semantic mesh PLY with object ids per-vertex
  bool loadSemMeshPLY(const std::string& plyFile);
  virtual bool loadPLY(const std::string& plyFile) override {
    return loadSemMeshPLY(plyFile);
  };

  // ==== rendering ====
  virtual void uploadBuffersToGPU(bool forceReload = false) override;
  RenderingBuffer* getRenderingBuffer() { return renderingBuffer_.get(); }

  virtual Magnum::GL::Mesh* getMagnumGLMesh() override;

 protected:
  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<vec3i> cpu_ibo_;
  std::vector<int> materialIds_;
  std::vector<int> segmentIds_;
  std::vector<int> categoryIds_;

  std::vector<uint32_t> objectIds_;
};

}  // namespace assets
}  // namespace esp
