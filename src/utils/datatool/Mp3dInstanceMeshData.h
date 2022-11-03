// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_UTILS_DATATOOL_MP3DINSTANCEDATA_H_
#define ESP_UTILS_DATATOOL_MP3DINSTANCEDATA_H_

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "esp/core/Esp.h"
#include "esp/core/EspEigen.h"

namespace esp {
namespace assets {

/*
 * MP3D object instance segmented mesh
 * Holds a vbo where each vertex is (x, y, z, objectId)
 */
class Mp3dInstanceMeshData {
 public:
  Mp3dInstanceMeshData() {}
  ~Mp3dInstanceMeshData() = default;

  //! Loads an MP3D house segmentations PLY file
  bool loadMp3dPLY(const std::string& plyFile);

  //! Saves semantic mesh PLY with object ids per-vertex
  bool saveSemMeshPLY(
      const std::string& plyFile,
      const std::unordered_map<int, int>& segmentIdToObjectIdMap);

 protected:
  std::vector<vec3f> cpu_vbo_;
  std::vector<vec3uc> cpu_cbo_;
  std::vector<vec3ui> perFaceIdxs_;
  std::vector<int> materialIds_;
  std::vector<int> segmentIds_;
  std::vector<int> categoryIds_;
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_UTILS_DATATOOL_MP3DINSTANCEDATA_H_
