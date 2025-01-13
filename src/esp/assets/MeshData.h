// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MESHDATA_H_
#define ESP_ASSETS_MESHDATA_H_

#include <vector>

#include "esp/core/Esp.h"

namespace esp {
namespace assets {

//! Raw mesh data storage
struct MeshData {
  //! Vertex positions
  std::vector<Magnum::Vector3> vbo;
  //! Vertex normals
  std::vector<Magnum::Vector3> nbo;
  //! Texture coordinates
  std::vector<Magnum::Vector2> tbo;
  //! Vertex colors
  std::vector<Magnum::Vector3> cbo;
  //! Index buffer
  std::vector<uint32_t> ibo;

  ESP_SMART_POINTERS(MeshData)
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MESHDATA_H_
