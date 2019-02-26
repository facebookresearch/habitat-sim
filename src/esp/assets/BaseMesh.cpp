// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BaseMesh.h"
#include <Magnum/MeshTools/Compile.h>

namespace esp {
namespace assets {

bool BaseMesh::setMeshType(SupportedMeshType type) {
  if (type < SupportedMeshType::NOT_DEFINED ||
      type >= SupportedMeshType::NUM_SUPPORTED_MESH_TYPES) {
    LOG(ERROR) << "Cannot set the mesh type to " << type;
    return false;
  }

  type_ = type;
  return true;
}

}  // namespace assets
}  // namespace esp
