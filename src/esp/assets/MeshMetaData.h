// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

namespace esp {
namespace assets {

//! Some mesh files include a transformation hierarchy. A @ref MeshTransformNode
//! stores this hierarchy and indices for the meshes at each level such that it
//! can be reused to instances meshes later.
struct MeshTransformNode {
  //! mesh ID within @ref MeshMetaData::meshIndex
  int meshIDLocal;

  //! material ID within @ref MeshMetaData::materialIndex
  int materialIDLocal;

  //! Object ID in the original file
  int componentID;

  std::vector<MeshTransformNode> children;

  //! Node local transform to the parent frame
  Magnum::Matrix4 T_parent_local;

  MeshTransformNode() {
    meshIDLocal = ID_UNDEFINED;
    materialIDLocal = ID_UNDEFINED;
    componentID = ID_UNDEFINED;
  };

  //! copy constructor which duplicates the @ref MeshTransformNode tree of which
  //! val is the root.
  MeshTransformNode(const MeshTransformNode& val) {
    componentID = val.componentID;
    meshIDLocal = val.meshIDLocal;
    materialIDLocal = val.materialIDLocal;
    T_parent_local = Magnum::Matrix4(val.T_parent_local);
    for (auto& child : val.children) {
      children.push_back(MeshTransformNode(child));
    }
  }
};

// for each scene (mesh file),
// we store the data based on the resource type: 'mesh', 'texture', and
// 'material'. each type may contain a few items;
// thus, we save the start index, and the end index (of each type) as a pair

// in current implementation:
// ptex mesh: meshes_ (1 item), textures_ (0 item), materials_ (0 item);
// instance mesh: meshes_ (1 item), textures_ (0 item), materials_ (0 item);
// gltf_mesh, glb_mesh: meshes_ (i items), textures (j items), materials_ (k
// items), i, j, k = 0, 1, 2 ...

struct MeshMetaData {
  typedef int start;
  typedef int end;
  std::pair<start, end> meshIndex = std::make_pair(ID_UNDEFINED, ID_UNDEFINED);
  std::pair<start, end> textureIndex =
      std::make_pair(ID_UNDEFINED, ID_UNDEFINED);
  std::pair<start, end> materialIndex =
      std::make_pair(ID_UNDEFINED, ID_UNDEFINED);

  MeshTransformNode root;

  MeshMetaData(){};
  MeshMetaData(int meshStart,
               int meshEnd,
               int textureStart = ID_UNDEFINED,
               int textureEnd = ID_UNDEFINED,
               int materialStart = ID_UNDEFINED,
               int materialEnd = ID_UNDEFINED) {
    meshIndex = std::make_pair(meshStart, meshEnd);
    textureIndex = std::make_pair(textureStart, textureEnd);
    materialIndex = std::make_pair(materialStart, materialEnd);
  }
  MeshMetaData(const MeshMetaData& val) {
    meshIndex = val.meshIndex;
    textureIndex = val.textureIndex;
    materialIndex = val.materialIndex;
    root = MeshTransformNode(val.root);
  }
  void setMeshIndices(int meshStart, int meshEnd) {
    meshIndex.first = meshStart;
    meshIndex.second = meshEnd;
  }
  void setTextureIndices(int textureStart, int textureEnd) {
    textureIndex.first = textureStart;
    textureIndex.second = textureEnd;
  }
  void setMaterialIndices(int materialStart, int materialEnd) {
    materialIndex.first = materialStart;
    materialIndex.second = materialEnd;
  }
};

}  // namespace assets
}  // namespace esp
