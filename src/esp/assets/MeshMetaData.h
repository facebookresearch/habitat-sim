// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MESHMETADATA_H_
#define ESP_ASSETS_MESHMETADATA_H_

/** @file
 * @brief Struct @ref esp::assets::MeshTransformNode, Struct @ref
 * esp::assets::MeshMetaData
 */

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include "esp/core/Esp.h"
#include "esp/geo/CoordinateFrame.h"

namespace esp {
namespace assets {

/**
 * @brief Stores meta data for objects with a multi-component transformation
 * hierarchy.
 *
 * Some mesh files include a transformation hierarchy. A @ref
 * MeshTransformNode stores this hierarchy and indices for the meshes and
 * materials at each level such that it can be reused to instance meshes later.
 */
struct MeshTransformNode {
  /** @brief Local mesh index within @ref MeshMetaData::meshIndex. */
  int meshIDLocal{ID_UNDEFINED};

  /** @brief Material key within global material manager. */
  std::string materialID{};  // initializes to default material;

  /** @brief Object index of asset component in the original file. */
  int componentID{ID_UNDEFINED};

  /** @brief The component transformation subtrees with this node as the root.
   */
  std::vector<MeshTransformNode> children;

  /** @brief Node local transform to the parent frame */
  Magnum::Matrix4 transformFromLocalToParent;

  /** @brief Default constructor. */
  MeshTransformNode() = default;

  /** @brief Node name in the original file. */
  std::string name{};
};

/**
 * @brief Stores meta data for an asset possibly containing multiple meshes,
 * materials, textures, and a hierarchy of component transform relationships.
 *
 * As each type of data may contain a few items, we save the start index, and
 * the end index (of each type) as a pair. In current implementation: instance
 * mesh: meshes_ (1 item), textures_ (0 item), materials_ (0 item); gltf_mesh,
 * glb_mesh: meshes_ (i items), textures (j items), materials_ (k items), i, j,
 * k = 0, 1, 2 ...
 */
struct MeshMetaData {
  /** @brief Start index of a data type in the global asset datastructure. */
  typedef int start;

  /** @brief End index of a data type in the global asset datastructure. */
  typedef int end;

  /** @brief Index range (inclusive) of mesh data for the asset in the global
   * asset datastructure. */
  std::pair<start, end> meshIndex = std::make_pair(ID_UNDEFINED, ID_UNDEFINED);

  /** @brief Index range (inclusive) of texture data for the asset in the global
   * asset datastructure. */
  std::pair<start, end> textureIndex =
      std::make_pair(ID_UNDEFINED, ID_UNDEFINED);

  /** @brief Index range (inclusive) of skin data for the asset in the global
   * asset datastructure. */
  std::pair<start, end> skinIndex = std::make_pair(ID_UNDEFINED, ID_UNDEFINED);

  /** @brief The root of the mesh component transformation hierarchy tree which
   * stores the relationship between components of the asset.*/
  MeshTransformNode root;

  /** @brief Default constructor. */
  MeshMetaData() = default;

  /** @brief  Constructor. */
  MeshMetaData(int meshStart,
               int meshEnd,
               int textureStart = ID_UNDEFINED,
               int textureEnd = ID_UNDEFINED) {
    meshIndex = std::make_pair(meshStart, meshEnd);
    textureIndex = std::make_pair(textureStart, textureEnd);
  }

  /**
   * @brief Sets the mesh indices for the asset See @ref
   * ResourceManager::meshes_.
   * @param meshStart First index for asset mesh data in the global mesh
   * datastructure.
   * @param meshEnd Final index for asset mesh data in the global mesh
   * datastructure.
   */
  void setMeshIndices(int meshStart, int meshEnd) {
    meshIndex.first = meshStart;
    meshIndex.second = meshEnd;
  }

  /**
   * @brief Sets the texture indices for the asset. See @ref
   * ResourceManager::textures_.
   * @param textureStart First index for asset texture data in the global
   * texture datastructure.
   * @param textureEnd Final index for asset texture data in the global texture
   * datastructure.
   */
  void setTextureIndices(int textureStart, int textureEnd) {
    textureIndex.first = textureStart;
    textureIndex.second = textureEnd;
  }

  /**
   * @brief Sets the skin indices for the asset. See @ref
   * ResourceManager::skins_.
   * @param skinStart First index for asset skin data in the global
   * skin datastructure.
   * @param skinEnd Final index for asset skin data in the global skin
   * datastructure.
   */
  void setSkinIndices(int skinStart, int skinEnd) {
    skinIndex.first = skinStart;
    skinIndex.second = skinEnd;
  }

  /**
   * @brief Set the root frame orientation based on passed frame
   * @param frame target frame in world space
   */
  void setRootFrameOrientation(const geo::CoordinateFrame& frame) {
    const quatf& transform = frame.rotationFrameToWorld();
    Magnum::Matrix4 R = Magnum::Matrix4::from(
        Magnum::Quaternion(transform).toMatrix(), Magnum::Vector3());
    root.transformFromLocalToParent = R * root.transformFromLocalToParent;
  }
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MESHMETADATA_H_
