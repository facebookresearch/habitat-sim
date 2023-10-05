// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_SKINDATA_H_
#define ESP_GFX_SKINDATA_H_

#include <Magnum/Trade/SkinData.h>
#include <esp/scene/SceneNode.h>
#include <unordered_map>

namespace esp {
namespace gfx {

/**
 * @brief Stores skinning data for an asset.
 */
struct SkinData {
  /** @brief Pointer to loaded skin data for the instance. */
  std::shared_ptr<Magnum::Trade::SkinData3D> skin{};
  /** @brief Map of bone names and skin joint IDs. */
  std::unordered_map<std::string, int> boneNameJointIdMap{};
  /** @brief Number of bones that can influence each vertex. */
  int perVertexJointCount{4};
};

/**
 * @brief Stores skinning data for an instance.
 * Contains association information of graphics bones and articulated object
 * links.
 */
struct InstanceSkinData {
  /** @brief Pointer to loaded skin data for the instance. */
  const std::shared_ptr<SkinData>& skinData = nullptr;
  /** @brief Root articulated object node. */
  scene::SceneNode* rootArticulatedObjectNode = nullptr;
  /** @brief Map between skin joint IDs and articulated object transform nodes.
   */
  std::unordered_map<int, const scene::SceneNode*> jointIdToTransformNode{};

  explicit InstanceSkinData(const std::shared_ptr<SkinData>& skinData)
      : skinData(skinData){};
};

/**
 * @brief Contains the nodes that control the articulations of a skinned model
 * instance.
 */
struct Rig {
  /** @brief Nodes that control the articulations of a skinned model instance.
   */
  std::vector<scene::SceneNode*> bones;
  /** @brief Bone name to 'bones' index map. */
  std::unordered_map<std::string, int> boneNames;
};
}  // namespace gfx
}  // namespace esp

#endif
