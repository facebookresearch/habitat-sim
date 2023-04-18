#pragma once

#include <Magnum/Magnum.h>
#include <Magnum/Trade/SkinData.h>
#include <esp/scene/SceneNode.h>
#include <memory>
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
  const std::shared_ptr<SkinData>& skinData;
  /** @brief Map between skin joint IDs and articulated object nodes. */
  std::unordered_map<int, const scene::SceneNode*>
      jointIdToArticulatedObjectNode{};
  /** @brief Map between skin joint IDs and scaled articulated object transform
   * nodes. */
  std::unordered_map<int, const scene::SceneNode*> jointIdToTransformNode{};
  /** @brief Skin joint ID of the root node. */
  int rootJointId{ID_UNDEFINED};

  std::unordered_map<int, Magnum::Matrix4> localTransforms;

  InstanceSkinData(const std::shared_ptr<SkinData>& skinData)
      : skinData(skinData){};
};
}  // namespace gfx
}  // namespace esp
