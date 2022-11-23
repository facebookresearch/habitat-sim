#ifndef ESP_GFX_REPLAY_PLAYERCALLBACKS_H_
#define ESP_GFX_REPLAY_PLAYERCALLBACKS_H_

#include "Magnum/Magnum.h"
#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace gfx {
namespace replay {

// Represents either a esp::scene::SceneNode or batch renderer index.
class GfxReplayNode;

/**
 * @brief Callbacks that contain the implementation for gfx::replay::Player.
 */
struct PlayerCallbacks {
  using LoadAndCreateRenderAssetInstance = std::function<GfxReplayNode*(
      const esp::assets::AssetInfo&,
      const esp::assets::RenderAssetInstanceCreationInfo&)>;

  using ChangeLightSetup = std::function<void(const gfx::LightSetup& lights)>;

  using DeleteAssetInstance = std::function<void(GfxReplayNode*)>;

  using SetAssetInstanceTransform =
      std::function<void(GfxReplayNode*, Magnum::Vector3, Magnum::Quaternion)>;

  using SetAssetInstanceSemanticId = std::function<void(GfxReplayNode*, int)>;

  LoadAndCreateRenderAssetInstance loadAndCreateRenderInstance_;
  DeleteAssetInstance deleteAssetInstance_;
  SetAssetInstanceTransform setAssetInstanceTransform_;
  SetAssetInstanceSemanticId setAssetInstanceSemanticId_;
  ChangeLightSetup changeLightSetup_;
};

/**
 * @brief Factory method to create a PlayerCallbacks that handles
 * esp::scene::SceneNode
 */
static inline PlayerCallbacks createSceneGraphPlayerCallbacks() {
  gfx::replay::PlayerCallbacks callbacks;
  callbacks.deleteAssetInstance_ = [](gfx::replay::GfxReplayNode* node) {
    // TODO: use NodeDeletionHelper to safely delete nodes owned by the Player.
    // the deletion here is unsafe because a Player may persist beyond the
    // lifetime of these nodes.
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    delete sceneNode;
  };
  callbacks.setAssetInstanceTransform_ = [](gfx::replay::GfxReplayNode* node,
                                            Magnum::Vector3 translation,
                                            Magnum::Quaternion rotation) {
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    sceneNode->setTranslation(translation);
    sceneNode->setRotation(rotation);
  };
  callbacks.setAssetInstanceSemanticId_ = [](gfx::replay::GfxReplayNode* node,
                                             int semanticId) {
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    setSemanticIdForSubtree(sceneNode, semanticId);
  };
  return callbacks;
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_REPLAY_PLAYERCALLBACKS_H_
