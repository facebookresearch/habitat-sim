#ifndef ESP_GFX_REPLAY_PLAYERCALLBACKS_H_
#define ESP_GFX_REPLAY_PLAYERCALLBACKS_H_

#include "Magnum/Magnum.h"

#include <functional>
#include <vector>

namespace esp {
namespace assets {
struct AssetInfo;
struct RenderAssetInstanceCreationInfo;
}  // namespace assets
namespace gfx {
struct LightInfo;
using LightSetup = std::vector<LightInfo>;
namespace replay {

// Represents either a scene::SceneNode* or batch renderer entity index.
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

  using SetNodeTransform =
      std::function<void(GfxReplayNode*, Magnum::Vector3, Magnum::Quaternion)>;

  using SetNodeSemanticId = std::function<void(GfxReplayNode*, int)>;

  LoadAndCreateRenderAssetInstance loadAndCreateRenderInstance_;
  DeleteAssetInstance deleteAssetInstance_;
  SetNodeTransform setNodeTransform_;
  SetNodeSemanticId setNodeSemanticId_;
  ChangeLightSetup changeLightSetup_;
};

/**
 * @brief Factory method that instantiates PlayerCallbacks boilerplate for
 * callbacks that handle scene graph nodes. Note that this does not provide an
 * implementation for all callbacks.
 */
PlayerCallbacks createSceneGraphPlayerCallbacks();

}  // namespace replay
}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_REPLAY_PLAYERCALLBACKS_H_
