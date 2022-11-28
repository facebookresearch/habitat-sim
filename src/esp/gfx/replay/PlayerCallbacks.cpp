#include "PlayerCallbacks.h"

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {
namespace replay {
PlayerCallbacks createSceneGraphPlayerCallbacks() {
  gfx::replay::PlayerCallbacks callbacks;
  callbacks.deleteAssetInstance_ = [](gfx::replay::GfxReplayNode* node) {
    // TODO: use NodeDeletionHelper to safely delete nodes owned by the Player.
    // the deletion here is unsafe because a Player may persist beyond the
    // lifetime of these nodes.
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    delete sceneNode;
  };
  callbacks.setNodeTransform_ = [](gfx::replay::GfxReplayNode* node,
                                   Magnum::Vector3 translation,
                                   Magnum::Quaternion rotation) {
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    sceneNode->setTranslation(translation);
    sceneNode->setRotation(rotation);
  };
  callbacks.setNodeSemanticId_ = [](gfx::replay::GfxReplayNode* node,
                                    int semanticId) {
    auto* sceneNode = reinterpret_cast<scene::SceneNode*>(node);
    setSemanticIdForSubtree(sceneNode, semanticId);
  };
  return callbacks;
};
}  // namespace replay
}  // namespace gfx
}  // namespace esp
