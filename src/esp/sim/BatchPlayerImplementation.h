// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_BATCHPLAYERIMPLEMENTATION_H_
#define ESP_SIM_BATCHPLAYERIMPLEMENTATION_H_

#include <esp/gfx/replay/Player.h>

namespace esp {
namespace gfx_batch {
class Renderer;
}
namespace sim {

class BatchPlayerImplementation
    : public gfx::replay::AbstractPlayerImplementation {
 public:
  BatchPlayerImplementation(gfx_batch::Renderer& renderer,
                            Mn::UnsignedInt sceneId);

 private:
  gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
      const esp::assets::AssetInfo& assetInfo,
      const esp::assets::RenderAssetInstanceCreationInfo& creation) override;

  void deleteAssetInstance(gfx::replay::NodeHandle node) override;

  void deleteAssetInstances(
      const std::unordered_map<gfx::replay::RenderAssetInstanceKey,
                               gfx::replay::NodeHandle>&) override;

  void setNodeTransform(gfx::replay::NodeHandle node,
                        const Mn::Vector3& translation,
                        const Mn::Quaternion& rotation) override;

  void setNodeTransform(gfx::replay::NodeHandle node,
                        const Mn::Matrix4& transform) override;

  Mn::Matrix4 hackGetNodeTransform(gfx::replay::NodeHandle node) const override;

  void changeLightSetup(const esp::gfx::LightSetup& lights) override;

  void createRigInstance(int, const std::vector<std::string>&) override;

  void deleteRigInstance(int) override;

  void setRigPose(int, const std::vector<gfx::replay::Transform>&) override;

  gfx_batch::Renderer& renderer_;
  Mn::UnsignedInt sceneId_;
};
}  // namespace sim
}  // namespace esp

#endif
