// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_RENDERINSTANCEHELPER_H_
#define ESP_SIM_RENDERINSTANCEHELPER_H_

#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>
#include <string>
#include <vector>

#include "esp/core/Esp.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace sim {
class Simulator;
}
}  // namespace esp

namespace esp {
namespace sim {

class RenderInstanceHelper {
 public:
  /**
   * @brief RenderInstanceHelper constructor
   *
   * @param sim the Habitat-sim instance where you'll render, e.g.
   * Simulator::drawObservation
   * @param useXYZWOrientations specify the format of the quaternions you'll
   * pass to set_world_poses later; If use_xyzw_orientations=False, we assume
   * wxyz
   */
  RenderInstanceHelper(Simulator& sim, bool useXYZWOrientations = true);

  /**
   * @brief Add an instance of a render asset to the scene. The instance gets an
   * identity pose; change it later using set_world_poses.
   *
   * @param assetFilepath can be for example a .glb or .obj 3D model file
   * @param semanticId used for semantic rendering
   * @param scale An optional local scaling vector applied to this render
   * instance.
   * @param translation An optional local translation vector offset applied to
   * this render instance.
   * @param rotation An optional local rotation vector offset applied to this
   * render instance.
   */
  int AddInstance(const std::string& assetFilepath,
                  int semanticId,
                  const Magnum::Vector3& scale = Magnum::Vector3(1.0, 1.0, 1.0),
                  const Magnum::Vector3& translation = Magnum::Vector3(0),
                  const Magnum::Quaternion& rotation = Magnum::Quaternion());

  /**
   * @brief Remove all instances from the scene.
   */
  void ClearAllInstances();
  int GetNumInstances();
  void SetWorldPoses(float* positions,
                     size_t positionsSize,
                     float* orientations,
                     size_t orientationsSize);

 private:
  Simulator* sim_ = nullptr;
  std::vector<scene::SceneNode*> instances_;
  bool isXYZW_ = false;

  ESP_SMART_POINTERS(RenderInstanceHelper)
};

}  // namespace sim
}  // namespace esp

#endif
