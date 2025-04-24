// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_RENDERINSTANCEHELPER_H_
#define ESP_SIM_RENDERINSTANCEHELPER_H_

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

/**
 * @brief A helper for adding and posing render asset instances in the
 * Habitat-sim visual scene. They will appear when rendering, e.g.
 * Simulator::drawObservation. A render asset is generally a 3D model file.
 *
 * The interface here is intentionally minimal to ensure (1) effciency, and (2)
 * that we can easily port the backend to other renderers.
 */
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
   */
  int AddInstance(const std::string& assetFilepath,
                  int semanticId,
                  const Magnum::Vector3& scale = Magnum::Vector3(1.0,
                                                                 1.0,
                                                                 1.0));

  /**
   * @brief Remove all instances from the scene.
   */
  void ClearAllInstances();

  /**
   * @brief Get the number of instances you've added.
   */
  int GetNumInstances();

  /**
   * @brief Set the world poses of all your instances. See the index returned by
   * AddInstance.
   *
   * The raw-pointer interface here is to enable efficient interop with numpy,
   * see SimBindings.cpp.
   *
   * @param positions pointer to position floats
   * @param positionsSize number of position floats; must be GetNumInstances() *
   * 3
   * @param orientations pointer to rotation quaternion floats, as xyzw or wxyz;
   * See also use_xyzw_orientations in the RenderAssetInstance constructor.
   * @param orientationsSize  number of orientation floats; must be
   * GetNumInstances() * 4
   */
  void SetWorldPoses(float* positions,
                     int positionsSize,
                     float* orientations,
                     int orientationsSize);

 private:
  Simulator* sim_ = nullptr;
  std::vector<scene::SceneNode*> instances_;
  bool isXYZW_ = false;

  ESP_SMART_POINTERS(RenderInstanceHelper)
};

}  // namespace sim
}  // namespace esp

#endif
