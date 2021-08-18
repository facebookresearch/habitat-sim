// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_BATCHED_SIMULATOR_H_
#define ESP_BATCHEDSIM_BATCHED_SIMULATOR_H_

#include "BpsSceneMapping.h"

#include "esp/physics/bullet/BulletArticulatedObject.h"
#include "esp/sim/Simulator.h"

#include <bps3D.hpp>

namespace esp {
namespace batched_sim {

struct BpsWrapper {
  BpsWrapper();
  ~BpsWrapper();

  std::shared_ptr<bps3D::Scene> scene_;
  std::vector<bps3D::Environment> envs_;
  std::unique_ptr<bps3D::Renderer> renderer_;
  std::unique_ptr<bps3D::AssetLoader> loader_;
};

struct Robot {
  Robot(const std::string& filepath, esp::sim::Simulator* sim);
  Robot() = default;

  int numPosVars = -1;
  // todo: delete artObj
  esp::physics::BulletArticulatedObject* artObj = nullptr;
  BpsSceneMapping sceneMapping;
  std::vector<Magnum::Matrix4> nodeTransformFixups;
};

class RobotInstanceSet {
 public:
  RobotInstanceSet() = default;
  RobotInstanceSet(Robot* robot,
                   int batchSize,
                   std::vector<bps3D::Environment>* envs);

  void updateLinkTransforms();

  Robot* robot_ = nullptr;
  int batchSize_ = 0;
  std::vector<float> jointPositions_;
  std::vector<Magnum::Matrix4> rootTransforms_;
  std::vector<Magnum::Matrix4> nodeTransforms_;
  std::vector<int> nodeInstanceIds_;

  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  std::vector<bps3D::Environment>* envs_;
};

struct SimInstanceSet {
  RobotInstanceSet robots;
};

class BatchedSimulator {
 public:
  BatchedSimulator();

  void stepPhysics();
  void startRender();
  void waitForFrame();
  bps3D::Renderer* debugGetBpsRenderer();

 private:
  Robot robot_;
  SimInstanceSet simInstances_;
  std::unique_ptr<esp::sim::Simulator> legacySim_;
  std::unique_ptr<BpsWrapper> bpsWrapper_;
};

}  // namespace batched_sim
}  // namespace esp

#endif
