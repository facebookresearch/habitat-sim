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
  std::pair<std::vector<float>, std::vector<float>> jointPositionLimits;
};

struct RolloutRecord {
  RolloutRecord() = default;
  RolloutRecord(int numRolloutSteps,
                int numEnvs,
                int numJointVars,
                int numNodes);

  int numRolloutSteps_ = 0;
  std::vector<float>
      jointPositions_;       // [numRolloutSteps * numEnvs * numJointVars]
  std::vector<float> yaws_;  // [numRolloutSteps * numEnvs]
  std::vector<Mn::Vector2> positions_;           // [numRolloutSteps * numEnvs]
  std::vector<Magnum::Matrix4> rootTransforms_;  // [numRolloutSteps * numEnvs]
  std::vector<Magnum::Matrix4>
      nodeTransforms_;  // [numRolloutSteps * numEnvs * numNodes]

  std::vector<float> rewards_;  // [numRolloutSteps * numEnvs]
};

class RobotInstanceSet {
 public:
  RobotInstanceSet() = default;
  RobotInstanceSet(Robot* robot,
                   int numEnvs,
                   std::vector<bps3D::Environment>* envs,
                   RolloutRecord* rollouts);

  void updateLinkTransforms(int currRolloutStep);

  Robot* robot_ = nullptr;
  int numEnvs_ = 0;
  RolloutRecord* rollouts_ = nullptr;

  std::vector<int> nodeInstanceIds_;

  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  std::vector<bps3D::Environment>* envs_;
};

struct RewardCalculationContext {
  RewardCalculationContext() = default;
  RewardCalculationContext(const Robot* robot,
                           int numEnvs,
                           RolloutRecord* rollouts);

  void calcRewards(int currRolloutStep, int bStart, int bEnd);

  esp::physics::BulletArticulatedObject* artObj_ = nullptr;
  std::unique_ptr<esp::sim::Simulator> legacySim_;
  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;
  const Robot* robot_ = nullptr;
  int numEnvs_ = -1;
  RolloutRecord* rollouts_ = nullptr;
};

class BatchedSimulator {
 public:
  BatchedSimulator();

  void setActions(std::vector<float>&& actions);
  void stepPhysics();
  void startRender();
  void waitForFrame();
  bps3D::Renderer& getBpsRenderer();

  void calcRewards();

 private:
  void randomizeRobotsForCurrentStep();

  Robot robot_;
  RobotInstanceSet robots_;
  int currRolloutStep_ = -1;
  RolloutRecord rollouts_;
  std::unique_ptr<esp::sim::Simulator> legacySim_;
  std::unique_ptr<BpsWrapper> bpsWrapper_;
  std::vector<float> actions_;
  int maxRolloutSteps_ = -1;
  RewardCalculationContext rewardContext_;

  ESP_SMART_POINTERS(BatchedSimulator)
};

}  // namespace batched_sim
}  // namespace esp

#endif
