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

struct CameraSensorConfig {
  int width = -1;
  int height = -1;
  float hfov = -1.f;
};

struct BpsWrapper {
  BpsWrapper(int gpuId, int numEnvs, const CameraSensorConfig& sensor0);
  ~BpsWrapper();

  std::shared_ptr<bps3D::Scene> scene_;
  std::vector<bps3D::Environment> envs_;
  std::unique_ptr<bps3D::Renderer> renderer_;
  std::unique_ptr<bps3D::AssetLoader> loader_;
};

struct Robot {
  Robot(const std::string& filepath, esp::sim::Simulator* sim, BpsSceneMapping* sceneMapping);
  Robot() = default;

  int numPosVars = -1;
  // todo: delete artObj
  esp::physics::BulletArticulatedObject* artObj = nullptr;
  BpsSceneMapping* sceneMapping_ = nullptr;
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

// represents stage and non-robot objects
class Scene {
 public:
  bps3D::Environment* env_ = nullptr;
  int stageInstance_ = -1;
};

class RobotInstanceSet {
 public:
  RobotInstanceSet() = default;
  RobotInstanceSet(Robot* robot,
                   int numEnvs,
                   std::vector<bps3D::Environment>* envs,
                   RolloutRecord* rollouts);

  void updateLinkTransforms(int currRolloutStep);
  void applyActionPenalties(const std::vector<float>& actions);

  Robot* robot_ = nullptr;
  int numEnvs_ = 0;
  RolloutRecord* rollouts_ = nullptr;

  std::vector<int> nodeInstanceIds_;

  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  std::vector<bps3D::Environment>* envs_;

  std::vector<float> hackRewards_;
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

struct BatchedSimulatorConfig {
  int numEnvs = -1;
  int gpuId = -1;
  CameraSensorConfig sensor0;

  ESP_SMART_POINTERS(BatchedSimulatorConfig);
};

class BatchedSimulator {
 public:
  BatchedSimulator(const BatchedSimulatorConfig& config);

  void startRender();
  void waitForFrame();

  void setActions(std::vector<float>&& actions);
  void autoResetOrStepPhysics();

  bps3D::Renderer& getBpsRenderer();

  const std::vector<float>& getRewards();
  const std::vector<bool>& getDones();

  // For debugging. Sets camera for all envs.
  void setCamera(const Mn::Vector3& camPos, const Mn::Quaternion& camRot);

 private:
  void reset();
  void stepPhysics();

  void calcRewards();
  void randomizeRobotsForCurrentStep();

  void initScenes();

  BatchedSimulatorConfig config_;
  bool isOkToRender_ = false;
  bool isOkToStep_ = false;
  bool isRenderStarted_ = false;
  Robot robot_;
  RobotInstanceSet robots_;
  int currRolloutStep_ = -1;
  RolloutRecord rollouts_;
  std::unique_ptr<esp::sim::Simulator> legacySim_;
  std::unique_ptr<BpsWrapper> bpsWrapper_;
  std::vector<float> actions_;
  int maxRolloutSteps_ = -1;
  RewardCalculationContext rewardContext_;
  std::vector<bool> hackDones_;
  std::vector<Scene> envScenes_;
  BpsSceneMapping sceneMapping_;

  ESP_SMART_POINTERS(BatchedSimulator)
};

}  // namespace batched_sim
}  // namespace esp

#endif
