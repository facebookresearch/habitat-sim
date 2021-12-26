// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_BATCHED_SIMULATOR_H_
#define ESP_BATCHEDSIM_BATCHED_SIMULATOR_H_

#include "esp/physics/bullet/BulletArticulatedObject.h"
#include "esp/sim/Simulator.h"
#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/BpsSceneMapping.h"
#include "esp/batched_sim/EpisodeSet.h"

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
  int numInstances_ = -1;
  int numCollisionSpheres_ = -1;

  // todo: delete artObj
  esp::physics::BulletArticulatedObject* artObj = nullptr;
  BpsSceneMapping* sceneMapping_ = nullptr;
  std::vector<Magnum::Matrix4> nodeTransformFixups;
  std::pair<std::vector<float>, std::vector<float>> jointPositionLimits;

  std::vector<std::vector<Magnum::Vector3>> collisionSphereLocalOriginsByNode_;
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
  void applyActionPenalties(const std::vector<float>& actions);

  Robot* robot_ = nullptr;
  int numEnvs_ = 0;
  RolloutRecord* rollouts_ = nullptr;

  // perf todo: try array of structs for items accessed together
  // size = num robot instances * robot num nodes (num nodes = num links + 1)
  std::vector<int> nodeInstanceIds_;
  // perf todo: try bullet aligned object array
   // size = num robot instances * robot num collision spheres
  std::vector<Mn::Vector3> collisionSphereWorldOrigins_;
  // don't try to pack this with other structs anywhere because it's int8
  std::vector<esp::batched_sim::ColumnGridSource::QueryCacheValue> collisionSphereQueryCaches_;
  // size = num robot instances * robot num instances
  std::vector<glm::mat4x3> nodeNewTransforms_;
  // size = num robot instances
  std::vector<bool> collisionResults_;
  bool areCollisionResultsValid_ = false;

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

  bps3D::Environment& getBpsEnvironment(int envIndex);

  void deleteDebugInstances(); // probably call at start of frame render
  // probably add these every frame ("immediate mode", not persistent)
  int addDebugInstance(const std::string& name, int envIndex, 
    const Magnum::Matrix4& transform=Magnum::Matrix4(Mn::Math::IdentityInit));

 private:
  void reset();
  void stepPhysics();
  void updateCollision();
  // for each robot, update instances for new pose OR undo action
  void postCollisionUpdate(bool useCollisionResults);
  void reverseActionsForEnvironment(int b);

  void calcRewards();
  void randomizeRobotsForCurrentStep();

  void initEpisodeSet();
  EpisodeInstance instantiateEpisode(int b, int episodeIndex);

  BatchedSimulatorConfig config_;
  bool isOkToRender_ = false;
  bool isOkToStep_ = false;
  bool isRenderStarted_ = false;
  Robot robot_;
  RobotInstanceSet robots_;
  int currRolloutStep_ = -1;
  int prevRolloutStep_ = -1;
  RolloutRecord rollouts_;
  std::unique_ptr<esp::sim::Simulator> legacySim_;
  std::unique_ptr<BpsWrapper> bpsWrapper_;
  std::vector<float> actions_;
  int maxRolloutSteps_ = -1;
  RewardCalculationContext rewardContext_;
  std::vector<bool> hackDones_;

  EpisodeSet episodeSet_;
  EpisodeInstanceSet episodeInstanceSet_;

  BpsSceneMapping sceneMapping_;
  std::vector<std::vector<int>> debugInstancesByEnv_;

  ESP_SMART_POINTERS(BatchedSimulator)
};

}  // namespace batched_sim
}  // namespace esp

#endif
