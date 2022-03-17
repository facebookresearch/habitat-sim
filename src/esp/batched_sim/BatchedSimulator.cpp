// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchedSimulator.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/PlacementHelper.h"
#include "esp/batched_sim/ProfilingScope.h"

#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/json.h"

// #include <bps3D.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>

#include <mutex>

#ifndef NDEBUG
#define ENABLE_DEBUG_INSTANCES
#endif

namespace esp {
namespace batched_sim {

namespace {

static const char* serializeCollectionFilepath_ = "../data/replicacad_composite.collection.json";

static constexpr glm::mat4x3 identityGlMat_ = glm::mat4x3(
  1.f, 0.f, 0.f,
  0.f, 1.f, 0.f,
  0.f, 0.f, 1.f, 
  0.f, 0.f, 0.f);

Mn::Vector3 toMagnumVector3(const btVector3& src) {
  return {src.x(), src.y(), src.z()};
}

Mn::Vector4 toMagnumVector4(const btVector3& src, float w = 0.f) {
  return {src.x(), src.y(), src.z(), w};
}

Mn::Matrix4 toMagnumMatrix4(const btTransform& btTrans) {
  const auto& basis = btTrans.getBasis();
  Mn::Matrix4 tmp2{btTrans};
  return tmp2;
}

std::string getMeshNameFromURDFVisualFilepath(const std::string& filepath) {
  // "../meshes/base_link.dae" => "base_link"
  auto index0 = filepath.rfind('/');
  if (index0 == -1) {
    // this is okay; the substring will start at 0
  }
  auto index1 = filepath.rfind('.');
  BATCHED_SIM_ASSERT(index1 != -1);

  auto retval = filepath.substr(index0 + 1, index1 - index0 - 1);
  return retval;
}

#ifdef ENABLE_DEBUG_INSTANCES
bool isPairedDebugEnv(const BatchedSimulatorConfig& config, int b) {
  return config.doPairedDebugEnvs && (b % 2 == 1);
}

bool shouldDrawDebugForEnv(const BatchedSimulatorConfig& config, int b, int substep) {
  return isPairedDebugEnv(config, b) && (substep % config.numSubsteps == 0);
}
bool shouldAddColumnGridDebugVisualsForEnv(const BatchedSimulatorConfig& config, int b) {
  return isPairedDebugEnv(config, b);
}
bool disableRobotVisualsForEnv(const BatchedSimulatorConfig& config, int b) {
  return isPairedDebugEnv(config, b);
}
bool disableFreeObjectVisualsForEnv(const BatchedSimulatorConfig& config, int b) {
  return isPairedDebugEnv(config, b);
}
bool disableStageVisualsForEnv(const BatchedSimulatorConfig& config, int b) {
  return isPairedDebugEnv(config, b);
}
#else
constexpr bool isPairedDebugEnv(const BatchedSimulatorConfig&, int) {
  return false;
}
constexpr bool shouldDrawDebugForEnv(const BatchedSimulatorConfig&, int, int) {
  return false;
}
constexpr bool shouldAddColumnGridDebugVisualsForEnv(const BatchedSimulatorConfig&, int) {
  return false;
}
constexpr bool disableRobotVisualsForEnv(const BatchedSimulatorConfig&, int) {
  return false;
}
constexpr bool disableFreeObjectVisualsForEnv(const BatchedSimulatorConfig&, int) {
  return false;
}
constexpr bool disableStageVisualsForEnv(const BatchedSimulatorConfig&, int) {
  return false; // false;
}
#endif


}  // namespace

void BatchedSimulator::randomizeRobotsForCurrentStep() {
  BATCHED_SIM_ASSERT(currRolloutSubstep_ >= 0);
  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  float* yaws = &safeVectorGet(rollouts_.yaws_, currRolloutSubstep_ * numEnvs);
  Mn::Vector2* positions = &safeVectorGet(rollouts_.positions_,currRolloutSubstep_ * numEnvs);
  float* jointPositions =
      &safeVectorGet(rollouts_.jointPositions_,currRolloutSubstep_ * numEnvs * numPosVars);

  auto random = core::Random(/*seed*/ 3);

  for (int b = 0; b < numEnvs; b++) {
    // hand-authored start location and yaw range that works for stages 0-12
    positions[b] = Mn::Vector2(2.59f, 0.f);
    yaws[b] = random.uniform_float(-float(Mn::Rad(Mn::Deg(135.f))), -float(Mn::Rad(Mn::Deg(45.f))));
        
    // temp move robot out of scene (hide/disable robot)
    // positions[b].y() -= 1000.f;

    for (int j = 0; j < robot_.numPosVars; j++) {
      auto& pos = jointPositions[b * robot_.numPosVars + j];
      pos = 0.f;
      pos = Mn::Math::clamp(pos, robot_.jointPositionLimits.first[j],
                            robot_.jointPositionLimits.second[j]);
    }

    // 7 shoulder, + is down
    // 8 twist, + is twist to right
    // 9 elbow, + is down
    // 10 elbow twist, + is twst to right
    // 11 wrist, + is down
    jointPositions[b * robot_.numPosVars + 9] = float(Mn::Rad(Mn::Deg(90.f)));
  }

  if (config_.doPairedDebugEnvs) {
    for (int b = 1; b < numEnvs; b += 2) {
      BATCHED_SIM_ASSERT(isPairedDebugEnv(config_, b));
      positions[b] = positions[b - 1];
      yaws[b] = yaws[b - 1];
      for (int j = 0; j < robot_.numPosVars; j++) {
        auto& pos = jointPositions[b * robot_.numPosVars + j];
        pos = jointPositions[(b - 1) * robot_.numPosVars + j];
      }   
    }   
  }
}

RobotInstanceSet::RobotInstanceSet(Robot* robot,
                                   const BatchedSimulatorConfig* config,
                                   std::vector<bps3D::Environment>* envs,
                                   RolloutRecord* rollouts)
    : config_(config), envs_(envs), robot_(robot), rollouts_(rollouts) {
  const int numLinks = robot->artObj->getNumLinks();
  const int numNodes = numLinks + 1;  // include base
  const int numEnvs = config_->numEnvs;
  const int batchNumPosVars = robot_->numPosVars * numEnvs;
  const int batchNumNodes = numNodes * numEnvs;

  nodeInstanceIds_.resize(batchNumNodes, -1);
  nodeNewTransforms_.resize(batchNumNodes);
  collisionSphereWorldOrigins_.resize(robot->numCollisionSpheres_ * numEnvs);
  collisionSphereQueryCaches_.resize(robot->numCollisionSpheres_ * numEnvs, 0);

  const auto* mb = robot_->artObj->btMultiBody_.get();

  int baseInstanceIndex = 0;
  for (int b = 0; b < numEnvs; b++) {
    auto& env = (*envs_)[b];

    // sloppy: pass -1 to getLinkVisualSceneNodes to get base
    for (int i = -1; i < numLinks; i++) {
      const auto& link = robot->artObj->getLink(i);  // -1 gets base link

      const auto& visualAttachments = link.visualAttachments_;

      BATCHED_SIM_ASSERT(visualAttachments.size() <= 1);

      int instanceId = -1;
      if (!visualAttachments.empty()) {
        const std::string linkVisualFilepath = visualAttachments[0].second;
        const std::string nodeName =
            getMeshNameFromURDFVisualFilepath(linkVisualFilepath);
        auto instanceBlueprint =
            robot_->sceneMapping_->findInstanceBlueprint(nodeName);

        if (!disableRobotVisualsForEnv(*config_, b)) {
          instanceId = env.addInstance(instanceBlueprint.meshIdx_, 
            instanceBlueprint.mtrlIdx_, identityGlMat_);
        } else {
          // sloppy; avoid leaving as -1 because that indicates no visual model attached to this link
          instanceId = -2;
        }
      } else {
        // these are camera links
        // sloppy: we should avoid having these items in the nodeInstanceIds_
        // array
      }

      const auto nodeIndex = i + 1;  // 0 is base
      const auto instanceIndex = baseInstanceIndex + nodeIndex;
      nodeInstanceIds_[instanceIndex] = instanceId;
    }

    baseInstanceIndex += numNodes;
  }

  collisionResults_.resize(numEnvs, false);

  robotInstances_.resize(numEnvs);
}

void RobotInstanceSet::applyActionPenalties(const std::vector<float>& actions) {
  int numEnvs = config_->numEnvs;

  float yawPenaltyScale = 0.01;
  float forwardPenaltyScale = 0.01;
  float jointPosPenaltyScale = 0.01;

  int actionIndex = 0;
  for (int b = 0; b < numEnvs; b++) {
    // hackRewards_[b] = 0.f; // hack disable state-based reward

    float deltaYaw = actions[actionIndex++];
    constexpr float maxDeltaYaw = float(Mn::Rad(Mn::Deg(15.f)));
    hackRewards_[b] -=
        Mn::Math::max(0.f, Mn::Math::abs(deltaYaw) - maxDeltaYaw) *
        yawPenaltyScale;

    float deltaForward = actions[actionIndex++];
    constexpr float maxDeltaForward = 0.2f;
    hackRewards_[b] -=
        Mn::Math::max(0.f, Mn::Math::abs(deltaForward) - maxDeltaForward) *
        forwardPenaltyScale;

    for (int j = 0; j < robot_->numPosVars; j++) {
      float deltaJointPos = actions[actionIndex++];
      constexpr float maxDeltaJointPos = 0.1f;
      hackRewards_[b] -=
          Mn::Math::max(0.f, Mn::Math::abs(deltaJointPos) - maxDeltaJointPos) *
          jointPosPenaltyScale;
    }
  }
  BATCHED_SIM_ASSERT(actionIndex == actions.size());
}

void BatchedSimulator::reverseActionsForEnvironment(int b) {
  BATCHED_SIM_ASSERT(prevRolloutSubstep_ != -1);
  const int numEnvs = config_.numEnvs;
  const int numPosVars = robot_.numPosVars;

  const float* prevYaws = &rollouts_.yaws_[prevRolloutSubstep_ * numEnvs];
  float* yaws = &rollouts_.yaws_[currRolloutSubstep_ * numEnvs];
  const Mn::Vector2* prevPositions =
      &rollouts_.positions_[prevRolloutSubstep_ * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutSubstep_ * numEnvs];
  Mn::Matrix4* rootTransforms =
      &rollouts_.rootTransforms_[currRolloutSubstep_ * numEnvs];
  const float* prevJointPositions =
      &rollouts_.jointPositions_[prevRolloutSubstep_ * numEnvs * numPosVars];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutSubstep_ * numEnvs * numPosVars];

  yaws[b] = prevYaws[b];
  positions[b] = prevPositions[b];
  int baseJointIndex = b * robot_.numPosVars;
  for (int j = 0; j < robot_.numPosVars; j++) {
    auto& pos = jointPositions[baseJointIndex + j];
    const auto& prevPos = prevJointPositions[baseJointIndex + j];
    pos = prevPos;
  }
}

void RobotInstanceSet::updateLinkTransforms(int currRolloutSubstep, bool updateForPhysics, bool updateForRender) {
  //ProfilingScope scope("BSim updateLinkTransforms");

  BATCHED_SIM_ASSERT(updateForPhysics || updateForRender);
  
  // esp::gfx::replay::Keyframe debugKeyframe;
  int numLinks = robot_->artObj->getNumLinks();
  int numNodes = numLinks + 1;
  int numEnvs = config_->numEnvs;
  int numPosVars = robot_->numPosVars;

  if (updateForPhysics) {
    areCollisionResultsValid_ = false;
  }

  auto* mb = robot_->artObj->btMultiBody_.get();
  int posCount = 0;

  const float* yaws = &rollouts_->yaws_[currRolloutSubstep * numEnvs];
  const Mn::Vector2* positions =
      &rollouts_->positions_[currRolloutSubstep * numEnvs];
  const float* jointPositions =
      &rollouts_->jointPositions_[currRolloutSubstep * numEnvs * numPosVars];
  Mn::Matrix4* rootTransforms =
      &rollouts_->rootTransforms_[currRolloutSubstep * numEnvs];

  for (int b = 0; b < config_->numEnvs; b++) {

    auto& robotInstance = robotInstances_[b];
    // perf todo: simplify this
    rootTransforms[b] =
        Mn::Matrix4::translation(
            Mn::Vector3(positions[b].x(), 0.f, positions[b].y())) *
        Mn::Matrix4::rotation(Mn::Rad(yaws[b]), {0.f, 1.f, 0.f}) *
        Mn::Matrix4::rotation(Mn::Deg(-90.f), {1.f, 0.f, 0.f});

    btTransform tr{rootTransforms[b]};
    mb->setBaseWorldTransform(tr);

    for (int i = 0; i < numLinks; ++i) {
      auto& link = mb->getLink(i);
      // optimization todo: find correct subset of links
      if (link.m_posVarCount > 0) {
        mb->setJointPosMultiDof(i,
                                const_cast<float*>(&jointPositions[posCount]));
        posCount += link.m_posVarCount;
      }
    }

    mb->forwardKinematics(scratch_q_, scratch_m_);

    auto& env = (*envs_)[b];
    int baseInstanceIndex = b * numNodes;
    const int baseSphereIndex = b * robot_->numCollisionSpheres_;

    // extract link transforms
    // todo: update base node
    {
      for (int i = -1; i < numLinks; i++) {
        const auto nodeIndex = i + 1;  // 0 is base
        const auto instanceIndex = baseInstanceIndex + nodeIndex;

        int instanceId = nodeInstanceIds_[instanceIndex];
        if (instanceId == -1) {
          // todo: avoid every calculating this link transform
          continue;
        }

        if (!updateForRender) {
          if (robot_->collisionSpheresByNode_[nodeIndex].empty()
            && robot_->gripperLink_ != i) {
            continue;
          }
        }

        // todo: avoid btTransform copy for case of i != -1
        const auto btTrans = i == -1 ? mb->getBaseWorldTransform()
                                     : mb->getLink(i).m_cachedWorldTransform;
        Mn::Matrix4 mat = toMagnumMatrix4(btTrans);

        auto tmp = robot_->nodeTransformFixups[nodeIndex];
        // auto vec = tmp[3];
        // const float scale = (float)b / (numEnvs_ - 1);
        // tmp[3] = Mn::Vector4(vec.xyz() * scale, 1.f);
        mat = mat * tmp;

        if (robot_->gripperLink_ == i) {
          robotInstance.cachedGripperLinkMat_ = mat;
        }

        if (updateForPhysics) {
          // perf todo: loop through collision spheres (and look up link id), instead of this sparse way here
          // compute collision sphere transforms
          for (const auto& localSphereIdx : robot_->collisionSpheresByNode_[nodeIndex]) {
            const auto& sphere = safeVectorGet(robot_->collisionSpheres_, localSphereIdx);
            collisionSphereWorldOrigins_[baseSphereIndex + localSphereIdx] = 
              mat.transformPoint(sphere.origin);
          }
        }

        if (updateForRender) {
          BATCHED_SIM_ASSERT(instanceIndex < nodeNewTransforms_.size());
          nodeNewTransforms_[instanceIndex] = toGlmMat4x3(mat);

          if (robot_->cameraAttachNode_ == nodeIndex) {
            robotInstance.cameraAttachNodeTransform_ = mat;
          }
        }
#if 0
        esp::gfx::replay::Transform absTransform{
            mat.translation(),
            Magnum::Quaternion::fromMatrix(mat.rotationShear())};

        esp::gfx::replay::RenderAssetInstanceState state{
          .absTransform=absTransform,
          .semanticId=-1
        };
        const int instanceId = b * 1000 + i; // temp hack
        debugKeyframe.stateUpdates.emplace_back(instanceId, state);
#endif
      }
    }
  }

#if 0
  {
    {
      rapidjson::Document document(rapidjson::kObjectType);
      rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
      esp::io::addMember(document, "debugKeyframe", debugKeyframe, allocator);
      esp::io::writeJsonToFile(document, "temp.replay.json");
    }
  }
#endif

}


void BatchedSimulator::updatePythonEnvironmentState() {

  const auto currEpisodeStep = currRolloutSubstep_ / config_.numSubsteps;

  const int numEnvs = config_.numEnvs;
  const Mn::Vector2* positions = &safeVectorGet(rollouts_.positions_, currRolloutSubstep_ * numEnvs);
  const float* yaws = &safeVectorGet(rollouts_.yaws_, currRolloutSubstep_ * numEnvs);

  for (int b = 0; b < config_.numEnvs; b++) {

    const auto& robotInstance = safeVectorGet(robots_.robotInstances_, b);
    auto& envState = safeVectorGet(pythonEnvStates_, b);

    envState.ee_pos = robotInstance.cachedGripperLinkMat_.translation();
    envState.did_finish_episode_and_reset = (currEpisodeStep == 0);
    envState.finished_episode_success = false; // temp hardcoded
    envState.episode_step_idx = currEpisodeStep;
    // todo: do logical or over all substeps
    envState.did_collide = robots_.collisionResults_[b];
    // temp hardcoded episode goal data
    envState.goal_pos = Mn::Vector3(0.f, 0.f, 0.f); // todo: only update this on episode change
    envState.target_obj_idx = 0; // todo: only update this on episode change
    // envState.obj_positions // this is updated incrementally
    envState.robot_position = Mn::Vector3(positions[b].x(), 0.f, positions[b].y());
    envState.robot_yaw = yaws[b];
  }
}

void BatchedSimulator::addSphereDebugInstance(const std::string& name, int b, const Magnum::Vector3& spherePos, float radius) {
  auto mat = Mn::Matrix4::translation(spherePos)
    * Mn::Matrix4::scaling(Mn::Vector3(radius, radius, radius));
  addDebugInstance(name, b, mat);
}

void BatchedSimulator::addBoxDebugInstance(const std::string& name, int b, const Magnum::Vector3& pos, 
  const Magnum::Quaternion& rotation, const Magnum::Range3D& aabb, float pad, bool showBackfaces) {

  auto adjustedAabb = aabb.padded({pad, pad, pad});

  Mn::Matrix4 mat = Mn::Matrix4::from(rotation.toMatrix(), pos);
  Mn::Matrix4 localToBox = Mn::Matrix4::translation(adjustedAabb.center())
    * Mn::Matrix4::scaling(adjustedAabb.size() * 0.5 * (showBackfaces ? -1.f : 1.f));
  auto adjustedMat = mat * localToBox;
  addDebugInstance(name, b, adjustedMat);
}



void BatchedSimulator::updateGripping() {
  //ProfilingScope scope("BSim updateGripping");

  const int numEnvs = config_.numEnvs;

  for (int b = 0; b < numEnvs; b++) {
  
    auto& env = bpsWrapper_->envs_[b];
    auto& robotInstance = robots_.robotInstances_[b];

    if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {

      const auto& gripperMat = robotInstance.cachedGripperLinkMat_;
      auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
      const auto gripperQueryWorldOrigin = gripperMat.transformPoint(
        robot_.gripperQueryOffset_);
      const auto gripperQueryRadius = robot_.gripperQueryRadius_;
      
      // draw preview of grip attempt
      if (robotInstance.grippedFreeObjectIndex_ == -1) {

        int grippedFreeObjectIndex = episodeInstance.colGrid_.contactTest(
          gripperQueryWorldOrigin, gripperQueryRadius);
        
        // highlight the object that could be gripped if we attempted
        if (grippedFreeObjectIndex != -1) {
          const auto& obs = episodeInstance.colGrid_.getObstacle(grippedFreeObjectIndex);
          addBoxDebugInstance(
            "cube_blue_wireframe",
            b, obs.pos, obs.invRotation.invertedNormalized(), *obs.aabb, 0.01);
        }

        // show query sphere
        addSphereDebugInstance(
            "sphere_blue_wireframe",
            b, gripperQueryWorldOrigin, gripperQueryRadius);
      }

      // draw line down from gripper query or held object
      constexpr float w = 0.005f;
      addBoxDebugInstance(
        "cube_blue",
        b, gripperQueryWorldOrigin, Mn::Quaternion(Mn::Math::IdentityInit),
        Mn::Range3D({-w, -1.f, -w}, {w, -0.04, w}));
    }

    // don't attempt grip if there was a collision (there's currently a bug where
    // cachedGripperLinkMat_ is wrong)
    // sloppy: not clear if this field is up-to-date or one-frame-stale
    if (robots_.collisionResults_[b]) {
      continue;
    }

    if (robotInstance.doAttemptGrip_) {
      // sloppy: doing this gripping logic here since it's the only time we have access to
      // the link transform as a Matrix4.

      // todo: assert this is valid
      BATCHED_SIM_ASSERT(robotInstance.grippedFreeObjectIndex_ == -1);
      const auto& gripperMat = robotInstance.cachedGripperLinkMat_;
      auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
      const auto gripperQueryWorldOrigin = gripperMat.transformPoint(
        robot_.gripperQueryOffset_);
      const auto gripperQueryRadius = robot_.gripperQueryRadius_;
      int grippedFreeObjectIndex = episodeInstance.colGrid_.contactTest(
        gripperQueryWorldOrigin, gripperQueryRadius);
      if (grippedFreeObjectIndex != -1) {
        removeFreeObjectFromCollisionGrid(b, grippedFreeObjectIndex);
        robotInstance.grippedFreeObjectIndex_ = grippedFreeObjectIndex;
        robotInstance.doAttemptGrip_ = false;
        episodeInstance.movedFreeObjectIndexes_.push_back(grippedFreeObjectIndex);

        recentStats_.numGrips_++;
      }

      recentStats_.numGripAttempts_++;
    }

    if (robotInstance.doAttemptDrop_) {
      BATCHED_SIM_ASSERT(robotInstance.grippedFreeObjectIndex_ != -1);

      // defer this: making object upright on drop
      //// find nearest up axis from list stored in freeObject
      //// rotate to match target up axis

      // search for xy, starting at dropPos (reuse this for episode generation/settling)
      //// snap to stage (use all spheres?)
      //// compute collision spheres, test collision against free objects
      // add to col grid
      // search for historical robot pose that is coll free

      int freeObjectIndex = robotInstance.grippedFreeObjectIndex_;
      const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
      const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
      const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
      const auto& columnGridSet = stageFixedObject.columnGridSet_;
      auto heldObjMat = getHeldObjectTransform(b);
      
      const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
        episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
      const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

      constexpr int maxFailedPlacements = 3;
      PlacementHelper placementHelper(columnGridSet, episodeInstance.colGrid_, 
        serializeCollection_, random_, maxFailedPlacements);
      bool success = placementHelper.place(heldObjMat, freeObject);

      if (success) {
        const auto rotationQuat = Mn::Quaternion::fromMatrix(heldObjMat.rotation());
        reinsertFreeObject(b, freeObjectIndex, heldObjMat.translation(), rotationQuat);
      } else {
        if (!disableFreeObjectVisualsForEnv(config_, b)) {
          // hack: remove object from scene visually
          const auto glMat = toGlmMat4x3(Mn::Matrix4::translation({0.f, -10000.f, 0.f})); 
          int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
          env.updateInstanceTransform(instanceId, glMat);
        }

        recentStats_.numFailedDrops_++;
      }
      robotInstance.grippedFreeObjectIndex_ = -1;
      robotInstance.doAttemptDrop_ = false;

      recentStats_.numDrops_++;
    }
  }
}


void BatchedSimulator::updateCollision() {
  //ProfilingScope scope("BSim updateCollision");

  const int numEnvs = config_.numEnvs;

  BATCHED_SIM_ASSERT(!robots_.areCollisionResultsValid_);
  BATCHED_SIM_ASSERT(robots_.collisionResults_.size() == numEnvs);

  robots_.areCollisionResultsValid_ = true;

#ifdef ENABLE_DEBUG_INSTANCES
  std::vector<bool> sphereHits(robot_.numCollisionSpheres_ * numEnvs, false);
  std::vector<bool> heldObjectHits(numEnvs, false);
  std::vector<bool> freeObjectHits(episodeSet_.maxFreeObjects_, false);
#else
  std::vector<bool> sphereHits;
  std::vector<bool> heldObjectHits;
  std::vector<bool> freeObjectHits;
#endif

  for (int b = 0; b < numEnvs; b++) {

    const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
    const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
    const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
    const auto& columnGridSet = stageFixedObject.columnGridSet_;
    const int baseSphereIndex = b * robot_.numCollisionSpheres_;
    const auto& robotInstance = robots_.robotInstances_[b];
    bool hit = false;

    // perf todo: if there was a hit last frame, cache that sphere and test it first here
    for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
      const int sphereIndex = baseSphereIndex + s;
      // perf todo: reconsider query cache usage; maybe one per link or one per robot
      auto& queryCache = robots_.collisionSphereQueryCaches_[sphereIndex];
      const auto& spherePos = robots_.collisionSphereWorldOrigins_[sphereIndex];
      const int radiusIdx = robot_.collisionSpheres_[s].radiusIdx;

      bool thisSphereHit = columnGridSet.contactTest(radiusIdx, spherePos, &queryCache);

      // If pairedDebugEnv, we want to visualize all collision sphere hits, not just the 
      // first one. This is misleading debug visualization, since we actually early-out
      // after the first hit.
      // (my reasoning is that it will be confusing to casual viewers to see *any* green 
      // spheres in collision)
      if (thisSphereHit) {
        hit = true;
        if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
          sphereHits[baseSphereIndex + s] = thisSphereHit;
        } else {
          break;
        }
      }
    }

    if (!hit && robotInstance.grippedFreeObjectIndex_ != -1) {
      ColumnGridSource::QueryCacheValue grippedObjectQueryCache = 0;
      auto mat = getHeldObjectTransform(b);
      const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
        episode.firstFreeObjectSpawnIndex_ + robotInstance.grippedFreeObjectIndex_);
      const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
      for (const auto& sphere : freeObject.collisionSpheres_) {
        const auto& sphereLocalOrigin = sphere.origin;
        auto sphereWorldOrigin = mat.transformPoint(sphereLocalOrigin);
        bool thisSphereHit = columnGridSet.contactTest(sphere.radiusIdx, sphereWorldOrigin, &grippedObjectQueryCache);
        // todo: proper debug-drawing of hits for held object spheres
        if (thisSphereHit) {
          hit = true;
          if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
            heldObjectHits[b] = thisSphereHit;
          } else {
            break;
          }
        }
      }
    }    

    robots_.collisionResults_[b] = hit;
  }

  // test against free objects
  for (int b = 0; b < numEnvs; b++) {

    if (robots_.collisionResults_[b] && !shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
      // already had a hit against column grid so don't test free objects
      continue;
    }

    const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
    const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
    const int baseSphereIndex = b * robot_.numCollisionSpheres_;
    bool hit = false;
    auto& env = bpsWrapper_->envs_[b];
    const auto& robotInstance = robots_.robotInstances_[b];

    // perf todo: if there was a hit last frame, cache that sphere and test it first here
    for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
      const int sphereIndex = baseSphereIndex + s;
      const auto& spherePos = safeVectorGet(robots_.collisionSphereWorldOrigins_, sphereIndex);
      const int radiusIdx = safeVectorGet(robot_.collisionSpheres_, s).radiusIdx;
      const auto sphereRadius = getCollisionRadius(serializeCollection_, radiusIdx);

      int hitFreeObjectIndex = episodeInstance.colGrid_.contactTest(spherePos, sphereRadius);
      if (hitFreeObjectIndex != -1) {
        hit = true;
        if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
          sphereHits[baseSphereIndex + s] = true;
          freeObjectHits[hitFreeObjectIndex] = true;
        } else {
          break;
        }
      }
    }

    if (!hit && robotInstance.grippedFreeObjectIndex_ != -1) {
      int grippedObjectQueryCache = 0;
      auto mat = getHeldObjectTransform(b);
      const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
        episode.firstFreeObjectSpawnIndex_ + robotInstance.grippedFreeObjectIndex_);
      const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
      for (const auto& sphere : freeObject.collisionSpheres_) {
        const auto& sphereLocalOrigin = sphere.origin;
        auto sphereWorldOrigin = mat.transformPoint(sphereLocalOrigin);
        const auto sphereRadius = getCollisionRadius(serializeCollection_, sphere.radiusIdx);

        int hitFreeObjectIndex = episodeInstance.colGrid_.contactTest(sphereWorldOrigin, sphereRadius);
        if (hitFreeObjectIndex != -1) {
          hit = true;
          if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
            heldObjectHits[b] = true;
            freeObjectHits[hitFreeObjectIndex] = true;
          } else {
            break;
          }
        }       
      } 
    }    

    // render free objects to debug env, colored by collision result
    if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
      for (int freeObjectIndex = 0; freeObjectIndex < episode.numFreeObjectSpawns_; freeObjectIndex++) {
        if (episodeInstance.colGrid_.isObstacleDisabled(freeObjectIndex)) {
          continue;
        }
        const auto& obs = episodeInstance.colGrid_.getObstacle(freeObjectIndex);
        addBoxDebugInstance(
          freeObjectHits[freeObjectIndex] ? "cube_pink_wireframe" : "cube_orange_wireframe",
          b, obs.pos, obs.invRotation.invertedNormalized(), *obs.aabb);

        #if 0
        // render collision spheres
        const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
          episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
        const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
        Mn::Matrix4 mat = Mn::Matrix4::from(
            obs.invRotation.invertedNormalized().toMatrix(), obs.pos);
        for (const auto& sphere : freeObject.collisionSpheres_) {
          const auto& sphereLocalOrigin = sphere.origin;
          const float sphereRadius = getCollisionRadius(serializeCollection_, sphere.radiusIdx);
          auto sphereWorldOrigin = mat.transformPoint(sphereLocalOrigin);
          addSphereDebugInstance("sphere_blue_wireframe", b, sphereWorldOrigin, sphereRadius);
        }
        #endif
        

        freeObjectHits[freeObjectIndex] = false; // clear for next env
      }
    }    

    robots_.collisionResults_[b] = robots_.collisionResults_[b] || hit;
  }

  for (int b = 0; b < numEnvs; b++) {
    // render collision spheres for debug env, colored by collision result
    if (shouldDrawDebugForEnv(config_, b, currRolloutSubstep_)) {
      const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
      const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
      const auto& robotInstance = robots_.robotInstances_[b];

      const int baseSphereIndex = b * robot_.numCollisionSpheres_;
      for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
        const int sphereIndex = baseSphereIndex + s;
        const auto& spherePos = safeVectorGet(robots_.collisionSphereWorldOrigins_, sphereIndex);
        const int radiusIdx = safeVectorGet(robot_.collisionSpheres_, s).radiusIdx;
        const auto sphereRadius = getCollisionRadius(serializeCollection_, radiusIdx);
        addSphereDebugInstance(sphereHits[baseSphereIndex + s] ? "sphere_pink_wireframe" : "sphere_green_wireframe",
          b, spherePos, sphereRadius);    
        sphereHits[baseSphereIndex + s] = false; // clear for next env
      }

      if (robotInstance.grippedFreeObjectIndex_ != -1) {
        int grippedObjectQueryCache = 0;
        auto mat = getHeldObjectTransform(b);
        const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
          episode.firstFreeObjectSpawnIndex_ + robotInstance.grippedFreeObjectIndex_);
        const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
        for (const auto& sphere : freeObject.collisionSpheres_) {
          const auto& sphereLocalOrigin = sphere.origin;
          auto sphereWorldOrigin = mat.transformPoint(sphereLocalOrigin);
          const auto sphereRadius = getCollisionRadius(serializeCollection_, sphere.radiusIdx);
          addSphereDebugInstance(
            heldObjectHits[b] ? "sphere_pink_wireframe" : "sphere_blue_wireframe", 
            b, sphereWorldOrigin, sphereRadius);    
        }
      }
    }

    if (robots_.collisionResults_[b]) {
      // todo: more robust handling of this, maybe at episode-load time
      ESP_CHECK(currRolloutSubstep_ > 1, "For env " << b << ", the robot is in collision on the first step of the episode.");
      recentStats_.numStepsInCollision_++;
    }
  }

  recentStats_.numSteps_ += numEnvs;
}

void BatchedSimulator::postCollisionUpdate() {
  //ProfilingScope scope("BSim postCollisionUpdate");

  const int numEnvs = config_.numEnvs;

  BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);

  for (int b = 0; b < numEnvs; b++) {
    if (robots_.collisionResults_[b]) {
      reverseActionsForEnvironment(b);
    } else {
      // nothing to do
    }
  }
}

void BatchedSimulator::updateRenderInstances(bool forceUpdate) {
  //ProfilingScope scope("BSim updateRenderInstances");

  const int numEnvs = config_.numEnvs;
  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  if (!forceUpdate) {
    BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);
  }

  for (int b = 0; b < numEnvs; b++) {

    auto& robotInstance = robots_.robotInstances_[b];
    auto& env = bpsWrapper_->envs_[b];

    // temp hack: we don't currently have bookeeping to know if a robot moved over
    // several substeps, so we assume it did here. perf todo: fix this
    bool didRobotMove = forceUpdate || 
      (!robots_.collisionResults_[b] || config_.numSubsteps > 1);

    // update robot links and camera
    if (didRobotMove) {
      // no collision, so let's update the robot instance instances

      int baseInstanceIndex = b * numNodes;

      for (int i = -1; i < numLinks; i++) {
        const auto nodeIndex = i + 1;  // 0 is base
        const auto instanceIndex = baseInstanceIndex + nodeIndex;

        int instanceId = robots_.nodeInstanceIds_[instanceIndex];
        if (instanceId == -1) {
          continue;
        }

        if (!disableRobotVisualsForEnv(config_, b)) {
          const auto& glMat = robots_.nodeNewTransforms_[instanceIndex];
          env.updateInstanceTransform(instanceId, glMat);
        }
      }

      // todo: cleaner handling of camera update that doesn't depend on attachment or collision
      if (robot_.cameraAttachNode_ != -1) {
        auto cameraTransform = robotInstance.cameraAttachNodeTransform_ * robot_.cameraAttachTransform_;
        auto glCameraNewInvTransform = glm::inverse(toGlmMat4(cameraTransform));
        env.setCameraView(glCameraNewInvTransform);
      }
    }

    // update gripped free object
    if (didRobotMove && robotInstance.grippedFreeObjectIndex_ != -1) {

      if (!disableFreeObjectVisualsForEnv(config_, b)) {
        int freeObjectIndex = robotInstance.grippedFreeObjectIndex_;
        auto mat = getHeldObjectTransform(b);
        glm::mat4x3 glMat = toGlmMat4x3(mat); 
        int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
        env.updateInstanceTransform(instanceId, glMat);
      }
    }
  }

  // add debug ground lines
  #if 0
  BATCHED_SIM_ASSERT(!config_.doPairedDebugEnv);
  {
    for (int b = 0; b < numEnvs; b++) {

      auto& env = bpsWrapper_->envs_[b];

      // sloppy: lots of logic here duplicated from BatchedSimulator::instantiateEpisode
      auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
      const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);

      for (int freeObjectIndex = 0; freeObjectIndex < episode.numFreeObjectSpawns_; freeObjectIndex++) {

        int globalFreeObjectIndex = b * episodeSet_.maxFreeObjects_ + freeObjectIndex;

        const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
          episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
        const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

        auto instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
        const glm::mat4x3& glMat = env.getInstanceTransform(instanceId);
        Mn::Vector3 pos = getMagnumTranslation(glMat);

        constexpr float groundY = -0.05;
        const auto widthVec = Mn::Vector3(0.005f, 0.f, 0.005f);
        Mn::Range3D aabb(Mn::Vector3(pos.x(), groundY, pos.z()) - widthVec, pos + widthVec);
        Mn::Matrix4 localToBox = Mn::Matrix4::translation(aabb.center())
          * Mn::Matrix4::scaling(-aabb.size() * 0.5);
        addDebugInstance("cube_green", b, localToBox);
      }
    }
  }
  #endif

  areRenderInstancesUpdated_ = true;
}


Mn::Matrix4 BatchedSimulator::getHeldObjectTransform(int b) const {

  auto& robotInstance = robots_.robotInstances_[b];
  BATCHED_SIM_ASSERT(robotInstance.grippedFreeObjectIndex_ != -1);

  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);

  const int freeObjectIndex = robotInstance.grippedFreeObjectIndex_;

  const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
    episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
  const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

  // todo: better decide how to orient gripped object
  const int startRotationIndex = freeObject.heldRotationIndex_;
  const auto& rotation = safeVectorGet(freeObject.startRotations_, startRotationIndex);

  Mn::Matrix4 linkToGripper = Mn::Matrix4::translation(robot_.gripperQueryOffset_);
  Mn::Matrix4 toOrientedObject = Mn::Matrix4::from(rotation, Mn::Vector3(Mn::Math::ZeroInit));
  // todo: offset to a few centimeters from the edge, instead of the center
  Mn::Matrix4 toObjectCenter = Mn::Matrix4::translation(-freeObject.aabb_.center());

  auto mat = robotInstance.cachedGripperLinkMat_ * linkToGripper * toOrientedObject * toObjectCenter;
  return mat;
}


Robot::Robot(const serialize::Collection& serializeCollection, esp::sim::Simulator* sim, BpsSceneMapping* sceneMapping) {

  ESP_CHECK(!serializeCollection.robots.empty(), "no robot found in collection.json");
  const serialize::Robot& serializeRobot = serializeCollection.robots.front();
  const std::string filepath = serializeRobot.urdfFilepath;

  // todo: delete object on destruction
  auto managedObj =
      sim->getArticulatedObjectManager()->addBulletArticulatedObjectFromURDF(
          filepath);

  sceneMapping_ = sceneMapping;

  artObj = static_cast<esp::physics::BulletArticulatedObject*>(
      managedObj->hackGetBulletObjectReference().get());

  jointPositionLimits = artObj->getJointPositionLimits();

  int numLinks = artObj->getNumLinks();
  int numNodes = numLinks + 1;
  nodeTransformFixups.resize(numNodes);

  // Sloppy: this is needed for correctness; I think it's because I preprocessed
  // from GLB to bps with the wrong axes specified on the command line.
  const auto globalFixup =
      Mn::Matrix4::rotation(Mn::Deg(90.f), {1.f, 0.f, 0.f});

  auto linkIds = artObj->getLinkIds();
  int numInstances = 0;

  collisionSpheresByNode_.resize(numNodes);

  for (int i = -1; i < numLinks; i++) {
    const auto nodeIndex = i + 1;           // 0 is base
    BATCHED_SIM_ASSERT(i == -1 || i == linkIds[i]);
    const auto& link = artObj->getLink(i);  // -1 gets base link
    linkIndexByName_[link.linkName] = i;
    const auto& visualAttachments = link.visualAttachments_;
    BATCHED_SIM_ASSERT(visualAttachments.size() <= 1);
    if (!visualAttachments.empty()) {
      const auto* sceneNode = visualAttachments[0].first;
      int nodeIndex = i + 1;  // 0 for base
      // This transform comes from the visual origin specified in the URDF;
      // it is essentially an additional transform to apply to the visual mesh.
      const auto tmp = sceneNode->transformation();
      nodeTransformFixups[nodeIndex] = tmp * globalFixup;
      numInstances++;
    }
  }

  numInstances_ = numInstances;

  numPosVars = artObj->getJointPositions().size();
  BATCHED_SIM_ASSERT(numPosVars > 0);

  updateFromSerializeCollection(serializeCollection);
}

void Robot::updateFromSerializeCollection(const serialize::Collection& serializeCollection) {

  const auto& serializeRobot = serializeCollection.robots.front();

  ESP_CHECK(linkIndexByName_.count(serializeRobot.gripper.attachLinkName), 
    "gripper attach link " << serializeRobot.gripper.attachLinkName 
    << " from collection.json not found in robot URDF");

  gripperLink_ = linkIndexByName_.at(serializeRobot.gripper.attachLinkName);
  gripperQueryOffset_ = serializeRobot.gripper.offset;
  gripperQueryRadius_ = serializeRobot.gripper.radius;

  int numLinks = artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  int numCollisionSpheres = 0;

  BATCHED_SIM_ASSERT(!collisionSpheresByNode_.empty());
  for (auto& nodeSpheres : collisionSpheresByNode_) {
    nodeSpheres.clear();
  }
  collisionSpheres_.clear();

  for (const auto& serLink : serializeRobot.links) {

    ESP_CHECK(linkIndexByName_.count(serLink.linkName), "link " << serLink.linkName 
      << " from collection.json not found in robot URDF");
    int linkIndex = linkIndexByName_[serLink.linkName];
    int nodeIndex = linkIndex + 1;

    auto& nodeSpheres = collisionSpheresByNode_[nodeIndex];

    for (const auto& serSphere : serLink.collisionSpheres) {
      nodeSpheres.push_back(collisionSpheres_.size());
      int radiusIdx = getCollisionRadiusIndex(serializeCollection, serSphere.radius);
      collisionSpheres_.push_back({serSphere.origin, radiusIdx});
      numCollisionSpheres++;
    }
  }
#if 0
  std::vector<std::string> armLinks = {
    "shoulder_lift_link",
    "shoulder_pan_link",
    "upperarm_roll_link",
    "elbow_flex_link",
    "forearm_roll_link",
    "wrist_flex_link",
    "wrist_roll_link",
    //"gripper_link"
  };

  constexpr int numArmLinkSpheres = 4;
  constexpr float armLinkLength = 0.15f;

  std::vector<std::string> bodyLinks = {
    "torso_lift_link",
    "base_link"
  };

  constexpr int numBodyLinkSpheres = 16;
  constexpr float bodyRingRadius = 0.28f;
  constexpr float bodyRingLocalHeight = 0.6f;
  constexpr float bodyOffsetX = -0.2f;

  for (int i = -1; i < numLinks; i++) {
    const auto nodeIndex = i + 1;           // 0 is base
    const auto& link = artObj->getLink(i);  // -1 gets base link

    // temp hard-coded spheres
    if (std::find(armLinks.begin(), armLinks.end(), link.linkName) != armLinks.end()) {
      for (int j = 0; j < numArmLinkSpheres; j++) {
        float offset = (float)j / numArmLinkSpheres * armLinkLength;
        collisionSphereLocalOriginsByNode_[nodeIndex].push_back({offset, 0.f, 0.f});
        numCollisionSpheres++;
      }
    }

    else if (std::find(bodyLinks.begin(), bodyLinks.end(), link.linkName) != bodyLinks.end()) {
      // add spheres in a circle
      for (int j = 0; j < numBodyLinkSpheres; j++) {
        Mn::Deg angle = (float)j / numBodyLinkSpheres * Mn::Deg(360);
        collisionSphereLocalOriginsByNode_[nodeIndex].push_back(
          Mn::Vector3(Mn::Math::sin(angle) + bodyOffsetX, bodyRingLocalHeight, Mn::Math::cos(angle)) * bodyRingRadius);
        numCollisionSpheres++;
      }
    }

    if (link.linkName == "gripper_link") {
      gripperLink_ = i;
      gripperQueryOffset_ = Mn::Vector3(0.3f, 0.f, 0.f);
      gripperQueryRadius_ = 0.1f;
    }
  }
#endif

  numCollisionSpheres_ = numCollisionSpheres;
}

BpsWrapper::BpsWrapper(int gpuId, int numEnvs, bool includeDepth, bool includeColor, 
  const CameraSensorConfig& sensor0) {
  glm::u32vec2 out_dim(
      sensor0.width,
      sensor0.height);  // see also rollout_test.py, python/rl/agent.py
  BATCHED_SIM_ASSERT(gpuId != -1);

  bps3D::RenderMode mode {};
  if (includeDepth) {
      mode |= bps3D::RenderMode::Depth;
  }
  if (includeColor) {
      mode |= bps3D::RenderMode::UnlitRGB;
  }

  renderer_ = std::make_unique<bps3D::Renderer>(bps3D::RenderConfig{
      gpuId, 1, uint32_t(numEnvs), out_dim.x, out_dim.y, false, mode});

  loader_ = std::make_unique<bps3D::AssetLoader>(renderer_->makeLoader());
  const std::string filepath =
      "../data/bps_data/replicacad_composite/replicacad_composite.bps";
  scene_ = loader_->loadScene(filepath);

  const Mn::Vector3 camPos(Mn::Math::ZeroInit);
  const Mn::Quaternion camRot(Mn::Math::IdentityInit);
  glm::mat4 world_to_camera(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < numEnvs; b++) {
    glm::mat4 view = world_to_camera;
    auto env = renderer_->makeEnvironment(scene_, view, sensor0.hfov, 0.f, 0.01,
                                          1000.f);
    envs_.emplace_back(std::move(env));
  }
}

BpsWrapper::~BpsWrapper() {
  // must destroy these in a particular order
  envs_.clear();
  scene_ = nullptr;
  loader_ = nullptr;
  renderer_ = nullptr;
}

BatchedSimulator::BatchedSimulator(const BatchedSimulatorConfig& config) {
  config_ = config;

  sceneMapping_ = BpsSceneMapping::loadFromFile(
      "../data/bps_data/replicacad_composite/replicacad_composite.bps.mapping.json");

  serializeCollection_ = serialize::Collection::loadFromFile(serializeCollectionFilepath_);

  bpsWrapper_ = std::make_unique<BpsWrapper>(config_.gpuId, config_.numEnvs, 
    config_.includeDepth, config_.includeColor, config_.sensor0);

#ifdef ENABLE_DEBUG_INSTANCES
  debugInstancesByEnv_.resize(config_.numEnvs);
#endif

  initEpisodeSet();

  esp::sim::SimulatorConfiguration simConfig{};
  simConfig.activeSceneName = "NONE";
  simConfig.enablePhysics = true;
  simConfig.createRenderer = false;
  simConfig.loadRenderAssets = false;
  // simConfig.physicsConfigFile = physicsConfigFile;

  legacySim_ = esp::sim::Simulator::create_unique(simConfig);

  robot_ = Robot(serializeCollection_, legacySim_.get(), &sceneMapping_);

  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  const int numEnvs = config_.numEnvs;
  robots_ = RobotInstanceSet(&robot_, &config_, &bpsWrapper_->envs_, &rollouts_);

  // see also python/rl/agent.py
  const int numOtherActions = 1; // doAttemptGrip/doAttemptDrop
  const int numJointDegrees = robot_.numPosVars;
  const int numBaseDegrees = 2;  // rotate and move-forward/back
  actionDim_ = numOtherActions + numJointDegrees + numBaseDegrees;

  int batchNumActions = (actionDim_) * numEnvs;
  actions_.resize(batchNumActions, 0.f);

  ESP_CHECK(config_.maxEpisodeLength > 1, "BatchedSimulatorConfig::maxEpisodeLength must be > 1");
  // For an episode length of 2, we produce an observation immediately after resetting,
  // then we take one step (config_.numSubsteps substeps), and we produce one more observation.
  maxRolloutSubsteps_ = (config_.maxEpisodeLength - 1) * config_.numSubsteps + 1;
  rollouts_ =
      RolloutRecord(maxRolloutSubsteps_, numEnvs, robot_.numPosVars, numNodes);

  rewardContext_ = RewardCalculationContext(&robot_, numEnvs, &rollouts_);
  robots_.hackRewards_.resize(numEnvs, 0.f);
  hackDones_.resize(numEnvs, false);
  pythonEnvStates_.resize(numEnvs);

  currRolloutSubstep_ =
      -1;  // trigger auto-reset on first call to autoResetOrStepPhysics
  prevRolloutSubstep_ = -1;
  isOkToRender_ = false;
  isOkToStep_ = false;
  isRenderStarted_ = false;

  // default camera
  {
    std::string cameraAttachLinkName = "torso_lift_link";
    Mn::Vector3 camPos = {-0.536559, 1.16173, 0.568379};
    Mn::Quaternion camRot = {{-0.26714, -0.541109, -0.186449}, 0.775289};
    const auto cameraMat = Mn::Matrix4::from(camRot.toMatrix(), camPos);
    attachCameraToLink(cameraAttachLinkName, cameraMat);
  }

  if (config_.doAsyncPhysicsStep) {
    physicsThread_ = std::thread(&BatchedSimulator::physicsThreadFunc, this, 0, config_.numEnvs);
    areRenderInstancesUpdated_ = false;
  }

  if (config_.doPairedDebugEnvs) {
    for (int b = 0; b < numEnvs; b++) {
      if (shouldAddColumnGridDebugVisualsForEnv(config_, b)) {
        debugRenderColumnGrids(b);
      }
    }
  }
}

void BatchedSimulator::close() {
  if (config_.doAsyncPhysicsStep) {
    if (physicsThread_.joinable()) {
      waitAsyncStepPhysics();
      signalKillPhysicsThread();
      physicsThread_.join();
    }
  }
  // todo: more close stuff?
}


void BatchedSimulator::setCamera(const Mn::Vector3& camPos, const Mn::Quaternion& camRot) {

  const int numEnvs = config_.numEnvs;

  robot_.cameraAttachNode_ = -1; // unattach camera

  glm::mat4 world_to_camera(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < numEnvs; b++) {
    auto& env = bpsWrapper_->envs_[b];
    env.setCameraView(world_to_camera);
  }
}

void BatchedSimulator::attachCameraToLink(const std::string& linkName, const Magnum::Matrix4& mat) {
  
  const int numEnvs = config_.numEnvs;
  
  BATCHED_SIM_ASSERT(robot_.linkIndexByName_.count(linkName));

  int linkIndex = robot_.linkIndexByName_[linkName];

  robot_.cameraAttachNode_ = linkIndex + 1;
  robot_.cameraAttachTransform_ = mat;

  // todo: threadsafe
  for (int b = 0; b < numEnvs; b++) {
    auto& env = bpsWrapper_->envs_[b];
    auto& robotInstance = robots_.robotInstances_[b];
    auto cameraTransform = robotInstance.cameraAttachNodeTransform_ * robot_.cameraAttachTransform_;
    auto glCameraNewInvTransform = glm::inverse(toGlmMat4(cameraTransform));
    env.setCameraView(glCameraNewInvTransform);
  }
}


bps3D::Environment& BatchedSimulator::getBpsEnvironment(int envIndex) {
  BATCHED_SIM_ASSERT(envIndex < config_.numEnvs);
  return bpsWrapper_->envs_[envIndex];
}

void BatchedSimulator::instantiateEpisode(int b, int episodeIndex) {

  auto& env = getBpsEnvironment(b);
  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);

  BATCHED_SIM_ASSERT(episodeInstance.episodeIndex_ == -1);
  episodeInstance.episodeIndex_ = episodeIndex;

  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeIndex);
  const auto& stageBlueprint = 
    safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex).instanceBlueprint_;
  if (!disableStageVisualsForEnv(config_, b)) {
    episodeInstance.stageFixedObjectInstanceId_ = env.addInstance(
      stageBlueprint.meshIdx_, stageBlueprint.mtrlIdx_, identityGlMat_);
  }

  {
    const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
    const auto& columnGrid = stageFixedObject.columnGridSet_.getColumnGrid(0);

    // todo: find extents for entire EpisodeSet, not just this specific columnGrid
    constexpr int maxBytes = 1000 * 1024;
    // this is tuned assuming a building-scale simulation with household-object-scale obstacles
    constexpr float maxGridSpacing = 0.5f;
    episodeInstance.colGrid_ = CollisionBroadphaseGrid(getMaxCollisionRadius(serializeCollection_), 
      columnGrid.minX, columnGrid.minZ,
      columnGrid.getMaxX(), columnGrid.getMaxZ(),
      maxBytes, maxGridSpacing);
  }

  for (int freeObjectIndex = 0; freeObjectIndex < episode.numFreeObjectSpawns_; freeObjectIndex++) {

    int globalFreeObjectIndex = b * episodeSet_.maxFreeObjects_ + freeObjectIndex;

    const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
      episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
    const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
    const auto& blueprint = freeObject.instanceBlueprint_;
    const auto& rotation = safeVectorGet(freeObject.startRotations_, freeObjectSpawn.startRotationIndex_);

    spawnFreeObject(b, freeObjectIndex, /*reinsert*/false);
  }

  episodeInstance.debugNumColGridObstacleInstances_ = episodeInstance.colGrid_.getNumObstacleInstances();

}


void BatchedSimulator::resetEpisodeInstance(int b) {
  //ProfilingScope scope("BSim resetEpisodeInstance");

  auto& env = getBpsEnvironment(b);

  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  auto& robotInstance = robots_.robotInstances_[b];

  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);

  for (int i = 0; i < episodeInstance.movedFreeObjectIndexes_.size(); i++) {

    int freeObjectIndex = episodeInstance.movedFreeObjectIndexes_[i];
    // disabled objects including currently-held and removed-from-scene
    if (!episodeInstance.colGrid_.isObstacleDisabled(freeObjectIndex)) {
      removeFreeObjectFromCollisionGrid(b, freeObjectIndex); 
    }
    // reinsert at spawn location
    spawnFreeObject(b, freeObjectIndex, /*reinsert*/true);
  }
  episodeInstance.movedFreeObjectIndexes_.clear();

  // sloppy: reset robotInstance here
  if (robotInstance.grippedFreeObjectIndex_ != -1) {
    // nothing to do here.
    // held free object was already in movedFreeObjectIndexes_
    robotInstance.grippedFreeObjectIndex_ = -1;
  }
  robotInstance.doAttemptDrop_ = false;
  robotInstance.doAttemptGrip_ = false;

  // todo: ensure spawnFreeObject also updates bps instance

  BATCHED_SIM_ASSERT(episodeInstance.debugNumColGridObstacleInstances_ == 
    episodeInstance.colGrid_.getNumObstacleInstances());
}




void BatchedSimulator::spawnFreeObject(int b, int freeObjectIndex, bool reinsert) {

  auto& env = getBpsEnvironment(b);
  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
  const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
    episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
  const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
  const auto& blueprint = freeObject.instanceBlueprint_;
  const auto& rotation = safeVectorGet(freeObject.startRotations_, freeObjectSpawn.startRotationIndex_);

  // create bps instance
  if (!reinsert) {
    // sloppy: this matrix gets created differently on episode reset
    Mn::Matrix4 mat = Mn::Matrix4::from(
        rotation, freeObjectSpawn.startPos_);
    glm::mat4x3 glMat = toGlmMat4x3(mat); 
    if (!disableFreeObjectVisualsForEnv(config_, b)) {
      int instanceId = env.addInstance(blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
      // store the first free object's bps instanceId and assume the rest will be contiguous
      if (freeObjectIndex == 0) {
        episodeInstance.firstFreeObjectInstanceId_ = instanceId;
      }
      BATCHED_SIM_ASSERT(instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex));
    }
  }

  // temp sloppy convert to quat here
  const auto rotationQuat = Mn::Quaternion::fromMatrix(rotation);
  if (!reinsert) {
    int16_t obsIndex = episodeInstance.colGrid_.insertObstacle(
      freeObjectSpawn.startPos_, rotationQuat, &freeObject.aabb_);
    BATCHED_SIM_ASSERT(obsIndex == freeObjectIndex);
  } else {
    reinsertFreeObject(b, freeObjectIndex, freeObjectSpawn.startPos_, rotationQuat);
  }
}

void BatchedSimulator::removeFreeObjectFromCollisionGrid(int b, int freeObjectIndex) {

  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  episodeInstance.colGrid_.disableObstacle(freeObjectIndex);
}

int BatchedSimulator::getFreeObjectBpsInstanceId(int b, int freeObjectIndex) const {

  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  return episodeInstance.firstFreeObjectInstanceId_ + freeObjectIndex;
}

void BatchedSimulator::reinsertFreeObject(int b, int freeObjectIndex,
  const Magnum::Vector3& pos, const Magnum::Quaternion& rotation) {

  auto& env = getBpsEnvironment(b);
  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  episodeInstance.colGrid_.reinsertObstacle(freeObjectIndex, pos, rotation);

  if (!disableFreeObjectVisualsForEnv(config_, b)) {
    // sloppy quat to Matrix3x3
    Mn::Matrix4 mat = Mn::Matrix4::from(
        rotation.toMatrix(), pos);
    glm::mat4x3 glMat = toGlmMat4x3(mat); 
    int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
    env.updateInstanceTransform(instanceId, glMat);
  }
}

void BatchedSimulator::initEpisodeSet() {

  const int numEnvs = config_.numEnvs;

  // generate exactly as many episodes as envs (this is not required)
  episodeSet_ = generateBenchmarkEpisodeSet(numEnvs, sceneMapping_, serializeCollection_);

  episodeInstanceSet_.episodeInstanceByEnv_.resize(numEnvs);
  for (int b = 0; b < numEnvs; b++) {
    auto episodeIndex = b * episodeSet_.episodes_.size() / numEnvs; // distribute episodes across instances
    if (config_.doPairedDebugEnvs) {
      episodeIndex /= 2;
    }
    instantiateEpisode(b, episodeIndex);
  }
}

void BatchedSimulator::setActions(std::vector<float>&& actions) {
  //ProfilingScope scope("BSim setActions");

  ESP_CHECK(actions.size() == actions_.size(),
            "BatchedSimulator::setActions: input dimension should be " +
                std::to_string(actions_.size()) + ", not " +
                std::to_string(actions.size()));
  const int numEnvs = config_.numEnvs;

  if (config_.forceRandomActions) {
    for (auto& action : actions_) {
      action = random_.uniform_float(-1.f, 1.f);
    }
  } else {
    actions_ = std::move(actions);
  }

  if (config_.doPairedDebugEnvs) {
    // copy actions from non-debug env to its paired debug env
    // every other env, see isPairedDebugEnv
    for (int b = 1; b < numEnvs; b += 2) {
      BATCHED_SIM_ASSERT(isPairedDebugEnv(config_, b));
      for (int actionIdx = 0; actionIdx < actionDim_; actionIdx++) {
        actions_[b * actionDim_ + actionIdx] = actions_[(b - 1) * actionDim_ + actionIdx];
      }
    }
  }
}

void BatchedSimulator::reset() {
  ProfilingScope scope("reset episodes");

  // we shouldn't be in the middle of rendering or stepping physics
  ESP_CHECK(!isPhysicsThreadActive(), "Don't call reset during async physics step");
  ESP_CHECK(!isRenderStarted_, "Don't call reset during async render");

  // all episodes are done; set done flag and reset
  std::fill(hackDones_.begin(), hackDones_.end(), true);

  int numEnvs = bpsWrapper_->envs_.size();

  currRolloutSubstep_ = 0;
  prevRolloutSubstep_ = -1;
  randomizeRobotsForCurrentStep();
  robots_.updateLinkTransforms(currRolloutSubstep_, /*updateforPhysics*/ false, /*updateForRender*/ true);
  std::fill(robots_.hackRewards_.begin(), robots_.hackRewards_.end(), 0.f);
  for (int b = 0; b < numEnvs; b++) {
    resetEpisodeInstance(b);
  }
  updateRenderInstances(/*forceUpdate*/true);

  updatePythonEnvironmentState();

  isOkToRender_ = true;
  isOkToStep_ = true;
  recentStats_.numEpisodes_ += numEnvs;
}

#if 0
void BatchedSimulator::autoResetOrStepPhysics() {
  // ProfilingScope scope("BSim autoResetOrStepPhysics");

  BATCHED_SIM_ASSERT(!config_.doAsyncPhysicsStep);

  deleteDebugInstances();

  if (currRolloutSubstep_ == -1 || currRolloutSubstep_ == maxRolloutSubsteps_ - 1) {
    // all episodes are done; set done flag and reset
    reset();
  } else {
    if (currRolloutSubstep_ == 0) {
      std::fill(hackDones_.begin(), hackDones_.end(), false);
    } else {
      BATCHED_SIM_ASSERT(!hackDones_[0]);
    }
    stepPhysics();
    calcRewards();
    updateRenderInstances(/*forceUpdate*/false);
  }
}
#endif



void BatchedSimulator::startAsyncStepPhysics(std::vector<float>&& actions) {
  ProfilingScope scope("start async physics");

  BATCHED_SIM_ASSERT(!isPhysicsThreadActive());
  BATCHED_SIM_ASSERT(config_.doAsyncPhysicsStep);
  BATCHED_SIM_ASSERT(currRolloutSubstep_ != -1);
  BATCHED_SIM_ASSERT(currRolloutSubstep_ < maxRolloutSubsteps_ - 1);

  setActions(std::move(actions));

  deleteDebugInstances();

  if (currRolloutSubstep_ == 0) {
    std::fill(hackDones_.begin(), hackDones_.end(), false);
  } else {
    BATCHED_SIM_ASSERT(!hackDones_[0]);
  }

  // send message to physicsThread_
  signalStepPhysics();
}

void BatchedSimulator::stepPhysics() {
  ProfilingScope scope("step physics");

  BATCHED_SIM_ASSERT(config_.numSubsteps > 0);
  for (int i = 0; i < config_.numSubsteps; i++) {
    substepPhysics();
  }

  robots_.updateLinkTransforms(currRolloutSubstep_, /*updateforPhysics*/ false, /*updateForRender*/ true);
}

void BatchedSimulator::substepPhysics() {
  ProfilingScope scope("substep");
  BATCHED_SIM_ASSERT(isOkToStep_);

  constexpr float stickyGrabDropThreshold = 0.85f;
  constexpr float minBaseMovement = -0.05f;
  constexpr float maxBaseMovement = 0.1f;
  // beware not all joints are angular
  constexpr float maxAbsYawAngle = float(Mn::Rad(Mn::Deg(5.f)));
  constexpr float maxAbsJointAngle = float(Mn::Rad(Mn::Deg(5.f)));

  BATCHED_SIM_ASSERT(currRolloutSubstep_ < maxRolloutSubsteps_ - 1);
  prevRolloutSubstep_ = currRolloutSubstep_;
  currRolloutSubstep_++;

  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  auto& robots = robots_;

  int actionIndex = 0;

  const float* prevYaws = &rollouts_.yaws_[prevRolloutSubstep_ * numEnvs];
  float* yaws = &rollouts_.yaws_[currRolloutSubstep_ * numEnvs];
  const Mn::Vector2* prevPositions =
      &rollouts_.positions_[prevRolloutSubstep_ * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutSubstep_ * numEnvs];
  Mn::Matrix4* rootTransforms =
      &rollouts_.rootTransforms_[currRolloutSubstep_ * numEnvs];
  const float* prevJointPositions =
      &rollouts_.jointPositions_[prevRolloutSubstep_ * numEnvs * numPosVars];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutSubstep_ * numEnvs * numPosVars];

  // stepping code
  for (int b = 0; b < numEnvs; b++) {
    auto& robotInstance = robots_.robotInstances_[b];
    const float grabDropAction = actions_[actionIndex++];

    // sticky grip behavior
    if (robotInstance.grippedFreeObjectIndex_ == -1) {
      if (robotInstance.doAttemptGrip_ && grabDropAction < -stickyGrabDropThreshold) {
        robotInstance.doAttemptGrip_ = false;
      } else if (!robotInstance.doAttemptGrip_ && grabDropAction > stickyGrabDropThreshold) {
        robotInstance.doAttemptGrip_ = true;
      }
    } else {
      robotInstance.doAttemptDrop_ = (grabDropAction < -stickyGrabDropThreshold);
    }

    const float clampedBaseYawAction = Mn::Math::clamp(actions_[actionIndex++], -maxAbsYawAngle, maxAbsYawAngle);
    yaws[b] = prevYaws[b] + clampedBaseYawAction; // todo: wrap angle to 360 degrees
    float clampedBaseMovementAction = Mn::Math::clamp(actions_[actionIndex++], minBaseMovement, maxBaseMovement);
    positions[b] =
        prevPositions[b] + 
        Mn::Vector2(Mn::Math::cos(Mn::Math::Rad(yaws[b])), -Mn::Math::sin(Mn::Math::Rad(yaws[b]))) 
        * clampedBaseMovementAction;

    int baseJointIndex = b * robot_.numPosVars;
    for (int j = 0; j < robot_.numPosVars; j++) {
      auto& pos = jointPositions[baseJointIndex + j];
      const auto& prevPos = prevJointPositions[baseJointIndex + j];
      const float clampedJointMovementAction = Mn::Math::clamp(actions_[actionIndex++], 
        -maxAbsJointAngle, maxAbsJointAngle);
      pos = prevPos + clampedJointMovementAction;
      pos = Mn::Math::clamp(pos, robot_.jointPositionLimits.first[j],
                            robot_.jointPositionLimits.second[j]);

    }
  }
  BATCHED_SIM_ASSERT(actionIndex == actions_.size());

  robots_.updateLinkTransforms(currRolloutSubstep_, /*updateforPhysics*/ true, /*updateForRender*/ false);

  updateCollision();

  updateGripping();

  postCollisionUpdate();

  // robots_.applyActionPenalties(actions_);
}

void BatchedSimulator::reverseRobotMovementActions() {

  if (prevRolloutSubstep_ == -1) {
    return;
  }

  BATCHED_SIM_ASSERT(!config_.doAsyncPhysicsStep);

  int numEnvs = bpsWrapper_->envs_.size();

  for (int b = 0; b < numEnvs; b++) {

    reverseActionsForEnvironment(b);

    auto& robotInstance = robots_.robotInstances_[b];
    robotInstance.doAttemptDrop_ = false;
    robotInstance.doAttemptGrip_ = false;
  }

  currRolloutSubstep_--;
  prevRolloutSubstep_--;

  robots_.updateLinkTransforms(currRolloutSubstep_, /*updateforPhysics*/ false, /*updateForRender*/ true);
  // updateGripping();
  // calcRewards(); // this doesn't work
  updateRenderInstances(true);
}


#if 0
void BatchedSimulator::stepPhysicsWithReferenceActions() {
  int numEnvs = bpsWrapper_->envs_.size();

  // todo: animate over time

  static float animTime = 0.f;
  animTime += 0.1f;

  Mn::Vector3 animBaseTranslationOffset{animTime * 0.2f, 0.f,
                                        0.f};      // translate at 0.2 m/s
  float animBaseRotationOffset = animTime * 90.f;  // rotate at 90 deg/s
  // move from -0.2 to 0.2 with period = 1s
  float animJointPosOffset = Mn::Math::sin(Mn::Deg(animTime * 360.f)) * 0.2f;

  auto& robots = robots_;

  int actionIndex = 0;

  // stepping code
  for (int b = 0; b < numEnvs; b++) {

    yaws[b] += actions_[actionIndex++]; // todo: wrap angle to 360 degrees
    // note clamp move-forward action to [0,-]
    positions[b] += Mn::Vector2(Mn::Math::cos(Mn::Math::Rad(yaws[b])),
      Mn::Math::sin(Mn::Math::Rad(yaws[b])))
      * Mn::Math::max(actions_[actionIndex++], 0.f);

    // perf todo: simplify this
    robots.rootTransforms_[b] =
        Mn::Matrix4::translation(Mn::Vector3(positions[b].x(), 0.f, positions[b].y())) *
        Mn::Matrix4::rotation(Mn::Deg(yaws[b]),
                              {0.f, 1.f, 0.f}) *
        Mn::Matrix4::rotation(Mn::Deg(-90.f), {1.f, 0.f, 0.f});

    int baseJointIndex = b * robot_.numPosVars;
    float jointPos = b * 0.05 + animJointPosOffset;
    for (int j = 0; j < robot_.numPosVars; j++) {
      jointPositions[baseJointIndex + j] += actions_[actionIndex++];
      // todo: clamp to joint limits
    }
  }

  robots_.updateLinkTransforms();
}
#endif

bool BatchedSimulator::isPhysicsThreadActive() const {
  return config_.doAsyncPhysicsStep &&
    (!isAsyncStepPhysicsFinished_ || signalStepPhysics_);
}

void BatchedSimulator::startRender() {
  BATCHED_SIM_ASSERT(!isPhysicsThreadActive());
  ProfilingScope scope("start render");
  BATCHED_SIM_ASSERT(isOkToRender_);
  bpsWrapper_->renderer_->render(bpsWrapper_->envs_.data());
  isOkToRender_ = false;
  isRenderStarted_ = true;
}

void BatchedSimulator::waitForRender() {
  ProfilingScope scope("wait for GPU render");
  BATCHED_SIM_ASSERT(isRenderStarted_);
  bpsWrapper_->renderer_->waitForFrame();
  isRenderStarted_ = false;
  isOkToRender_ = true;
}

bps3D::Renderer& BatchedSimulator::getBpsRenderer() {
  BATCHED_SIM_ASSERT(bpsWrapper_->renderer_.get());
  return *bpsWrapper_->renderer_.get();
}

RewardCalculationContext::RewardCalculationContext(const Robot* robot,
                                                   int numEnvs,
                                                   RolloutRecord* rollouts)
    : robot_(robot), numEnvs_(numEnvs), rollouts_(rollouts) {
  esp::sim::SimulatorConfiguration simConfig{};
  simConfig.activeSceneName = "NONE";
  simConfig.enablePhysics = true;
  simConfig.createRenderer = false;
  simConfig.loadRenderAssets =
      false;  // todo: avoid creating render assets for stage
  // simConfig.physicsConfigFile = physicsConfigFile;

  legacySim_ = esp::sim::Simulator::create_unique(simConfig);

  // todo: avoid code duplication with Robot
  const std::string filepath = "../data/URDF/opt_fetch/robots/fetch.urdf";
  // todo: delete object on destruction
  auto managedObj = legacySim_->getArticulatedObjectManager()
                        ->addBulletArticulatedObjectFromURDF(filepath);
  artObj_ = static_cast<esp::physics::BulletArticulatedObject*>(
      managedObj->hackGetBulletObjectReference().get());
}

void RewardCalculationContext::calcRewards(int currRolloutSubstep,
                                           int bStart,
                                           int bEnd) {
  const Robot* robot = robot_;
  esp::physics::BulletArticulatedObject* artObj = artObj_;
  auto* mb = artObj->btMultiBody_.get();
  RolloutRecord& rollouts = *rollouts_;

  int numPosVars = robot->numPosVars;
  int numEnvs = numEnvs_;
  int numLinks = robot->artObj->getNumLinks();

  const float* yaws = &rollouts.yaws_[currRolloutSubstep * numEnvs];
  const Mn::Vector2* positions =
      &rollouts.positions_[currRolloutSubstep * numEnvs];
  const float* jointPositions =
      &rollouts.jointPositions_[currRolloutSubstep * numEnvs * numPosVars];
  const Mn::Matrix4* rootTransforms =
      &rollouts_->rootTransforms_[currRolloutSubstep * numEnvs];

  float* rewards = &rollouts.rewards_[currRolloutSubstep * numEnvs];

  int posCount = bStart * numPosVars;

  for (int b = bStart; b < bEnd; b++) {
    // this should already be computed
    // rootTransforms[b] =
    //     Mn::Matrix4::translation(Mn::Vector3(positions[b].x(), 0.f,
    //     positions[b].y())) * Mn::Matrix4::rotation(Mn::Rad(yaws[b]),
    //                           {0.f, 1.f, 0.f}) *
    //     Mn::Matrix4::rotation(Mn::Deg(-90.f), {1.f, 0.f, 0.f});

    btTransform tr{rootTransforms[b]};
    mb->setBaseWorldTransform(tr);

    for (int i = 0; i < numLinks; ++i) {
      auto& link = mb->getLink(i);
      // optimization todo: find correct subset of links
      if (link.m_posVarCount > 0) {
        mb->setJointPosMultiDof(i,
                                const_cast<float*>(&jointPositions[posCount]));
        posCount += link.m_posVarCount;
      }
    }

    //// copied from BulletArticulatedObject::updateKinematicState ////
    mb->forwardKinematics(scratch_q_, scratch_m_);
    mb->updateCollisionObjectWorldTransforms(scratch_q_, scratch_m_);
    artObj->updateAabbs();

    bool isContact = artObj->contactTest();
    // if (isContact) {
    //   ESP_WARNING() << "collision, step " << currRolloutSubstep << ", env " <<
    //   b;
    // }

    rewards[b] = isContact ? -1.f : 1.f;
  }
}

const std::vector<float>& BatchedSimulator::getRewards() {
  return robots_.hackRewards_;
}

const std::vector<bool>& BatchedSimulator::getDones() {
  return hackDones_;
}

void BatchedSimulator::calcRewards() {

  BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);

  // update rewards on main thread
  const int numEnvs = config_.numEnvs;
  for (int b = 0; b < numEnvs; b++) {
    robots_.hackRewards_[b] = robots_.collisionResults_[b] ? -1.f : 0.f;
  }

  // int numEnvs = bpsWrapper_->envs_.size();
  // rewardContext_.calcRewards(currRolloutSubstep_, 0, numEnvs);
}

RolloutRecord::RolloutRecord(int numRolloutSubsteps,
                             int numEnvs,
                             int numPosVars,
                             int numNodes)
    : numRolloutSubsteps_(numRolloutSubsteps) {
  Magnum::Matrix4 nanMat(NAN);
  Mn::Vector2 nanVec(NAN);

  jointPositions_.resize(numRolloutSubsteps * numEnvs * numPosVars, NAN);
  yaws_.resize(numRolloutSubsteps * numEnvs, NAN);
  positions_.resize(numRolloutSubsteps * numEnvs, nanVec);
  rootTransforms_.resize(numRolloutSubsteps * numEnvs, nanMat);
  nodeTransforms_.resize(numRolloutSubsteps * numEnvs * numNodes, nanMat);

  rewards_.resize(numRolloutSubsteps * numEnvs, NAN);
}

void BatchedSimulator::deleteDebugInstances() {
#ifdef ENABLE_DEBUG_INSTANCES
  const int numEnvs = config_.numEnvs;
  for (int b = 0; b < numEnvs; b++) {

    auto& env = bpsWrapper_->envs_[b];
    for (int instanceId : debugInstancesByEnv_[b]) {

      env.deleteInstance(instanceId);
    }

    debugInstancesByEnv_[b].clear();
  }
#endif
}

int BatchedSimulator::addDebugInstance(const std::string& name, int envIndex, 
  const Magnum::Matrix4& transform, bool persistent) {
#ifdef ENABLE_DEBUG_INSTANCES
  glm::mat4x3 glMat = toGlmMat4x3(transform);
  const auto& blueprint = sceneMapping_.findInstanceBlueprint(name);
  BATCHED_SIM_ASSERT(envIndex < config_.numEnvs);
  auto& env = getBpsEnvironment(envIndex);
  int instanceId = env.addInstance(blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
  if (!persistent) {
    debugInstancesByEnv_[envIndex].push_back(instanceId);
  }
  return instanceId;
#else
  return -1;
#endif
}

std::string BatchedSimulator::getRecentStatsAndReset() const {
  if (recentStats_.numSteps_ == 0) {
    return "no recent steps";
  } else if (recentStats_.numEpisodes_ == 0) {
    return "no recent episodes";
  }
  float collisionFraction = recentStats_.numSteps_ > 0 ? (float)recentStats_.numStepsInCollision_ / recentStats_.numSteps_ : -1.f;
  float gripAttemptsPerEpisode = (float)recentStats_.numGripAttempts_ / recentStats_.numEpisodes_;
  float gripsPerEpisode = (float)recentStats_.numGrips_ / recentStats_.numEpisodes_;
  float dropsPerEpisode = (float)recentStats_.numDrops_ / recentStats_.numEpisodes_;
  float failedDropsPerEpisode = (float)recentStats_.numFailedDrops_ / recentStats_.numEpisodes_;

  recentStats_ = StatRecord();

  std::stringstream ss;
  ss << "collisionFraction " << std::setprecision(5) << collisionFraction << ", "
    << "gripAttemptsPerEpisode " << std::setprecision(5) << gripAttemptsPerEpisode << ", "
    << "gripsPerEpisode " << std::setprecision(5) << gripsPerEpisode << ", "
    << "dropsPerEpisode " << std::setprecision(5) << dropsPerEpisode << ", "
    << "failedDropsPerEpisode " << std::setprecision(5) << failedDropsPerEpisode;

  return ss.str();
}

void BatchedSimulator::signalStepPhysics() {
  //ProfilingScope scope("BSim signalStepPhysics");
  
  std::lock_guard<std::mutex> lck(physicsMutex_);
  signalStepPhysics_ = true;
  isAsyncStepPhysicsFinished_ = false;
  areRenderInstancesUpdated_ = false;
  physicsCondVar_.notify_one(); // notify_all?
}

void BatchedSimulator::signalKillPhysicsThread() {
  std::lock_guard<std::mutex> lck(physicsMutex_);
  signalKillPhysicsThread_ = true;
  physicsCondVar_.notify_one(); // notify_all?
}


const std::vector<PythonEnvironmentState>& BatchedSimulator::getEnvironmentStates() const {
  ESP_CHECK(!isPhysicsThreadActive(), "Don't call getEnvironmentStates during async physics step");
  ESP_CHECK(isRenderStarted_, "For best runtime perf, call getEnvironmentStates *after* startRender");  
  return pythonEnvStates_;  
}

void BatchedSimulator::waitAsyncStepPhysics() {
  //ProfilingScope scope("BSim waitAsyncStepPhysics");

  {
    std::unique_lock<std::mutex> lck(physicsMutex_);
    physicsCondVar_.wait(lck, [&]{ return isAsyncStepPhysicsFinished_; });
  }

  // perf todo: do this reset on the physics thread
  if (currRolloutSubstep_ == maxRolloutSubsteps_ - 1) {
    reset();
  } else {
    updatePythonEnvironmentState();
  }

  // sloppy: don't calc rewards if we just did a reset
  if (currRolloutSubstep_ != 0) {
    calcRewards();
  }
}

void BatchedSimulator::physicsThreadFunc(int startEnvIndex, int numEnvs) {
  ProfilingScope scope("physics background thread");

  while (true) {
    {
      ProfilingScope scope("wait for main thread");
      std::unique_lock<std::mutex> lck(physicsMutex_);
      physicsCondVar_.wait(lck, [&]{ return signalStepPhysics_ || signalKillPhysicsThread_; });
    }

    if (signalKillPhysicsThread_) {
      break;
    }

    BATCHED_SIM_ASSERT(startEnvIndex == 0 && numEnvs == config_.numEnvs);
    // BATCHED_SIM_ASSERT(startEnvIndex >= 0 && startEnvIndex);
    // BATCHED_SIM_ASSERT(numEnvs > 0);
    // BATCHED_SIM_ASSERT(startEnvIndex + numEnvs <= config_.numEnvs);

    stepPhysics();

    updateRenderInstances(/*forceUpdate*/false);

    {
      // ProfilingScope scope("physicsThreadFunc notify after step");
      std::lock_guard<std::mutex> lck(physicsMutex_);
      isAsyncStepPhysicsFinished_ = true;
      signalStepPhysics_ = false;
      physicsCondVar_.notify_one(); // notify_all?
    }
  }
}


void BatchedSimulator::debugRenderColumnGrids(int b, int minProgress, int maxProgress) const {
#ifdef ENABLE_DEBUG_INSTANCES

  auto& env = safeVectorGet(bpsWrapper_->envs_, b);
  const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
  const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
  const auto& source = stageFixedObject.columnGridSet_.getColumnGrid(0);

  const auto& blueprint = sceneMapping_.findInstanceBlueprint("cube_gray_shaded");

  // note off by one
  const float maxOccludedY = 3.f;
  for (int layerIdx = 0; layerIdx < source.layers.size() - 1; layerIdx++) {
    // temp
    for (int cellZ = 0; cellZ < source.dimZ; cellZ++) {
      for (int cellX = 0; cellX < source.dimX; cellX++) {

        int progress = cellX + cellZ;
        if (progress < minProgress || 
          (maxProgress != -1 && progress >= maxProgress)) {
          continue;
        }

        auto col0 = source.debugGetColumn(cellX, cellZ, layerIdx);
        auto col1 = source.debugGetColumn(cellX, cellZ, layerIdx + 1);
        if (col0.freeMinY == source.INVALID_Y) {
          continue;
        }
        if (col0.freeMaxY >= maxOccludedY) {
          continue;
        }
        float occludedMinY = col0.freeMaxY;
        float occludedMaxY = (col1.freeMinY == source.INVALID_Y)
          ? maxOccludedY
          : col1.freeMinY;

        Mn::Range3D aabb(
          Mn::Vector3(
            source.minX + cellX * source.gridSpacing,
            occludedMinY,
            source.minZ + cellZ * source.gridSpacing),
          Mn::Vector3(
            source.minX + (cellX + 1) * source.gridSpacing,
            occludedMaxY,
            source.minZ + (cellZ + 1) * source.gridSpacing));

        Mn::Matrix4 localToBox = Mn::Matrix4::translation(aabb.center())
          * Mn::Matrix4::scaling(aabb.size() * 0.5);

        glm::mat4x3 glMat = toGlmMat4x3(localToBox);
        int instanceId = env.addInstance(blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
      }
    }
  }
#endif
}

void BatchedSimulator::reloadSerializeCollection() {

  int numEnvs = bpsWrapper_->envs_.size();

  serializeCollection_ = serialize::Collection::loadFromFile(serializeCollectionFilepath_);

  robot_.updateFromSerializeCollection(serializeCollection_);

  robots_.collisionSphereWorldOrigins_.resize(robot_.numCollisionSpheres_ * numEnvs);
  robots_.collisionSphereQueryCaches_.resize(robot_.numCollisionSpheres_ * numEnvs, 0);

  updateFromSerializeCollection(episodeSet_, serializeCollection_);

}


}  // namespace batched_sim
}  // namespace esp
