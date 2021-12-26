// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchedSimulator.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/BatchedSimAssert.h"

#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/json.h"

// #include <bps3D.hpp>

#include <iostream>

#include <mutex>

#ifndef NDEBUG
// #define ENABLE_DEBUG_INSTANCES
#endif

namespace esp {
namespace batched_sim {

namespace {

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

  // return Mn::Matrix4{
  //   toMagnumVector4(basis.getColumn(0)),
  //   toMagnumVector4(basis.getColumn(1)),
  //   toMagnumVector4(basis.getColumn(2)),
  //   toMagnumVector4(btTrans.getOrigin(), 1.f)
  // };

  // auto tmp = Mn::Matrix4{
  //   toMagnumVector4(basis.getRow(0)),
  //   toMagnumVector4(basis.getRow(1)),
  //   toMagnumVector4(basis.getRow(2)),
  //   toMagnumVector4(btTrans.getOrigin(), 1.f)
  // };

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

}  // namespace

void BatchedSimulator::randomizeRobotsForCurrentStep() {
  BATCHED_SIM_ASSERT(currRolloutStep_ >= 0);
  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  float* yaws = &safeVectorGet(rollouts_.yaws_, currRolloutStep_ * numEnvs);
  Mn::Vector2* positions = &safeVectorGet(rollouts_.positions_,currRolloutStep_ * numEnvs);
  float* jointPositions =
      &safeVectorGet(rollouts_.jointPositions_,currRolloutStep_ * numEnvs * numPosVars);

  auto random = core::Random(/*seed*/ 1);

  for (int b = 0; b < numEnvs; b++) {
    yaws[b] = random.uniform_float(-float(Mn::Rad(Mn::Deg(90.f))), 0.f);
    positions[b] =
        Mn::Vector2(1.61, 0.98) + Mn::Vector2(random.uniform_float(-0.2, 0.2f),
                                              random.uniform_float(-0.2, 0.2f));
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
    jointPositions[b * robot_.numPosVars + 9] = float(Mn::Rad(Mn::Deg(-90.f)));
  }
}

RobotInstanceSet::RobotInstanceSet(Robot* robot,
                                   int numEnvs,
                                   std::vector<bps3D::Environment>* envs,
                                   RolloutRecord* rollouts)
    : numEnvs_(numEnvs), envs_(envs), robot_(robot), rollouts_(rollouts) {
  int numLinks = robot->artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base
  int batchNumPosVars = robot_->numPosVars * numEnvs;
  int batchNumNodes = numNodes * numEnvs;

  nodeInstanceIds_.resize(batchNumNodes, -1);
  nodeNewTransforms_.resize(batchNumNodes);
  collisionSphereWorldOrigins_.resize(robot->numCollisionSpheres_ * numEnvs);
  collisionSphereQueryCaches_.resize(robot->numCollisionSpheres_ * numEnvs, 0);

  const auto* mb = robot_->artObj->btMultiBody_.get();

  int baseInstanceIndex = 0;
  for (auto& env : *envs_) {
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

        instanceId = env.addInstance(instanceBlueprint.meshIdx_, 
          instanceBlueprint.mtrlIdx_, identityGlMat_);
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

  collisionResults_.resize(robot_->numCollisionSpheres_ * numEnvs, false);
}

void RobotInstanceSet::applyActionPenalties(const std::vector<float>& actions) {
  int numEnvs = numEnvs_;

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
  const int numEnvs = config_.numEnvs;
  const int numPosVars = robot_.numPosVars;

  const float* prevYaws = &rollouts_.yaws_[prevRolloutStep_ * numEnvs];
  float* yaws = &rollouts_.yaws_[currRolloutStep_ * numEnvs];
  const Mn::Vector2* prevPositions =
      &rollouts_.positions_[prevRolloutStep_ * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutStep_ * numEnvs];
  Mn::Matrix4* rootTransforms =
      &rollouts_.rootTransforms_[currRolloutStep_ * numEnvs];
  const float* prevJointPositions =
      &rollouts_.jointPositions_[prevRolloutStep_ * numEnvs * numPosVars];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutStep_ * numEnvs * numPosVars];

  yaws[b] = prevYaws[b];
  positions[b] = prevPositions[b];
  int baseJointIndex = b * robot_.numPosVars;
  for (int j = 0; j < robot_.numPosVars; j++) {
    auto& pos = jointPositions[baseJointIndex + j];
    const auto& prevPos = prevJointPositions[baseJointIndex + j];
    pos = prevPos;
  }
}

void RobotInstanceSet::updateLinkTransforms(int currRolloutStep) {
  // esp::gfx::replay::Keyframe debugKeyframe;
  int numLinks = robot_->artObj->getNumLinks();
  int numNodes = numLinks + 1;
  int numEnvs = numEnvs_;
  int numPosVars = robot_->numPosVars;

  areCollisionResultsValid_ = false;

  auto* mb = robot_->artObj->btMultiBody_.get();
  int posCount = 0;

  const float* yaws = &rollouts_->yaws_[currRolloutStep * numEnvs];
  const Mn::Vector2* positions =
      &rollouts_->positions_[currRolloutStep * numEnvs];
  const float* jointPositions =
      &rollouts_->jointPositions_[currRolloutStep * numEnvs * numPosVars];
  Mn::Matrix4* rootTransforms =
      &rollouts_->rootTransforms_[currRolloutStep * numEnvs];

  for (int b = 0; b < numEnvs_; b++) {
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
    int sphereIndex = baseSphereIndex;

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

        // todo: avoid btTransform copy for case of i != -1
        const auto btTrans = i == -1 ? mb->getBaseWorldTransform()
                                     : mb->getLink(i).m_cachedWorldTransform;
        Mn::Matrix4 mat = toMagnumMatrix4(btTrans);

        auto tmp = robot_->nodeTransformFixups[nodeIndex];
        // auto vec = tmp[3];
        // const float scale = (float)b / (numEnvs_ - 1);
        // tmp[3] = Mn::Vector4(vec.xyz() * scale, 1.f);
        mat = mat * tmp;
        // perf todo: loop through collision spheres (and look up link id), instead of this sparse way here
        // compute collision sphere transforms
        for (const auto& localOrigin : robot_->collisionSphereLocalOriginsByNode_[nodeIndex]) {
          collisionSphereWorldOrigins_[sphereIndex++] = 
            mat.transformPoint(localOrigin);
        }

        BATCHED_SIM_ASSERT(instanceIndex < nodeNewTransforms_.size());
        nodeNewTransforms_[instanceIndex] = toGlmMat4x3(mat);

        // hack calc reward
        {
          constexpr int rewardLink = 22;
          constexpr Mn::Vector3 targetPos(2.5f, 0.25f, 1.f);  // 1.61, 0.98

          if (i == rewardLink) {
            const Mn::Vector3& pos = mat.translation();
            float dist = (pos - targetPos).length();
            hackRewards_[b] = -dist;

            // addition penalty for too-large actions
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

      BATCHED_SIM_ASSERT(sphereIndex == baseSphereIndex + robot_->numCollisionSpheres_);
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

void BatchedSimulator::updateCollision() {

  const int numEnvs = config_.numEnvs;

  BATCHED_SIM_ASSERT(!robots_.areCollisionResultsValid_);
  BATCHED_SIM_ASSERT(robots_.collisionResults_.size() == robot_.numCollisionSpheres_ * numEnvs);

  robots_.areCollisionResultsValid_ = true;

  for (int b = 0; b < numEnvs; b++) {

    const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
    const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
    const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
    const auto& columnGrid = stageFixedObject.columnGrid_;
    const int baseSphereIndex = b * robot_.numCollisionSpheres_;
    bool hit = false;

    // perf todo: if there was a hit last frame, cache that sphere and test it first here
    for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
      const int sphereIndex = baseSphereIndex + s;
      auto& queryCache = robots_.collisionSphereQueryCaches_[sphereIndex];
      const auto& spherePos = robots_.collisionSphereWorldOrigins_[sphereIndex];

      if (columnGrid.contactTest(spherePos, &queryCache)) {
        hit = true;
        break;
      }
    }

    robots_.collisionResults_[b] = hit;
  }

#define UPDATE_COLLISION_USE_BATCH_TEST
#ifdef UPDATE_COLLISION_USE_BATCH_TEST

// bool batchSphereOrientedBoxContactTest(const glm::mat4x3* orientedBoxTransforms, const Magnum::Vector3* positions,
//  float sphereRadiusSq, const Magnum::Range3D* boxRanges);

  constexpr int contactTestBatchSize = 64;
  std::array<const glm::mat4x3*, contactTestBatchSize> orientedBoxTransforms;
  std::array<const Magnum::Vector3*, contactTestBatchSize> spherePositions;
  std::array<const Magnum::Range3D*, contactTestBatchSize> boxRanges;
  int numCollectedTests = 0;
#endif

  for (int b = 0; b < numEnvs; b++) {

    if (robots_.collisionResults_[b]) {
      continue;
    }

    const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
    const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
    const int baseSphereIndex = b * robot_.numCollisionSpheres_;
    bool hit = false;
    auto& env = bpsWrapper_->envs_[b];

    // perf todo: if there was a hit last frame, cache that sphere and test it first here
    for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
      const int sphereIndex = baseSphereIndex + s;
      auto& queryCache = robots_.collisionSphereQueryCaches_[sphereIndex];
      const auto& spherePos = robots_.collisionSphereWorldOrigins_[sphereIndex];

      // todo: get pre-culled list of objects from a spatial data structure
      for (int i = 0; i < episode.numFreeObjectSpawns_; i++) {

        int globalFreeObjectIndex = b * episodeSet_.maxFreeObjects_ + i;

        // perf todo: consider storing 
        const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
          episode.firstFreeObjectSpawnIndex_ + i);
        const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

        auto instanceId = safeVectorGet(episodeInstanceSet_.freeObjectInstanceIds_, globalFreeObjectIndex);
        const glm::mat4x3& glMat = env.getInstanceTransform(instanceId);

        constexpr float sphereRadiusSq = 0.1f * 0.1f; // todo: get from Robot

#ifdef ENABLE_DEBUG_INSTANCES
        if (s == 0) {
          const auto& rotation = safeVectorGet(freeObject.startRotations_, freeObjectSpawn.startRotationIndex_);
          Mn::Matrix4 mat = Mn::Matrix4::from(
              rotation, freeObjectSpawn.startPos_);

          Mn::Matrix4 localToBox = Mn::Matrix4::translation(freeObject.aabb_.center())
            * Mn::Matrix4::scaling(freeObject.aabb_.size() * 0.5);
          auto adjustedMat = mat * localToBox;
          addDebugInstance("cube_green", b, adjustedMat);
        }
#endif

#ifdef UPDATE_COLLISION_USE_BATCH_TEST
        orientedBoxTransforms[numCollectedTests] = &glMat;
        spherePositions[numCollectedTests] = &spherePos;
        boxRanges[numCollectedTests] = &freeObject.aabb_;
        numCollectedTests++;
        bool isLastTestForRobot = (s == (robot_.numCollisionSpheres_ - 1)
          && i == episode.numFreeObjectSpawns_ - 1);
        if (numCollectedTests == contactTestBatchSize || isLastTestForRobot) {

          hit = numCollectedTests != contactTestBatchSize
            ? batchSphereOrientedBoxContactTest<contactTestBatchSize, false>(&orientedBoxTransforms[0],
            &spherePositions[0], sphereRadiusSq, &boxRanges[0], numCollectedTests)
            : batchSphereOrientedBoxContactTest<contactTestBatchSize, true>(&orientedBoxTransforms[0],
            &spherePositions[0], sphereRadiusSq, &boxRanges[0], numCollectedTests);
          numCollectedTests = 0;
          if (hit) {
            break;
          }
        }
#else
        auto posLocal = inverseTransformPoint(glMat, spherePos);

        if (sphereBoxContactTest(posLocal, sphereRadiusSq, freeObject.aabb_)) {
          hit = true;
          break;
        }
#endif        
      }

      if (hit) {
        break;
      }
    }

    robots_.collisionResults_[b] = hit;
  }

#ifdef ENABLE_DEBUG_INSTANCES
  for (int b = 0; b < numEnvs; b++) {

    const auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
    const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
    const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
    const auto& columnGrid = stageFixedObject.columnGrid_;
    const int baseSphereIndex = b * robot_.numCollisionSpheres_;

    for (int s = 0; s < robot_.numCollisionSpheres_; s++) {
      const int sphereIndex = baseSphereIndex + s;
      auto spherePos = robots_.collisionSphereWorldOrigins_[sphereIndex];

      // temp hack raise in y to help visualize
      constexpr float hackOffsetY = 0.0f;
      spherePos.y() += hackOffsetY;

      constexpr float sphereRadius = 0.1f; // todo: get from Robot
      Mn::Matrix4 mat = Mn::Matrix4::translation(spherePos)
        * Mn::Matrix4::scaling(Mn::Vector3(sphereRadius, sphereRadius, sphereRadius));

      // redo the query here since we didn't save collision results per sphere
      esp::batched_sim::ColumnGridSource::QueryCacheValue queryCache = 0;
      bool sphereHit = columnGrid.contactTest(spherePos, &queryCache);

      //addDebugInstance(sphereHit ? "sphere_orange" : "sphere_green", b, mat);
      addDebugInstance((s % 2 == 0) ? "sphere_orange" : "sphere_green", b, mat);
    }
  }
#endif
}

void BatchedSimulator::postCollisionUpdate(bool useCollisionResults) {

  const int numEnvs = config_.numEnvs;
  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  if (useCollisionResults) {
    BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);
  }

  for (int b = 0; b < numEnvs; b++) {

    if (useCollisionResults && robots_.collisionResults_[b]) {
      reverseActionsForEnvironment(b);

    } else {
      // no collision, so let's update the robot instance instances

      int baseInstanceIndex = b * numNodes;
      auto& env = bpsWrapper_->envs_[b];

      for (int i = -1; i < numLinks; i++) {
        const auto nodeIndex = i + 1;  // 0 is base
        const auto instanceIndex = baseInstanceIndex + nodeIndex;

        int instanceId = robots_.nodeInstanceIds_[instanceIndex];
        if (instanceId == -1) {
          continue;
        }

        const auto& glMat = robots_.nodeNewTransforms_[instanceIndex];
        env.updateInstanceTransform(instanceId, glMat);
      }
    }
  }
}

Robot::Robot(const std::string& filepath, esp::sim::Simulator* sim, BpsSceneMapping* sceneMapping) {
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

  collisionSphereLocalOriginsByNode_.resize(numNodes);
  int numCollisionSpheres = 0;

  std::vector<std::string> armLinks = {
    "shoulder_lift_link",
    "shoulder_pan_link",
    "upperarm_roll_link",
    "elbow_flex_link",
    "forearm_roll_link",
    "wrist_flex_link",
    "wrist_roll_link",
    "gripper_link"
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
    BATCHED_SIM_ASSERT(i == -1 || i == linkIds[i]);
    const auto& link = artObj->getLink(i);  // -1 gets base link
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

  }

  numInstances_ = numInstances;
  numCollisionSpheres_ = numCollisionSpheres;

  numPosVars = artObj->getJointPositions().size();
  BATCHED_SIM_ASSERT(numPosVars > 0);
}

BpsWrapper::BpsWrapper(int gpuId, int numEnvs, const CameraSensorConfig& sensor0) {
  glm::u32vec2 out_dim(
      sensor0.width,
      sensor0.height);  // see also rollout_test.py, python/rl/agent.py
  BATCHED_SIM_ASSERT(gpuId != -1);

  renderer_ = std::make_unique<bps3D::Renderer>(bps3D::RenderConfig{
      gpuId, 1, uint32_t(numEnvs), out_dim.x, out_dim.y, false,
      bps3D::RenderMode::Depth | bps3D::RenderMode::UnlitRGB});

  loader_ = std::make_unique<bps3D::AssetLoader>(renderer_->makeLoader());
  const std::string filepath =
      "../data/bps_data/replicacad_composite/replicacad_composite.bps";
  scene_ = loader_->loadScene(filepath);

  const Mn::Vector3 camPos{-1.61004, 1.5, 3.5455};
  const Mn::Quaternion camRot{{0, -0.529178, 0}, 0.848511};
  glm::mat4 base(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < numEnvs; b++) {
    glm::mat4 view = base;
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

  bpsWrapper_ = std::make_unique<BpsWrapper>(config_.gpuId, config_.numEnvs, config_.sensor0);

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

  const std::string filepath = "../data/URDF/opt_fetch/robots/fetch.urdf";

  robot_ = Robot(filepath, legacySim_.get(), &sceneMapping_);
  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  const int numEnvs = config_.numEnvs;
  robots_ = RobotInstanceSet(&robot_, numEnvs, &bpsWrapper_->envs_, &rollouts_);

  // see also python/rl/agent.py
  const int numJointDegrees = robot_.numPosVars;
  const int numBaseDegrees = 2;  // rotate and move-forward/back

  int batchNumActions = (numJointDegrees + numBaseDegrees) * numEnvs;
  actions_.resize(batchNumActions, 0.f);

  maxRolloutSteps_ = 50;
  rollouts_ =
      RolloutRecord(maxRolloutSteps_, numEnvs, robot_.numPosVars, numNodes);

  rewardContext_ = RewardCalculationContext(&robot_, numEnvs, &rollouts_);
  robots_.hackRewards_.resize(numEnvs, 0.f);
  hackDones_.resize(numEnvs, false);

  currRolloutStep_ =
      -1;  // trigger auto-reset on first call to autoResetOrStepPhysics
  prevRolloutStep_ = -1;
  isOkToRender_ = false;
  isOkToStep_ = false;
  isRenderStarted_ = false;
}

void BatchedSimulator::setCamera(const Mn::Vector3& camPos, const Mn::Quaternion& camRot) {

  const int numEnvs = config_.numEnvs;

  glm::mat4 world_to_camera(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < numEnvs; b++) {
    auto& env = bpsWrapper_->envs_[b];
    env.setCameraView(world_to_camera);
  }
}

bps3D::Environment& BatchedSimulator::getBpsEnvironment(int envIndex) {
  BATCHED_SIM_ASSERT(envIndex < config_.numEnvs);
  return bpsWrapper_->envs_[envIndex];
}

EpisodeInstance BatchedSimulator::instantiateEpisode(int b, int episodeIndex) {

  auto& env = getBpsEnvironment(b);

  EpisodeInstance epInstance;
  epInstance.episodeIndex_ = episodeIndex;

  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeIndex);
  const auto& stageBlueprint = 
    safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex).instanceBlueprint_;
  epInstance.stageFixedObjectInstanceId_ = env.addInstance(
    stageBlueprint.meshIdx_, stageBlueprint.mtrlIdx_, identityGlMat_);

  for (int i = 0; i < episode.numFreeObjectSpawns_; i++) {

    int globalFreeObjectIndex = b * episodeSet_.maxFreeObjects_ + i;

    const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
      episode.firstFreeObjectSpawnIndex_ + i);

    const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);
    const auto& blueprint = freeObject.instanceBlueprint_;

    const auto& rotation = safeVectorGet(freeObject.startRotations_, freeObjectSpawn.startRotationIndex_);
    Mn::Matrix4 mat = Mn::Matrix4::from(
        rotation, freeObjectSpawn.startPos_);
    glm::mat4x3 glMat = toGlmMat4x3(mat); 

    safeVectorGet(episodeInstanceSet_.freeObjectInstanceIds_, globalFreeObjectIndex) = env.addInstance(
      blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
  }

  return epInstance;
}

void BatchedSimulator::initEpisodeSet() {

  const int numEnvs = config_.numEnvs;

  // generate exactly as many episodes as envs (this is not required)
  episodeSet_ = generateBenchmarkEpisodeSet(numEnvs, sceneMapping_);

  constexpr int invalidInstanceId = -1;
  episodeInstanceSet_.freeObjectInstanceIds_.resize(numEnvs * episodeSet_.maxFreeObjects_, invalidInstanceId);

  episodeInstanceSet_.episodeInstanceByEnv_.reserve(numEnvs);
  for (int b = 0; b < numEnvs; b++) {
    const auto episodeIndex = b * episodeSet_.episodes_.size() / numEnvs; // distribute episodes across instances
    EpisodeInstance epInstance = instantiateEpisode(b, episodeIndex);
    episodeInstanceSet_.episodeInstanceByEnv_.emplace_back(std::move(epInstance));
  }
}

void BatchedSimulator::setActions(std::vector<float>&& actions) {
  ESP_CHECK(actions.size() == actions_.size(),
            "BatchedSimulator::setActions: input dimension should be " +
                std::to_string(actions_.size()) + ", not " +
                std::to_string(actions.size()));
  actions_ = std::move(actions);
}

void BatchedSimulator::reset() {
  BATCHED_SIM_ASSERT(!isRenderStarted_);

  currRolloutStep_ = 0;
  prevRolloutStep_ = -1;
  randomizeRobotsForCurrentStep();
  robots_.updateLinkTransforms(currRolloutStep_);
  postCollisionUpdate(/*useCollisionResults*/false);

  isOkToRender_ = true;
  isOkToStep_ = true;
}

void BatchedSimulator::autoResetOrStepPhysics() {

  deleteDebugInstances();

  if (currRolloutStep_ == -1 || currRolloutStep_ == maxRolloutSteps_ - 1) {
    // all episodes are done; set done flag and reset
    std::fill(hackDones_.begin(), hackDones_.end(), true);
    reset();
  } else {
    if (currRolloutStep_ == 0) {
      std::fill(hackDones_.begin(), hackDones_.end(), false);
    } else {
      BATCHED_SIM_ASSERT(!hackDones_[0]);
    }
    stepPhysics();
  }
}

void BatchedSimulator::stepPhysics() {
  BATCHED_SIM_ASSERT(isOkToStep_);

  prevRolloutStep_ = currRolloutStep_;
  currRolloutStep_++;

  // temp reset rollout
  if (currRolloutStep_ == maxRolloutSteps_) {
    currRolloutStep_ = 0;
  }

  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  auto& robots = robots_;

  int actionIndex = 0;

  const float* prevYaws = &rollouts_.yaws_[prevRolloutStep_ * numEnvs];
  float* yaws = &rollouts_.yaws_[currRolloutStep_ * numEnvs];
  const Mn::Vector2* prevPositions =
      &rollouts_.positions_[prevRolloutStep_ * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutStep_ * numEnvs];
  Mn::Matrix4* rootTransforms =
      &rollouts_.rootTransforms_[currRolloutStep_ * numEnvs];
  const float* prevJointPositions =
      &rollouts_.jointPositions_[prevRolloutStep_ * numEnvs * numPosVars];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutStep_ * numEnvs * numPosVars];

  // stepping code
  for (int b = 0; b < numEnvs; b++) {
    yaws[b] = prevYaws[b] +
              actions_[actionIndex++];  // todo: wrap angle to 360 degrees
    // note clamp move-forward action to [0,-]
    constexpr float baseMovementSpeed = 1.f;
    positions[b] =
        prevPositions[b] + Mn::Vector2(Mn::Math::cos(Mn::Math::Rad(yaws[b])),
                                       -Mn::Math::sin(Mn::Math::Rad(yaws[b]))) *
                               Mn::Math::max(actions_[actionIndex++], 0.f) *
                               baseMovementSpeed;

    int baseJointIndex = b * robot_.numPosVars;
    for (int j = 0; j < robot_.numPosVars; j++) {
      auto& pos = jointPositions[baseJointIndex + j];
      const auto& prevPos = prevJointPositions[baseJointIndex + j];
      pos = prevPos + actions_[actionIndex++];
      pos = Mn::Math::clamp(pos, robot_.jointPositionLimits.first[j],
                            robot_.jointPositionLimits.second[j]);

      // todo: clamp to joint limits
    }
  }
  BATCHED_SIM_ASSERT(actionIndex == actions_.size());

  robots_.updateLinkTransforms(currRolloutStep_);

  updateCollision();

  postCollisionUpdate(/*useCollisionResults*/true);

  robots_.applyActionPenalties(actions_);
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

void BatchedSimulator::startRender() {
  BATCHED_SIM_ASSERT(isOkToRender_);
  bpsWrapper_->renderer_->render(bpsWrapper_->envs_.data());
  isOkToRender_ = false;
  isRenderStarted_ = true;
}

void BatchedSimulator::waitForFrame() {
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

void RewardCalculationContext::calcRewards(int currRolloutStep,
                                           int bStart,
                                           int bEnd) {
  const Robot* robot = robot_;
  esp::physics::BulletArticulatedObject* artObj = artObj_;
  auto* mb = artObj->btMultiBody_.get();
  RolloutRecord& rollouts = *rollouts_;

  int numPosVars = robot->numPosVars;
  int numEnvs = numEnvs_;
  int numLinks = robot->artObj->getNumLinks();

  const float* yaws = &rollouts.yaws_[currRolloutStep * numEnvs];
  const Mn::Vector2* positions =
      &rollouts.positions_[currRolloutStep * numEnvs];
  const float* jointPositions =
      &rollouts.jointPositions_[currRolloutStep * numEnvs * numPosVars];
  const Mn::Matrix4* rootTransforms =
      &rollouts_->rootTransforms_[currRolloutStep * numEnvs];

  float* rewards = &rollouts.rewards_[currRolloutStep * numEnvs];

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
    //   ESP_WARNING() << "collision, step " << currRolloutStep << ", env " <<
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
  int numEnvs = bpsWrapper_->envs_.size();

  rewardContext_.calcRewards(currRolloutStep_, 0, numEnvs);
}

RolloutRecord::RolloutRecord(int numRolloutSteps,
                             int numEnvs,
                             int numPosVars,
                             int numNodes)
    : numRolloutSteps_(numRolloutSteps) {
  Magnum::Matrix4 nanMat(NAN);
  Mn::Vector2 nanVec(NAN);

  jointPositions_.resize(numRolloutSteps * numEnvs * numPosVars, NAN);
  yaws_.resize(numRolloutSteps * numEnvs, NAN);
  positions_.resize(numRolloutSteps * numEnvs, nanVec);
  rootTransforms_.resize(numRolloutSteps * numEnvs, nanMat);
  nodeTransforms_.resize(numRolloutSteps * numEnvs * numNodes, nanMat);

  rewards_.resize(numRolloutSteps * numEnvs, NAN);
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
  const Magnum::Matrix4& transform) {
#ifdef ENABLE_DEBUG_INSTANCES
  glm::mat4x3 glMat = toGlmMat4x3(transform);
  const auto& blueprint = sceneMapping_.findInstanceBlueprint(name);
  BATCHED_SIM_ASSERT(envIndex < config_.numEnvs);
  auto& env = getBpsEnvironment(envIndex);
  int instanceId = env.addInstance(blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
  debugInstancesByEnv_[envIndex].push_back(instanceId);
  return instanceId;
#else
  return -1;
#endif
}

}  // namespace batched_sim
}  // namespace esp
