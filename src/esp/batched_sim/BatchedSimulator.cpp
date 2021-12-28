// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchedSimulator.h"
#include "esp/batched_sim/GlmUtils.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/PlacementHelper.h"

#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/json.h"

// #include <bps3D.hpp>

#include <iostream>

#include <mutex>

#ifndef NDEBUG
#define ENABLE_DEBUG_INSTANCES
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

  robotInstances_.resize(numEnvs);
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

        if ((robotInstance.doAttemptGrip_ || robotInstance.grippedFreeObjectIndex_ != -1) 
          && robot_->gripperLink_ == i) {
          robotInstance.cachedGripperLinkMat_ = mat;
        }

        BATCHED_SIM_ASSERT(instanceIndex < nodeNewTransforms_.size());
        nodeNewTransforms_[instanceIndex] = toGlmMat4x3(mat);


        if (robot_->cameraAttachNode_ == nodeIndex) {
          robotInstance.cameraAttachNodeTransform_ = mat;
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

void BatchedSimulator::updateGripping() {

  const int numEnvs = config_.numEnvs;

  for (int b = 0; b < numEnvs; b++) {
  
    auto& env = bpsWrapper_->envs_[b];
    auto& robotInstance = robots_.robotInstances_[b];

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
      const auto& gripperMat = robotInstance.cachedGripperLinkMat_;

      auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);

      const auto gripperQueryWorldOrigin = gripperMat.transformPoint(robot_.gripperQueryOffset_);
      const auto gripperQueryRadius = robot_.gripperQueryRadius_;
      
#ifdef ENABLE_DEBUG_INSTANCES
      auto mat = Mn::Matrix4::translation(gripperQueryWorldOrigin)
        * Mn::Matrix4::scaling(Mn::Vector3(gripperQueryRadius, gripperQueryRadius, gripperQueryRadius));
      addDebugInstance("sphere_green", b, mat);
#endif          

      int grippedFreeObjectIndex = episodeInstance.colGrid_.contactTest(gripperQueryWorldOrigin, gripperQueryRadius);
      if (grippedFreeObjectIndex != -1) {

        removeFreeObjectFromCollisionGrid(b, grippedFreeObjectIndex);
        robotInstance.grippedFreeObjectIndex_ = grippedFreeObjectIndex;
        episodeInstance.movedFreeObjectIndexes_.push_back(grippedFreeObjectIndex);
      }
    }

    if (robotInstance.doAttemptDrop_ && robotInstance.grippedFreeObjectIndex_ != -1) {

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
      const auto& columnGrid = stageFixedObject.columnGrid_;
      auto heldObjMat = getHeldObjectTransform(b);
      const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
        episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
      const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

      constexpr int maxFailedPlacements = 3;
      PlacementHelper placementHelper(columnGrid, episodeInstance.colGrid_, random_, maxFailedPlacements);
      bool success = placementHelper.place(heldObjMat, freeObject);

      if (success) {
        const auto rotationQuat = Mn::Quaternion::fromMatrix(heldObjMat.rotation());
        reinsertFreeObject(b, freeObjectIndex, heldObjMat.translation(), rotationQuat);
      } else {
        // hack: remove object from scene visually
        const auto glMat = toGlmMat4x3(Mn::Matrix4::translation({0.f, -10000.f, 0.f})); 
        int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
        env.updateInstanceTransform(instanceId, glMat);
      }
      robotInstance.grippedFreeObjectIndex_ = -1;
    }
  }
}


void BatchedSimulator::updateCollision() {

  const int numEnvs = config_.numEnvs;
  constexpr float sphereRadius = 0.1f; // todo: get from Robot

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

#ifdef ENABLE_DEBUG_INSTANCES
        auto mat = Mn::Matrix4::translation(spherePos)
          * Mn::Matrix4::scaling(Mn::Vector3(sphereRadius, sphereRadius, sphereRadius));
        addDebugInstance("sphere_orange", b, mat);
#endif
        break;
      }
    }

    robots_.collisionResults_[b] = hit;
    if (hit) {
      numRecentStepsInCollision_++;
    }
  }

// batch API didn't prove to be any faster, so we aren't using it
// #define UPDATE_COLLISION_USE_BATCH_TEST
#define USE_COLLISION_BROADPHASE_GRID
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
      const auto& spherePos = robots_.collisionSphereWorldOrigins_[sphereIndex];

#ifdef USE_COLLISION_BROADPHASE_GRID
      if (episodeInstance.colGrid_.contactTest(spherePos, sphereRadius) != -1) {
#ifdef ENABLE_DEBUG_INSTANCES
        {
          auto lastContact = episodeInstance.colGrid_.getLastContact();
          Mn::Matrix4 mat = Mn::Matrix4::from(
              lastContact.obsRotation.toMatrix(), lastContact.obsPos);

          // temp hack negative scale so the box back faces are rendered. Makes it easier to see contained object
          Mn::Matrix4 localToBox = Mn::Matrix4::translation(lastContact.obsLocalAabb.center())
            * Mn::Matrix4::scaling(-lastContact.obsLocalAabb.size() * 0.5);
          auto adjustedMat = mat * localToBox;
          addDebugInstance("cube_green", b, adjustedMat);

          mat = Mn::Matrix4::translation(spherePos)
            * Mn::Matrix4::scaling(Mn::Vector3(sphereRadius, sphereRadius, sphereRadius));

          addDebugInstance("sphere_orange", b, mat);
                    
        }
#endif
        hit = true;
        break;
      }
#else

      // todo: get pre-culled list of objects from a spatial data structure
      for (int i = 0; i < episode.numFreeObjectSpawns_; i++) {

        int globalFreeObjectIndex = b * episodeSet_.maxFreeObjects_ + i;

        // perf todo: consider storing 
        const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
          episode.firstFreeObjectSpawnIndex_ + i);
        const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

        auto instanceId = safeVectorGet(episodeInstanceSet_.freeObjectInstanceIds_, globalFreeObjectIndex);
#error this does not work with multithreading
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
#endif
    }

    robots_.collisionResults_[b] = hit;
    if (hit) {
      numRecentStepsInCollision_++;
    }
  }

  numRecentSteps_ += numEnvs;

// draw all collision spheres
#if 0 // #ifdef ENABLE_DEBUG_INSTANCES
  for (int b = 0; b < numEnvs; b++) {

    // only draw spheres for robots not in collision
    if (robots_.collisionResults_[b]) {
      continue;
    }

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
      addDebugInstance("sphere_green", b, mat);
    }
  }
#endif
}

void BatchedSimulator::postCollisionUpdate() {

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

  const int numEnvs = config_.numEnvs;
  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  if (!forceUpdate) {
    BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);
  }

  for (int b = 0; b < numEnvs; b++) {

    auto& robotInstance = robots_.robotInstances_[b];
    auto& env = bpsWrapper_->envs_[b];

    bool didRobotMove = forceUpdate || !robots_.collisionResults_[b];

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

        const auto& glMat = robots_.nodeNewTransforms_[instanceIndex];
        env.updateInstanceTransform(instanceId, glMat);
      }

      // todo: cleaner handling of camera update that doesn't depend on attachment or collision
      if (robot_.cameraAttachNode_ != -1) {
        auto cameraTransform = robotInstance.cameraAttachNodeTransform_ * robot_.cameraAttachTransform_;
        auto glCameraNewInvTransform = glm::inverse(toGlmMat4(cameraTransform));
        env.setCameraView(glCameraNewInvTransform);
      }
    }

    // update gripped free object
    if (didRobotMove && robotInstance.grippedFreeObjectIndex_ != -1 && didRobotMove) {

      int freeObjectIndex = robotInstance.grippedFreeObjectIndex_;
      auto mat = getHeldObjectTransform(b);
      glm::mat4x3 glMat = toGlmMat4x3(mat); 

      int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
      env.updateInstanceTransform(instanceId, glMat);
    }
  }

  // add debug ground lines
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

  areRenderInstancesUpdated_ = true;
}


Magnum::Matrix4 BatchedSimulator::getHeldObjectTransform(int b) const {

  auto& episodeInstance = safeVectorGet(episodeInstanceSet_.episodeInstanceByEnv_, b);
  const auto& episode = safeVectorGet(episodeSet_.episodes_, episodeInstance.episodeIndex_);
  auto& robotInstance = robots_.robotInstances_[b];
  BATCHED_SIM_ASSERT(robotInstance.grippedFreeObjectIndex_ != -1);

  const int freeObjectIndex = robotInstance.grippedFreeObjectIndex_;

  const auto& freeObjectSpawn = safeVectorGet(episodeSet_.freeObjectSpawns_, 
    episode.firstFreeObjectSpawnIndex_ + freeObjectIndex);
  const auto& freeObject = safeVectorGet(episodeSet_.freeObjects_, freeObjectSpawn.freeObjIndex_);

  // todo: decide how to orient gripped object. For now, we use startRotation #0 relative to gripper link
  constexpr int startRotationIndex = 0;
  const auto& rotation = safeVectorGet(freeObject.startRotations_, startRotationIndex);
  Mn::Matrix4 localToGripper = Mn::Matrix4::from(
      rotation, robot_.gripperQueryOffset_);
  auto mat = robotInstance.cachedGripperLinkMat_ * localToGripper;
  return mat;
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
  const int numOtherActions = 1; // doAttemptGrip/doAttemptDrop
  const int numJointDegrees = robot_.numPosVars;
  const int numBaseDegrees = 2;  // rotate and move-forward/back

  int batchNumActions = (numOtherActions + numJointDegrees + numBaseDegrees) * numEnvs;
  actions_.resize(batchNumActions, 0.f);

  maxRolloutSteps_ = config_.maxEpisodeLength;
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

  if (config_.doAsyncPhysicsStep) {
    physicsThread_ = std::thread(&BatchedSimulator::physicsThreadFunc, this, 0, config_.numEnvs);
    areRenderInstancesUpdated_ = false;
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
  episodeInstance.stageFixedObjectInstanceId_ = env.addInstance(
    stageBlueprint.meshIdx_, stageBlueprint.mtrlIdx_, identityGlMat_);

  {
    const auto& stageFixedObject = safeVectorGet(episodeSet_.fixedObjects_, episode.stageFixedObjIndex);
    const auto& columnGrid = stageFixedObject.columnGrid_;

    // todo: find extents for entire EpisodeSet, not just this specific columnGrid
    constexpr int maxBytes = 100 * 1024;
    // this is tuned assuming a building-scale simulation with household-object-scale obstacles
    constexpr float maxGridSpacing = 0.5f;
    constexpr float sphereRadius = 0.1f; // todo: get from robot_
    episodeInstance.colGrid_ = CollisionBroadphaseGrid(sphereRadius, 
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

  if (robotInstance.grippedFreeObjectIndex_ != -1) {
    // nothing to do here.
    // held free object was already in movedFreeObjectIndexes_
    robotInstance.grippedFreeObjectIndex_ = -1;
  }

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
    int instanceId = env.addInstance(blueprint.meshIdx_, blueprint.mtrlIdx_, glMat);
    // store the first free object's bps instanceId and assume the rest will be contiguous
    if (freeObjectIndex == 0) {
      episodeInstance.firstFreeObjectInstanceId_ = instanceId;
    }
    BATCHED_SIM_ASSERT(instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex));
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

  // sloppy quat to Matrix3x3
  Mn::Matrix4 mat = Mn::Matrix4::from(
      rotation.toMatrix(), pos);
  glm::mat4x3 glMat = toGlmMat4x3(mat); 
  int instanceId = getFreeObjectBpsInstanceId(b, freeObjectIndex);
  env.updateInstanceTransform(instanceId, glMat);
}

void BatchedSimulator::initEpisodeSet() {

  const int numEnvs = config_.numEnvs;

  // generate exactly as many episodes as envs (this is not required)
  episodeSet_ = generateBenchmarkEpisodeSet(numEnvs, sceneMapping_);

  episodeInstanceSet_.episodeInstanceByEnv_.resize(numEnvs);
  for (int b = 0; b < numEnvs; b++) {
    const auto episodeIndex = b * episodeSet_.episodes_.size() / numEnvs; // distribute episodes across instances
    instantiateEpisode(b, episodeIndex);
  }
}

void BatchedSimulator::setActions(std::vector<float>&& actions) {
  ESP_CHECK(actions.size() == actions_.size(),
            "BatchedSimulator::setActions: input dimension should be " +
                std::to_string(actions_.size()) + ", not " +
                std::to_string(actions.size()));

  if (config_.forceRandomActions) {
    for (auto& action : actions_) {
      action = random_.uniform_float(-1.f, 1.f);
    }
  } else {
    actions_ = std::move(actions);
  }
}

void BatchedSimulator::reset() {
  int numEnvs = bpsWrapper_->envs_.size();

  BATCHED_SIM_ASSERT(isAsyncStepPhysicsFinished_);
  BATCHED_SIM_ASSERT(!signalStepPhysics_);

  currRolloutStep_ = 0;
  prevRolloutStep_ = -1;
  randomizeRobotsForCurrentStep();
  robots_.updateLinkTransforms(currRolloutStep_);
  std::fill(robots_.hackRewards_.begin(), robots_.hackRewards_.end(), 0.f);
  for (int b = 0; b < numEnvs; b++) {
    resetEpisodeInstance(b);
  }
  updateRenderInstances(/*forceUpdate*/true);

  isOkToRender_ = true;
  isOkToStep_ = true;
}

void BatchedSimulator::autoResetOrStepPhysics() {

  BATCHED_SIM_ASSERT(!config_.doAsyncPhysicsStep);

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
    calcRewards();
    updateRenderInstances(/*forceUpdate*/false);
  }
}

void BatchedSimulator::autoResetOrStartAsyncStepPhysics() {

  BATCHED_SIM_ASSERT(config_.doAsyncPhysicsStep);

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

    BATCHED_SIM_ASSERT(isAsyncStepPhysicsFinished_);
    BATCHED_SIM_ASSERT(!signalStepPhysics_);

    // send message to physicsThread_
    signalStepPhysics();
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
    auto& robotInstance = robots_.robotInstances_[b];
    const float grabDropAction = actions_[actionIndex++];
    robotInstance.doAttemptGrip_ = grabDropAction > 0.f 
      && robotInstance.grippedFreeObjectIndex_ == -1;
    robotInstance.doAttemptDrop_ = grabDropAction <= 0.f 
      && robotInstance.grippedFreeObjectIndex_ != -1;

    constexpr float maxAbsAngle = float(Mn::Rad(Mn::Deg(10.f)));
    const float clampedBaseYawAction = Mn::Math::clamp(actions_[actionIndex++], -maxAbsAngle, maxAbsAngle);
    yaws[b] = prevYaws[b] + clampedBaseYawAction; // todo: wrap angle to 360 degrees
    float clampedBaseMovementAction = Mn::Math::clamp(actions_[actionIndex++], -0.1f, 0.2f);
    positions[b] =
        prevPositions[b] + 
        Mn::Vector2(Mn::Math::cos(Mn::Math::Rad(yaws[b])), -Mn::Math::sin(Mn::Math::Rad(yaws[b]))) 
        * clampedBaseMovementAction;

    int baseJointIndex = b * robot_.numPosVars;
    for (int j = 0; j < robot_.numPosVars; j++) {
      auto& pos = jointPositions[baseJointIndex + j];
      const auto& prevPos = prevJointPositions[baseJointIndex + j];
      constexpr float maxAbsAngle = float(Mn::Rad(Mn::Deg(15.f)));
      const float clampedJointMovementAction = Mn::Math::clamp(actions_[actionIndex++], -maxAbsAngle, maxAbsAngle);
      pos = prevPos + clampedJointMovementAction;
      pos = Mn::Math::clamp(pos, robot_.jointPositionLimits.first[j],
                            robot_.jointPositionLimits.second[j]);

    }
  }
  BATCHED_SIM_ASSERT(actionIndex == actions_.size());

  robots_.updateLinkTransforms(currRolloutStep_);

  updateCollision();

  updateGripping();

  postCollisionUpdate();

  // robots_.applyActionPenalties(actions_);
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

  deleteDebugInstances();
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

  BATCHED_SIM_ASSERT(robots_.areCollisionResultsValid_);

  // update rewards on main thread
  const int numEnvs = config_.numEnvs;
  for (int b = 0; b < numEnvs; b++) {
    robots_.hackRewards_[b] = robots_.collisionResults_[b] ? -1.f : 0.f;
  }

  // int numEnvs = bpsWrapper_->envs_.size();
  // rewardContext_.calcRewards(currRolloutStep_, 0, numEnvs);
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

float BatchedSimulator::getRecentCollisionFractionAndReset() const {
  float retVal = numRecentSteps_ > 0 ? (float)numRecentStepsInCollision_ / numRecentSteps_ : -1.f;
  numRecentSteps_ = 0;
  numRecentStepsInCollision_ = 0;
  return retVal;
}

void BatchedSimulator::signalStepPhysics() {
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


void BatchedSimulator::waitAsyncStepPhysics() {

  if (areRenderInstancesUpdated_) {
    return;
  }

  {
    std::unique_lock<std::mutex> lck(physicsMutex_);
    physicsCondVar_.wait(lck, [&]{ return isAsyncStepPhysicsFinished_; });
  }

  calcRewards();

  updateRenderInstances(/*forceUpdate*/false);
}

void BatchedSimulator::physicsThreadFunc(int startEnvIndex, int numEnvs) {

  while (true) {
    {
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

    {
      std::lock_guard<std::mutex> lck(physicsMutex_);
      isAsyncStepPhysicsFinished_ = true;
      signalStepPhysics_ = false;
      physicsCondVar_.notify_one(); // notify_all?
    }
  }
}

}  // namespace batched_sim
}  // namespace esp
