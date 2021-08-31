// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchedSimulator.h"
#include "GlmUtils.h"

#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/json.h"

// #include <bps3D.hpp>

#include <iostream>

#include <mutex>

namespace esp {
namespace batched_sim {

namespace {

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
  CORRADE_INTERNAL_ASSERT(index1 != -1);

  auto retval = filepath.substr(index0 + 1, index1 - index0 - 1);
  return retval;
}

}  // namespace

void BatchedSimulator::randomizeRobotsForCurrentStep() {
  CORRADE_INTERNAL_ASSERT(currRolloutStep_ >= 0);
  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  float* yaws = &rollouts_.yaws_[currRolloutStep_ * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutStep_ * numEnvs];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutStep_ * numEnvs * numPosVars];

  auto random = core::Random(/*seed*/ 0);
  for (int b = 0; b < numEnvs; b++) {
    yaws[b] = random.uniform_float(-float(Mn::Rad(Mn::Deg(90.f))), 0.f);
    positions[b] =
        Mn::Vector2(1.61, 0.98) + Mn::Vector2(random.uniform_float(-0.1, 0.1f),
                                              random.uniform_float(-0.1, 0.1f));
    for (int j = 0; j < robot_.numPosVars; j++) {
      auto& pos = jointPositions[b * robot_.numPosVars + j];
      pos = random.uniform_float(-0.1, 0.2f);
      pos = Mn::Math::clamp(pos, robot_.jointPositionLimits.first[j],
                            robot_.jointPositionLimits.second[j]);
    }
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

  const auto* mb = robot_->artObj->btMultiBody_.get();

  glm::mat4x3 identityMat(1.f, 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                          0.f);

  int baseInstanceIndex = 0;
  for (auto& env : *envs_) {
    // sloppy: pass -1 to getLinkVisualSceneNodes to get base
    for (int i = -1; i < numLinks; i++) {
      const auto& link = robot->artObj->getLink(i);  // -1 gets base link

      const auto& visualAttachments = link.visualAttachments_;

      CORRADE_INTERNAL_ASSERT(visualAttachments.size() <= 1);

      int instanceId = -1;
      if (!visualAttachments.empty()) {
        const std::string linkVisualFilepath = visualAttachments[0].second;
        const std::string nodeName =
            getMeshNameFromURDFVisualFilepath(linkVisualFilepath);
        const auto [meshIndex, mtrlIndex, scale] =
            robot_->sceneMapping.findMeshIndexMaterialIndexScale(nodeName);
        CORRADE_INTERNAL_ASSERT(scale == 1.f);

        instanceId = env.addInstance(meshIndex, mtrlIndex, identityMat);
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
}

void RobotInstanceSet::updateLinkTransforms(int currRolloutStep) {
  esp::gfx::replay::Keyframe debugKeyframe;
  int numLinks = robot_->artObj->getNumLinks();
  int numNodes = numLinks + 1;
  int numEnvs = numEnvs_;
  int numPosVars = robot_->numPosVars;

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

        env.updateInstanceTransform(instanceId, toGlmMat4x3(mat));

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

#if 0
    if (b == 0) {
      std::ostringstream ss;
      auto* mb = robot_->artObj->btMultiBody_.get();

      int numLinks = robot_->artObj->getNumLinks();
      for (int i = -1; i < numLinks; i++) {
        const auto nodeIndex = i + 1;                   // 0 is base
        const auto& link = robot_->artObj->getLink(i);  // -1 gets base link
        const auto& visualAttachments = link.visualAttachments_;
        if (!visualAttachments.empty()) {
          const std::string linkVisualFilepath = visualAttachments[0].second;
          ss << linkVisualFilepath << "\n";

          // ss << "m_cachedWorldTransform: ";
          const auto btTrans = i == -1 ? mb->getBaseWorldTransform()
                                       : mb->getLink(i).m_cachedWorldTransform;
          for (int i = 0; i < 3; i++) {
            ss << btTrans.getBasis().getRow(i).x() << " ";
            ss << btTrans.getBasis().getRow(i).y() << " ";
            ss << btTrans.getBasis().getRow(i).z() << "\n";
          }
          ss << btTrans.getOrigin().x() << " ";
          ss << btTrans.getOrigin().y() << " ";
          ss << btTrans.getOrigin().z() << " ";
          ss << "\n";

          Mn::Matrix4 mat = toMagnumMatrix4(btTrans);

          const auto& tmp = robot_->nodeTransformFixups[nodeIndex];

          auto printTransform = [](const Mn::Matrix4& mat,
                                   std::ostringstream& ss) {
            for (int i = 0; i < 4; i++) {
              ss << mat[i][0] << " ";
              ss << mat[i][1] << " ";
              ss << mat[i][2] << "\n";
            }
          };

          ss << "fixup:\n";
          printTransform(tmp, ss);

          mat = mat * tmp;
          ss << "mat with fixup:\n";
          printTransform(mat, ss);
        }
      }

      ss << "\n";
      LOG(WARNING) << ss.str();
    }
#endif
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

Robot::Robot(const std::string& filepath, esp::sim::Simulator* sim) {
  // todo: delete object on destruction
  auto managedObj =
      sim->getArticulatedObjectManager()->addBulletArticulatedObjectFromURDF(
          filepath);

  artObj = static_cast<esp::physics::BulletArticulatedObject*>(
      managedObj->hackGetBulletObjectReference().get());
  sceneMapping = BpsSceneMapping::loadFromFile(
      "/home/eundersander/projects/bps_data/combined_Stage_v3_sc0_staging/"
      "combined_Stage_v3_sc0_staging_trimesh.bps.mapping.json");

  jointPositionLimits = artObj->getJointPositionLimits();

  int numLinks = artObj->getNumLinks();
  int numNodes = numLinks + 1;
  nodeTransformFixups.resize(numNodes);

  // Sloppy: this is needed for correctness; I think it's because I preprocessed
  // from GLB to bps with the wrong axes specified on the command line.
  const auto globalFixup =
      Mn::Matrix4::rotation(Mn::Deg(90.f), {1.f, 0.f, 0.f});

  for (int i = -1; i < numLinks; i++) {
    const auto nodeIndex = i + 1;           // 0 is base
    const auto& link = artObj->getLink(i);  // -1 gets base link
    const auto& visualAttachments = link.visualAttachments_;
    CORRADE_INTERNAL_ASSERT(visualAttachments.size() <= 1);
    int instanceId = -1;
    if (!visualAttachments.empty()) {
      const auto* sceneNode = visualAttachments[0].first;
      int nodeIndex = i + 1;  // 0 for base
      // This transform comes from the visual origin specified in the URDF;
      // it is essentially an additional transform to apply to the visual mesh.
      const auto tmp = sceneNode->transformation();
      nodeTransformFixups[nodeIndex] = tmp * globalFixup;
    }
  }

  numPosVars = artObj->getJointPositions().size();
  CORRADE_INTERNAL_ASSERT(numPosVars > 0);
}

BpsWrapper::BpsWrapper() {
  uint32_t numEnvs = 128;  // todo: get from python
  glm::u32vec2 out_dim(256,
                       256);  // see also rollout_test.py, python/rl/agent.py

  renderer_ = std::make_unique<bps3D::Renderer>(bps3D::RenderConfig{
      0, 1, numEnvs, out_dim.x, out_dim.y, false,
      bps3D::RenderMode::Depth | bps3D::RenderMode::UnlitRGB});

  loader_ = std::make_unique<bps3D::AssetLoader>(renderer_->makeLoader());
  const std::string filepath =
      "/home/eundersander/projects/bps_data/combined_Stage_v3_sc0_staging/"
      "combined_Stage_v3_sc0_staging_trimesh.bps";
  scene_ = loader_->loadScene(filepath);

  const Mn::Vector3 camPos{-1.61004, 1.5, 3.5455};
  const Mn::Quaternion camRot{{0, -0.529178, 0}, 0.848511};
  glm::mat4 base(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < numEnvs; b++) {
    glm::mat4 view = base;
    auto env = renderer_->makeEnvironment(scene_, view, /*fov*/ 45.f, 0.f, 0.01,
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

BatchedSimulator::BatchedSimulator() {
  bpsWrapper_ = std::make_unique<BpsWrapper>();

  esp::sim::SimulatorConfiguration simConfig{};
  simConfig.activeSceneName = "NONE";
  simConfig.enablePhysics = true;
  simConfig.createRenderer = false;
  simConfig.loadRenderAssets = false;
  // simConfig.physicsConfigFile = physicsConfigFile;

  legacySim_ = esp::sim::Simulator::create_unique(simConfig);

  const std::string filepath = "data/URDF/opt_fetch/robots/fetch.urdf";

  robot_ = Robot(filepath, legacySim_.get());
  int numLinks = robot_.artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base

  const int numEnvs = bpsWrapper_->envs_.size();
  robots_ = RobotInstanceSet(&robot_, numEnvs, &bpsWrapper_->envs_, &rollouts_);

  // see also python/rl/agent.py
  const int numJointDegrees = robot_.numPosVars;
  const int numBaseDegrees = 2;  // rotate and move-forward/back

  int batchNumActions = (numJointDegrees + numBaseDegrees) * numEnvs;
  actions_.resize(batchNumActions, 0.f);

  maxRolloutSteps_ = 32;
  currRolloutStep_ = 0;
  rollouts_ =
      RolloutRecord(maxRolloutSteps_, numEnvs, robot_.numPosVars, numNodes);

  rewardContext_ = RewardCalculationContext(&robot_, numEnvs, &rollouts_);

  randomizeRobotsForCurrentStep();
  robots_.updateLinkTransforms(currRolloutStep_);

  // todo: check that everything is in good state to render (even though we
  // haven't stepped)
}

void BatchedSimulator::setActions(std::vector<float>&& actions) {
  ESP_CHECK(actions.size() == actions_.size(),
            "BatchedSimulator::setActions: input dimension should be " +
                std::to_string(actions_.size()) + ", not " +
                std::to_string(actions.size()));
  actions_ = std::move(actions);
}

void BatchedSimulator::stepPhysics() {
  int prevRolloutStep = currRolloutStep_;
  currRolloutStep_++;

  // temp reset rollout
  if (currRolloutStep_ == maxRolloutSteps_) {
    currRolloutStep_ = 0;
  }

  int numEnvs = bpsWrapper_->envs_.size();
  int numPosVars = robot_.numPosVars;

  auto& robots = robots_;

  int actionIndex = 0;

  const float* prevYaws = &rollouts_.yaws_[prevRolloutStep * numEnvs];
  float* yaws = &rollouts_.yaws_[currRolloutStep_ * numEnvs];
  const Mn::Vector2* prevPositions =
      &rollouts_.positions_[prevRolloutStep * numEnvs];
  Mn::Vector2* positions = &rollouts_.positions_[currRolloutStep_ * numEnvs];
  Mn::Matrix4* rootTransforms =
      &rollouts_.rootTransforms_[currRolloutStep_ * numEnvs];
  const float* prevJointPositions =
      &rollouts_.jointPositions_[prevRolloutStep * numEnvs * numPosVars];
  float* jointPositions =
      &rollouts_.jointPositions_[currRolloutStep_ * numEnvs * numPosVars];

  // stepping code
  for (int b = 0; b < numEnvs; b++) {
    yaws[b] = prevYaws[b] +
              actions_[actionIndex++] * 5.f;  // todo: wrap angle to 360 degrees
    // note clamp move-forward action to [0,-]
    constexpr float baseMovementSpeed =
        0.f;  // temp disable movement so that robot stays on-screen
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
  CORRADE_INTERNAL_ASSERT(actionIndex == actions_.size());

  robots_.updateLinkTransforms(currRolloutStep_);
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
  bpsWrapper_->renderer_->render(bpsWrapper_->envs_.data());
}

void BatchedSimulator::waitForFrame() {
  bpsWrapper_->renderer_->waitForFrame();
}

bps3D::Renderer& BatchedSimulator::getBpsRenderer() {
  CORRADE_INTERNAL_ASSERT(bpsWrapper_->renderer_.get());
  return *bpsWrapper_->renderer_.get();
}

RewardCalculationContext::RewardCalculationContext(const Robot* robot,
                                                   int numEnvs,
                                                   RolloutRecord* rollouts)
    : robot_(robot), numEnvs_(numEnvs), rollouts_(rollouts) {
  esp::sim::SimulatorConfiguration simConfig{};
  simConfig.activeSceneName =
      "data/stages/Stage_v3_sc0_staging_no_textures.glb";
  simConfig.enablePhysics = true;
  simConfig.createRenderer = false;
  simConfig.loadRenderAssets =
      false;  // todo: avoid creating render assets for stage
  // simConfig.physicsConfigFile = physicsConfigFile;

  legacySim_ = esp::sim::Simulator::create_unique(simConfig);

  // todo: avoid code duplication with Robot
  const std::string filepath = "data/URDF/opt_fetch/robots/fetch.urdf";
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

#if 0
class ThreadPool {

  struct Job {
    int currRolloutStep,
    int bStart,
    int bEnd
  };

  void createThreadPool();
  void loopFunction(int workerThread);
  void onResetRollouts() {
    {
      unique_lock<mutex> lock(threadpool_mutex_);
      CORRADE_INTERNAL_ASSERT(jobStack_.empty());
      // todo: assert not waiting for jobs to finish
    }
    numQueuedSteps_ = 0;
  }

  std::mutex threadpool_mutex_;
  bool terminate_pool_; // protect with mutex

  std::vector<Job> jobStack_;
  std::vector<RewardCalculationContext> rewardContextPerThread_;
  int numQueuedSteps_ = 0;
  int numActiveJobs_ = 0; // protect with mutex
};


void ThreadPool::createThreadPool() {
  int numThreads = thread::hardware_concurrency();
  vector<thread> pool;
  for(int ii = 0; ii < numThreads; ii++) {
    pool.push_back(thread(ThreadPool::loopFunction, ii));
  }
}

// runs on main thread
void ThreadPool::tryQueueWork(int currRolloutStep) {

  // todo: check for off-by-one
  if (currRolloutStep == numQueuedSteps_) {
    return;
  }

  {
    unique_lock<mutex> lock(threadpool_mutex_);
    while (currRolloutStep > numQueuedSteps_) {
      // temp: for now, queue all envs for one step as single job
      jobStack_.push_back(Job{numQueuedSteps_, 0, numEnvs_});
      numQueuedSteps_++;
    }


}

void ThreadPool::~ThreadPool() {
{
  {
    unique_lock<mutex> lock(threadpool_mutex_);
    terminate_pool_ = true; // use this flag in condition.wait
  }
  condition.notify_all(); // wake up all threads.

  // Join all threads.
  for(std::thread &every_thread : thread_vector) {
    every_thread.join();
  }

  thread_vector.clear();
  stopped = true; // use this flag in destructor, if not set, call shutdown()

}

void issueCalculateRewardJobs() {

  Pool_Obj.Add_Job(std::bind(&Some_Class::Some_Method, &Some_object));

}

void ThreadPool::loopFunction(int threadIndex)
{
  while(true)
  {
    {
        unique_lock<mutex> lock(threadpool_mutex_);
        condition.wait(lock, [this](){return !Queue.empty() || terminate_pool_;});
        if (terminate_pool_) {
          break;
        }
        Job = Queue.front();
        Queue.pop();
    }
    Job(); // function<void()> type
  }
};

void The_Pool::Add_Job(function<void()> New_Job)
{
    {
         unique_lock<mutex> lock(Queue_Mutex);
         Queue.push(New_Job);
    }
    condition.notify_one();
}
#endif

}  // namespace batched_sim
}  // namespace esp
