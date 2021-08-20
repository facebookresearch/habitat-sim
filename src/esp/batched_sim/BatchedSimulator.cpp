// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchedSimulator.h"
#include "GlmUtils.h"

#include "esp/gfx/replay/Keyframe.h"
#include "esp/io/json.h"

// #include <bps3D.hpp>

#include <iostream>

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

RobotInstanceSet::RobotInstanceSet(Robot* robot,
                                   int batchSize,
                                   std::vector<bps3D::Environment>* envs)
    : batchSize_(batchSize), envs_(envs), robot_(robot) {
  int numLinks = robot->artObj->getNumLinks();
  int numNodes = numLinks + 1;  // include base
  int batchNumPosVars = robot_->numPosVars * batchSize;
  int batchNumNodes = numNodes * batchSize;
  jointPositions_.resize(batchNumPosVars, 0);
  rootTransforms_.resize(batchSize,
                         Magnum::Matrix4(Magnum::Math::IdentityInit));
  nodeTransforms_.resize(batchNumNodes,
                         Magnum::Matrix4(Magnum::Math::IdentityInit));
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

void RobotInstanceSet::updateLinkTransforms() {
  esp::gfx::replay::Keyframe debugKeyframe;
  int numLinks = robot_->artObj->getNumLinks();
  int numNodes = numLinks + 1;

  auto* mb = robot_->artObj->btMultiBody_.get();
  int posCount = 0;

  for (int b = 0; b < batchSize_; b++) {
    btTransform tr{rootTransforms_[b]};
    mb->setBaseWorldTransform(tr);

    for (int i = 0; i < numLinks; ++i) {
      auto& link = mb->getLink(i);
      // optimization todo: find correct subset of links
      if (link.m_posVarCount > 0) {
        mb->setJointPosMultiDof(i,
                                const_cast<float*>(&jointPositions_[posCount]));
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
        // const float scale = (float)b / (batchSize_ - 1);
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

  {
    {
      rapidjson::Document document(rapidjson::kObjectType);
      rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
      esp::io::addMember(document, "debugKeyframe", debugKeyframe, allocator);
      esp::io::writeJsonToFile(document, "temp.replay.json");
    }
  }
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
  uint32_t batch_size = 11;  // todo: get from python
  glm::u32vec2 out_dim(1024, 1024);

  renderer_ = std::make_unique<bps3D::Renderer>(bps3D::RenderConfig{
      0, 1, batch_size, out_dim.x, out_dim.y, false,
      bps3D::RenderMode::Depth | bps3D::RenderMode::UnlitRGB});

  loader_ = std::make_unique<bps3D::AssetLoader>(renderer_->makeLoader());
  const std::string filepath =
      "/home/eundersander/projects/bps_data/combined_Stage_v3_sc0_staging/"
      "combined_Stage_v3_sc0_staging_trimesh.bps";
  scene_ = loader_->loadScene(filepath);

  const Mn::Vector3 camPos{-1.61004, 1.5, 3.5455};
  const Mn::Quaternion camRot{{0, -0.529178, 0}, 0.848511};
  glm::mat4 base(glm::inverse(toGlmMat4(camPos, camRot)));

  for (int b = 0; b < batch_size; b++) {
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
  // simConfig.physicsConfigFile = physicsConfigFile;

  legacySim_ = esp::sim::Simulator::create_unique(simConfig);

  const std::string filepath = "data/URDF/opt_fetch/robots/fetch.urdf";

  robot_ = Robot(filepath, legacySim_.get());

  const int batchSize = bpsWrapper_->envs_.size();
  simInstances_.robots =
      RobotInstanceSet(&robot_, batchSize, &bpsWrapper_->envs_);

  // todo: check that everything is in good state to render (even though we
  // haven't stepped)
}

void BatchedSimulator::stepPhysics() {
  int batchSize = bpsWrapper_->envs_.size();

  // todo: animate over time

  static float animTime = 0.f;
  animTime += 0.1f;

  Mn::Vector3 animBaseTranslationOffset{animTime * 0.2f, 0.f,
                                        0.f};      // translate at 0.2 m/s
  float animBaseRotationOffset = animTime * 90.f;  // rotate at 90 deg/s
  // move from -0.2 to 0.2 with period = 1s
  float animJointPosOffset = Mn::Math::sin(Mn::Deg(animTime * 360.f)) * 0.2f;

  // stepping code
  for (int b = 0; b < batchSize; b++) {
    simInstances_.robots.rootTransforms_[b] =
        Mn::Matrix4::translation(Mn::Vector3{1.61, 0.0, 0.98} +
                                 animBaseTranslationOffset) *
        Mn::Matrix4::rotation(Mn::Deg(-b * 10.f + animBaseRotationOffset),
                              {0.f, 1.f, 0.f}) *
        Mn::Matrix4::rotation(Mn::Deg(-90.f), {1.f, 0.f, 0.f});

    int baseJointIndex = b * robot_.numPosVars;
    float jointPos = b * 0.05 + animJointPosOffset;
    for (int j = 0; j < robot_.numPosVars; j++) {
      simInstances_.robots.jointPositions_[baseJointIndex + j] = jointPos;
    }
  }

  simInstances_.robots.updateLinkTransforms();
}

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

}  // namespace batched_sim
}  // namespace esp
