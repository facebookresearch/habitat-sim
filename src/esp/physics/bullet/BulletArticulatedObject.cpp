// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Construction code adapted from Bullet3/examples/

#include "BulletArticulatedObject.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletPhysicsManager.h"
#include "BulletURDFImporter.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace physics {

// set node state from btTransform
// TODO: this should probably be moved
static void setRotationScalingFromBulletTransform(const btTransform& trans,
                                                  scene::SceneNode* node) {
  Mn::Matrix4 converted{trans};
  node->setRotation(Mn::Quaternion::fromMatrix(converted.rotation()));
  node->setTranslation(converted.translation());
}

///////////////////////////////////
// Class functions
///////////////////////////////////

BulletArticulatedObject::~BulletArticulatedObject() {
  // ESP_DEBUG()     << "deconstructing ~BulletArticulatedObject";
  if (objectMotionType_ == MotionType::DYNAMIC) {
    // KINEMATIC and STATIC objects have already been removed from the world.
    bWorld_->removeMultiBody(btMultiBody_.get());
  }
  // remove link collision objects from world
  for (int colIx = 0; colIx < btMultiBody_->getNumLinks(); ++colIx) {
    auto* linkCollider = btMultiBody_->getLinkCollider(colIx);
    bWorld_->removeCollisionObject(linkCollider);
    collisionObjToObjIds_->erase(linkCollider);
    delete linkCollider;
  }

  // remove fixed base rigid body
  if (bFixedObjectRigidBody_) {
    bWorld_->removeRigidBody(bFixedObjectRigidBody_.get());
    collisionObjToObjIds_->erase(bFixedObjectRigidBody_.get());
    bFixedObjectRigidBody_ = nullptr;
    bFixedObjectShape_ = nullptr;
  }

  // remove base collider
  auto* baseCollider = btMultiBody_->getBaseCollider();
  bWorld_->btCollisionWorld::removeCollisionObject(baseCollider);
  collisionObjToObjIds_->erase(baseCollider);
  delete baseCollider;

  // remove motors from the world
  for (auto& motorId : getExistingJointMotors()) {
    BulletArticulatedObject::removeJointMotor(motorId.first);
  }

  // remove joint limit constraints
  for (auto& jlIter : jointLimitConstraints) {
    bWorld_->removeMultiBodyConstraint(jlIter.second.con);
    delete jlIter.second.con;
  }
}

void BulletArticulatedObject::initializeFromURDF(
    URDFImporter& urdfImporter,
    const Mn::Matrix4& worldTransform,
    scene::SceneNode* physicsNode) {
  Mn::Matrix4 rootTransformInWorldSpace{worldTransform};

  BulletURDFImporter& u2b = *(static_cast<BulletURDFImporter*>(&urdfImporter));

  auto urdfModel = u2b.getModel();

  node().setSemanticId(urdfModel->getSemanticId());

  // cache the global scaling from the source model
  globalScale_ = urdfModel->getGlobalScaling();

  int urdfLinkIndex = u2b.getRootLinkIndex();

  bool recursive = (u2b.flags & CUF_MAINTAIN_LINK_ORDER) == 0;

  if (recursive) {
    // NOTE: recursive path only
    u2b.convertURDF2BulletInternal(urdfLinkIndex, rootTransformInWorldSpace,
                                   bWorld_.get(), linkCompoundShapes_,
                                   linkChildShapes_, recursive);
  } else {
    std::vector<Mn::Matrix4> parentTransforms;
    parentTransforms.resize(urdfLinkIndex + 1);
    parentTransforms[urdfLinkIndex] = rootTransformInWorldSpace;
    std::vector<childParentIndex> allIndices;

    u2b.getAllIndices(urdfLinkIndex, -1, allIndices);
    std::sort(allIndices.begin(), allIndices.end(),
              [](const childParentIndex& a, const childParentIndex& b) {
                return a.m_index < b.m_index;
              });

    if (allIndices.size() + 1 > parentTransforms.size()) {
      parentTransforms.resize(allIndices.size() + 1);
    }
    for (size_t i = 0; i < allIndices.size(); ++i) {
      int urdfLinkIndex = allIndices[i].m_index;
      int parentIndex = allIndices[i].m_parentIndex;
      Mn::Matrix4 parentTr = parentIndex >= 0 ? parentTransforms[parentIndex]
                                              : rootTransformInWorldSpace;
      Mn::Matrix4 tr = u2b.convertURDF2BulletInternal(
          urdfLinkIndex, parentTr, bWorld_.get(), linkCompoundShapes_,
          linkChildShapes_, recursive);
      parentTransforms[urdfLinkIndex] = tr;
    }
  }

  if (u2b.cache->m_bulletMultiBody) {
    btMultiBody* mb = u2b.cache->m_bulletMultiBody;
    jointLimitConstraints = u2b.cache->m_jointLimitConstraints;

    mb->setHasSelfCollision((u2b.flags & CUF_USE_SELF_COLLISION) !=
                            0);  // NOTE: default no

    mb->finalizeMultiDof();

    btTransform localInertialFrameRoot =
        u2b.cache->m_urdfLinkLocalInertialFrames[urdfLinkIndex];

    {
      mb->setBaseWorldTransform(btTransform(rootTransformInWorldSpace) *
                                localInertialFrameRoot);
    }
    {
      btAlignedObjectArray<btQuaternion> scratch_q;
      btAlignedObjectArray<btVector3> scratch_m;
      mb->forwardKinematics(scratch_q, scratch_m);
      mb->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
    }

    btMultiBody_.reset(
        mb);  // take ownership of the object in the URDFImporter cache
    bWorld_->addMultiBody(btMultiBody_.get());
    btMultiBody_->setCanSleep(true);

    // construct separate fixed base rigid object proxy for marked links
    constructStaticRigidBaseObject();

    // create the BulletArticulatedLinks
    for (size_t urdfLinkIx = 0;
         urdfLinkIx < urdfImporter.getModel()->m_links.size(); ++urdfLinkIx) {
      int bulletLinkIx =
          u2b.cache->m_urdfLinkIndices2BulletLinkIndices[urdfLinkIx];
      auto urdfLink = u2b.getModel()->getLink(urdfLinkIx);

      ArticulatedLink* linkObject = nullptr;
      if (bulletLinkIx >= 0) {
        links_[bulletLinkIx] = std::make_unique<BulletArticulatedLink>(
            &physicsNode->createChild(), resMgr_, bWorld_, bulletLinkIx,
            collisionObjToObjIds_);
        linkObject = links_[bulletLinkIx].get();
        linkObject->linkJointName = urdfLink->m_parentJoint.lock()->m_name;
      } else {
        if (!baseLink_) {
          baseLink_ = std::make_unique<BulletArticulatedLink>(
              &node().createChild(), resMgr_, bWorld_, bulletLinkIx,
              collisionObjToObjIds_);
        }
        linkObject = baseLink_.get();
      }
      linkObject->linkName = urdfLink->m_name;

      linkObject->node().setType(esp::scene::SceneNodeType::OBJECT);
    }

    // Build damping motors
    for (int linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
      btMultibodyLink& link = btMultiBody_->getLink(linkIx);
      JointMotorSettings settings;
      settings.maxImpulse = double(link.m_jointDamping);
      if (supportsSingleDofJointMotor(linkIx)) {
        settings.motorType = JointMotorType::SingleDof;
        createJointMotor(linkIx, settings);
      } else if (link.m_jointType == btMultibodyLink::eSpherical) {
        settings.motorType = JointMotorType::Spherical;
        createJointMotor(linkIx, settings);
      }
    }
    // set user config attributes from model.
    setUserAttributes(urdfModel->getUserConfiguration());

    // in case the base transform is not zero by default
    syncPose();
  }
}

void BulletArticulatedObject::constructStaticRigidBaseObject() {
  // start explicitly with base collider
  btCollisionObject* col = btMultiBody_->getBaseCollider();
  btTransform tr;
  tr.setIdentity();
  for (int m = -1; m < btMultiBody_->getNumLinks(); ++m) {
    if (m >= 0) {
      // prepare for link
      col = btMultiBody_->getLink(m).m_collider;
      tr = col->getWorldTransform();
      if (col == nullptr) {
        continue;
      }
    }
    if (col->getBroadphaseHandle()->m_collisionFilterGroup ==
        int(CollisionGroup::Static)) {
      // accumulate shapes in the new compound
      if (bFixedObjectShape_ == nullptr) {
        bFixedObjectShape_ = std::make_unique<btCompoundShape>();
      }
      bFixedObjectShape_->addChildShape(tr, col->getCollisionShape());
      // disable collisions for the old object
      col->getBroadphaseHandle()->m_collisionFilterGroup =
          int(CollisionGroup::Noncollidable);
      col->getBroadphaseHandle()->m_collisionFilterMask = uint32_t(
          CollisionGroupHelper::getMaskForGroup(CollisionGroup::Noncollidable));
    }
  }

  // create the proxy btRigidBody to replace the static shapes
  if (bFixedObjectShape_ != nullptr) {
    btRigidBody::btRigidBodyConstructionInfo info =
        btRigidBody::btRigidBodyConstructionInfo(0.f, nullptr,
                                                 bFixedObjectShape_.get());
    bFixedObjectRigidBody_ = std::make_unique<btRigidBody>(info);
    bWorld_->addRigidBody(
        bFixedObjectRigidBody_.get(), int(CollisionGroup::Static),
        uint32_t(
            CollisionGroupHelper::getMaskForGroup(CollisionGroup::Static)));
    collisionObjToObjIds_->emplace(bFixedObjectRigidBody_.get(), objectId_);
  }
}

void BulletArticulatedObject::updateNodes(bool force) {
  isDeferringUpdate_ = false;
  if (force || btMultiBody_->getBaseCollider()->isActive()) {
    setRotationScalingFromBulletTransform(btMultiBody_->getBaseWorldTransform(),
                                          &node());
  }
  // update link transforms
  for (auto& link : links_) {
    if (force || btMultiBody_->getLinkCollider(link.first)->isActive())
      setRotationScalingFromBulletTransform(
          btMultiBody_->getLink(link.first).m_cachedWorldTransform,
          &link.second->node());
  }
}

////////////////////////////
// BulletArticulatedLink
////////////////////////////

std::shared_ptr<metadata::attributes::SceneAOInstanceAttributes>
BulletArticulatedObject::getCurrentStateInstanceAttr() {
  // get mutable copy of initialization SceneAOInstanceAttributes for this AO
  auto sceneArtObjInstanceAttr =
      ArticulatedObject::getCurrentStateInstanceAttr();
  if (!sceneArtObjInstanceAttr) {
    // if no scene instance attributes specified, no initial state is set
    return nullptr;
  }
  sceneArtObjInstanceAttr->setAutoClampJointLimits(autoClampJointLimits_);

  const std::vector<float> jointPos = getJointPositions();
  int i = 0;
  for (const float& v : jointPos) {
    const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
    sceneArtObjInstanceAttr->addInitJointPoseVal(key, v);
  }

  const std::vector<float> jointVels = getJointVelocities();
  i = 0;
  for (const float& v : jointVels) {
    const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
    sceneArtObjInstanceAttr->addInitJointVelocityVal(key, v);
  }
  return sceneArtObjInstanceAttr;
}  // BulletArticulatedObject::getCurrentStateInstanceAttr

void BulletArticulatedObject::resetStateFromSceneInstanceAttr() {
  auto sceneObjInstanceAttr = getInitObjectInstanceAttr();
  if (!sceneObjInstanceAttr) {
    // if no scene instance attributes specified, no initial state is set
    return;
  }
  // Set whether dofs should be clamped to limits before phys step
  autoClampJointLimits_ = sceneObjInstanceAttr->getAutoClampJointLimits();

  // now move objects
  // set object's location and rotation based on translation and rotation
  // params specified in instance attributes
  auto translate = sceneObjInstanceAttr->getTranslation();

  // construct initial transformation state.
  Mn::Matrix4 state = Mn::Matrix4::from(
      sceneObjInstanceAttr->getRotation().toMatrix(), translate);
  setTransformation(state);
  // set object's motion type if different than set value
  const physics::MotionType attrObjMotionType =
      static_cast<physics::MotionType>(sceneObjInstanceAttr->getMotionType());
  if (attrObjMotionType != physics::MotionType::UNDEFINED) {
    setMotionType(attrObjMotionType);
  }
  // set initial joint positions
  // get array of existing joint dofs
  std::vector<float> aoJointPose = getJointPositions();
  // get instance-specified initial joint positions
  const auto& initJointPos = sceneObjInstanceAttr->getInitJointPose();
  // map instance vals into
  size_t idx = 0;
  for (const auto& elem : initJointPos) {
    if (idx >= aoJointPose.size()) {
      ESP_WARNING() << "Attempting to specify more initial joint poses than "
                       "exist in articulated object"
                    << sceneObjInstanceAttr->getHandle() << ", so skipping";
      break;
    }
    aoJointPose[idx++] = elem.second;
  }
  setJointPositions(aoJointPose);

  // set initial joint velocities
  // get array of existing joint vel dofs
  std::vector<float> aoJointVels = getJointVelocities();
  // get instance-specified initial joint velocities
  const std::map<std::string, float>& initJointVel =
      sceneObjInstanceAttr->getInitJointVelocities();
  idx = 0;
  for (const auto& elem : initJointVel) {
    if (idx >= aoJointVels.size()) {
      ESP_WARNING()
          << "Attempting to specify more initial joint velocities than "
             "exist in articulated object"
          << sceneObjInstanceAttr->getHandle() << ", so skipping";
      break;
    }
    aoJointVels[idx++] = elem.second;
  }

  setJointVelocities(aoJointVels);

}  // BulletArticulatedObject::resetStateFromSceneInstanceAttr

void BulletArticulatedObject::setRootState(const Mn::Matrix4& state) {
  btTransform tr{state};
  btMultiBody_->setBaseWorldTransform(tr);
  if (bFixedObjectRigidBody_) {
    bFixedObjectRigidBody_->setWorldTransform(tr);
  }
  // update the simulation state
  updateKinematicState();
}

Mn::Vector3 BulletArticulatedObject::getRootLinearVelocity() const {
  return Mn::Vector3(btMultiBody_->getBaseVel());
}

void BulletArticulatedObject::setRootLinearVelocity(const Mn::Vector3& linVel) {
  btMultiBody_->setBaseVel(btVector3(linVel));
}

Mn::Vector3 BulletArticulatedObject::getRootAngularVelocity() const {
  return Mn::Vector3(btMultiBody_->getBaseOmega());
}
void BulletArticulatedObject::setRootAngularVelocity(
    const Mn::Vector3& angVel) {
  btMultiBody_->setBaseOmega(btVector3(angVel));
}

void BulletArticulatedObject::setJointForces(const std::vector<float>& forces) {
  if (forces.size() != size_t(btMultiBody_->getNumDofs())) {
    ESP_DEBUG() << "Force vector size mis-match (input:" << forces.size()
                << ", expected:" << btMultiBody_->getNumDofs()
                << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btMultibodyLink& link = btMultiBody_->getLink(i);
    for (int dof = 0; dof < link.m_dofCount; ++dof) {
      link.m_jointTorque[dof] = forces[dofCount];
      ++dofCount;
    }
  }
}

void BulletArticulatedObject::addJointForces(const std::vector<float>& forces) {
  if (forces.size() != size_t(btMultiBody_->getNumDofs())) {
    ESP_DEBUG() << "Force vector size mis-match (input:" << forces.size()
                << ", expected:" << btMultiBody_->getNumDofs()
                << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btMultibodyLink& link = btMultiBody_->getLink(i);
    for (int dof = 0; dof < link.m_dofCount; ++dof) {
      link.m_jointTorque[dof] += forces[dofCount];
      ++dofCount;
    }
  }
}

std::vector<float> BulletArticulatedObject::getJointForces() {
  std::vector<float> forces(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* dofForces = btMultiBody_->getJointTorqueMultiDof(i);
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      forces[dofCount] = dofForces[dof];
      ++dofCount;
    }
  }
  return forces;
}

void BulletArticulatedObject::setJointVelocities(
    const std::vector<float>& vels) {
  if (vels.size() != size_t(btMultiBody_->getNumDofs())) {
    ESP_DEBUG() << "Velocity vector size mis-match (input:" << vels.size()
                << ", expected:" << btMultiBody_->getNumDofs()
                << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (btMultiBody_->getLink(i).m_dofCount > 0) {
      // this const_cast is only needed for Bullet 2.87. It is harmless in any
      // case.
      btMultiBody_->setJointVelMultiDof(i, const_cast<float*>(&vels[dofCount]));
      dofCount += btMultiBody_->getLink(i).m_dofCount;
    }
  }
}

std::vector<float> BulletArticulatedObject::getJointVelocities() {
  std::vector<float> vels(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* dofVels = btMultiBody_->getJointVelMultiDof(i);
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      vels[dofCount] = dofVels[dof];
      ++dofCount;
    }
  }
  return vels;
}

void BulletArticulatedObject::setJointPositions(
    const std::vector<float>& positions) {
  if (positions.size() != size_t(btMultiBody_->getNumPosVars())) {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Position vector size mis-match (input:" << positions.size()
        << ", expected:" << btMultiBody_->getNumPosVars() << "), aborting.";
  }

  int posCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    auto& link = btMultiBody_->getLink(i);
    if (link.m_posVarCount > 0) {
      btMultiBody_->setJointPosMultiDof(
          i, const_cast<float*>(&positions[posCount]));
      posCount += link.m_posVarCount;
    }
  }

  // update the simulation state
  updateKinematicState();
}

std::vector<float> BulletArticulatedObject::getJointPositions() {
  std::vector<float> positions(btMultiBody_->getNumPosVars());
  int posCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* linkPos = btMultiBody_->getJointPosMultiDof(i);
    for (int pos = 0; pos < btMultiBody_->getLink(i).m_posVarCount; ++pos) {
      positions[posCount] = linkPos[pos];
      ++posCount;
    }
  }
  return positions;
}

std::vector<float> BulletArticulatedObject::getJointMotorTorques(
    double fixedTimeStep) {
  std::vector<float> torques(btMultiBody_->getNumDofs());
  for (auto& motor : jointMotors_) {
    auto& settings = motor.second->settings;
    if (settings.motorType == JointMotorType::SingleDof) {
      auto& btMotor = articulatedJointMotors.at(motor.first);
      btScalar impulse = btMotor->getAppliedImpulse(0);
      float force = impulse / float(fixedTimeStep);
      int link_dof = btMultiBody_->getLink(motor.second->index).m_dofOffset;
      torques[link_dof] += force;
    } else {
      ESP_CHECK(
          false,
          "getJointMotorTorques is not yet implemented for spherical joints.");
    }
  }

  return torques;
}

std::pair<std::vector<float>, std::vector<float>>
BulletArticulatedObject::getJointPositionLimits() {
  std::vector<float> lowerLimits(btMultiBody_->getNumPosVars(), -INFINITY);
  std::vector<float> upperLimits(btMultiBody_->getNumPosVars(), INFINITY);

  int posCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    auto jntLimitCnstrntIter = jointLimitConstraints.find(i);
    if (jntLimitCnstrntIter != jointLimitConstraints.end()) {
      // a joint limit constraint exists for this link's parent joint
      lowerLimits[posCount] = jntLimitCnstrntIter->second.lowerLimit;
      upperLimits[posCount] = jntLimitCnstrntIter->second.upperLimit;
      ++posCount;
    } else {
      posCount += btMultiBody_->getLink(i).m_posVarCount;
    }
  }
  CORRADE_INTERNAL_ASSERT(posCount == btMultiBody_->getNumPosVars());
  return std::make_pair(lowerLimits, upperLimits);
}

void BulletArticulatedObject::addArticulatedLinkForce(int linkId,
                                                      Mn::Vector3 force) {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId);
  btMultiBody_->addLinkForce(linkId, btVector3{force});
}

float BulletArticulatedObject::getArticulatedLinkFriction(int linkId) {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId);
  return btMultiBody_->getLinkCollider(linkId)->getFriction();
}

void BulletArticulatedObject::setArticulatedLinkFriction(int linkId,
                                                         float friction) {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId);
  btMultiBody_->getLinkCollider(linkId)->setFriction(friction);
}

JointType BulletArticulatedObject::getLinkJointType(int linkId) const {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId && linkId >= 0);
  return JointType(int(btMultiBody_->getLink(linkId).m_jointType));
}

int BulletArticulatedObject::getLinkDoFOffset(int linkId) const {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId && linkId >= 0);
  return btMultiBody_->getLink(linkId).m_dofOffset;
}

int BulletArticulatedObject::getLinkNumDoFs(int linkId) const {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId && linkId >= 0);
  return btMultiBody_->getLink(linkId).m_dofCount;
}

int BulletArticulatedObject::getLinkJointPosOffset(int linkId) const {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId && linkId >= 0);
  return btMultiBody_->getLink(linkId).m_cfgOffset;
}

int BulletArticulatedObject::getLinkNumJointPos(int linkId) const {
  CORRADE_INTERNAL_ASSERT(getNumLinks() > linkId && linkId >= 0);
  return btMultiBody_->getLink(linkId).m_posVarCount;
}

void BulletArticulatedObject::reset() {
  // reset positions and velocities to zero
  // clears forces/torques
  // Note: does not update root state
  std::vector<float> zeros(btMultiBody_->getNumPosVars(), 0.0f);
  // handle spherical quaternions
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    auto& link = btMultiBody_->getLink(i);
    if (link.m_jointType ==
        btMultibodyLink::eFeatherstoneJointType::eSpherical) {
      // need a valid identity quaternion [0,0,0,1]
      zeros[link.m_cfgOffset + 3] = 1;
    }
  }

  // also updates kinematic state
  setJointPositions(zeros);

  btMultiBody_->clearConstraintForces();
  btMultiBody_->clearVelocities();
  btMultiBody_->clearForcesAndTorques();
}

void BulletArticulatedObject::setActive(bool active) {
  if (!active) {
    btMultiBody_->goToSleep();
  } else {
    btMultiBody_->wakeUp();
  }
}

bool BulletArticulatedObject::isActive() const {
  return btMultiBody_->isAwake();
}

bool BulletArticulatedObject::getCanSleep() {
  return btMultiBody_->getCanSleep();
}

void BulletArticulatedObject::setMotionType(MotionType mt) {
  if (mt == objectMotionType_) {
    return;
  }
  if (mt == MotionType::UNDEFINED) {
    return;
  }

  // only need to change the state if the previous state was different (i.e.,
  // DYNAMIC -> other)
  if (mt == MotionType::DYNAMIC) {
    bWorld_->addMultiBody(btMultiBody_.get());
  } else if (objectMotionType_ == MotionType::DYNAMIC) {
    // TODO: STATIC and KINEMATIC are equivalent for simplicity. Could manually
    // limit STATIC...
    bWorld_->removeMultiBody(btMultiBody_.get());
  }
  objectMotionType_ = mt;
}

void BulletArticulatedObject::clampJointLimits() {
  auto pose = getJointPositions();
  bool poseModified = false;

  // some small contrived error term for overflow
  float corrective_eps = 0.000001;

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    auto jntLimitCnstrntIter = jointLimitConstraints.find(i);
    if (jntLimitCnstrntIter != jointLimitConstraints.end()) {
      // a joint limit constraint exists for this link
      // position clamping:
      if (pose[dofCount] <
          jntLimitCnstrntIter->second.lowerLimit - corrective_eps) {
        poseModified = true;
        pose[dofCount] = jntLimitCnstrntIter->second.lowerLimit;
      } else if (pose[dofCount] >
                 jntLimitCnstrntIter->second.upperLimit + corrective_eps) {
        poseModified = true;
        pose[dofCount] = jntLimitCnstrntIter->second.upperLimit;
      }
    }

    // continue incrementing the dof counter
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      ++dofCount;
    }
  }

  if (poseModified) {
    setJointPositions(pose);
  }
}

void BulletArticulatedObject::overrideCollisionGroup(CollisionGroup group) {
  // for collision object in model:
  int collisionFilterGroup = int(group);
  int collisionFilterMask = uint32_t(CollisionGroupHelper::getMaskForGroup(
      CollisionGroup(collisionFilterGroup)));

  if (bFixedObjectRigidBody_ != nullptr) {
    // A fixed base shape exists. Overriding with a uniform group could be a
    // perf issue or cause unexpected contacts
    ESP_WARNING() << "Overriding all link collision groups for an "
                     "ArticulatedObject with a STATIC base collision shape "
                     "defined. Only do this if you understand the risks.";
    bWorld_->removeRigidBody(bFixedObjectRigidBody_.get());
    bWorld_->addRigidBody(bFixedObjectRigidBody_.get(), collisionFilterGroup,
                          collisionFilterMask);
  }

  // override separate base link object group
  auto* baseCollider = btMultiBody_->getBaseCollider();
  bWorld_->removeCollisionObject(baseCollider);
  bWorld_->addCollisionObject(baseCollider, collisionFilterGroup,
                              collisionFilterMask);

  // override link collision object groups
  for (int colIx = 0; colIx < btMultiBody_->getNumLinks(); ++colIx) {
    auto* linkCollider = btMultiBody_->getLinkCollider(colIx);
    bWorld_->removeCollisionObject(linkCollider);
    bWorld_->addCollisionObject(linkCollider, collisionFilterGroup,
                                collisionFilterMask);
  }
}

void BulletArticulatedObject::updateKinematicState() {
  btMultiBody_->forwardKinematics(scratch_q_, scratch_m_);
  btMultiBody_->updateCollisionObjectWorldTransforms(scratch_q_, scratch_m_);
  // Need to update the aabbs manually also for broadphase collision detection
  for (int linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
    bWorld_->updateSingleAabb(btMultiBody_->getLinkCollider(linkIx));
  }
  bWorld_->updateSingleAabb(btMultiBody_->getBaseCollider());
  if (bFixedObjectRigidBody_) {
    bWorld_->updateSingleAabb(bFixedObjectRigidBody_.get());
  }
  // update visual shapes
  if (!isDeferringUpdate_) {
    updateNodes(true);
  }
}

/**
 * @brief Specific callback function for ArticulatedObject::contactTest to
 * screen self-collisions.
 */
struct AOSimulationContactResultCallback
    : public SimulationContactResultCallback {
  btMultiBody* mb_ = nullptr;
  btRigidBody* fixedBaseColObj_ = nullptr;

  /**
   * @brief Constructor taking the AO's btMultiBody as input to screen
   * self-collisions.
   */
  AOSimulationContactResultCallback(btMultiBody* mb,
                                    btRigidBody* fixedBaseColObj)
      : mb_(mb), fixedBaseColObj_(fixedBaseColObj) {
    bCollision = false;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const override {
    // base method checks for group|mask filter
    bool collides = SimulationContactResultCallback::needsCollision(proxy0);
    // check for self-collision
    if (!mb_->hasSelfCollision()) {
      // This should always be a valid conversion to btCollisionObject
      auto* co = static_cast<btCollisionObject*>(proxy0->m_clientObject);
      auto* mblc = dynamic_cast<btMultiBodyLinkCollider*>(co);
      if (mblc) {
        if (mblc->m_multiBody == mb_) {
          // screen self-collisions
          collides = false;
        }
      } else if (co == fixedBaseColObj_) {
        // screen self-collisions w/ fixed base rigid
        collides = false;
      }
    }
    return collides;
  }
};

bool BulletArticulatedObject::contactTest() {
  AOSimulationContactResultCallback src(btMultiBody_.get(),
                                        bFixedObjectRigidBody_.get());

  auto* baseCollider = btMultiBody_->getBaseCollider();
  // Do a contact test for each piece of the AO and return at soonest contact.
  // Should be cheaper to hit multiple local aabbs than to check the full scene.
  if (bFixedObjectRigidBody_) {
    src.m_collisionFilterGroup =
        bFixedObjectRigidBody_->getBroadphaseHandle()->m_collisionFilterGroup;
    src.m_collisionFilterMask =
        bFixedObjectRigidBody_->getBroadphaseHandle()->m_collisionFilterMask;

    bWorld_->getCollisionWorld()->contactTest(bFixedObjectRigidBody_.get(),
                                              src);
    if (src.bCollision) {
      return src.bCollision;
    }
  } else if (baseCollider) {
    src.m_collisionFilterGroup =
        baseCollider->getBroadphaseHandle()->m_collisionFilterGroup;
    src.m_collisionFilterMask =
        baseCollider->getBroadphaseHandle()->m_collisionFilterMask;
    bWorld_->getCollisionWorld()->contactTest(baseCollider, src);
    if (src.bCollision) {
      return src.bCollision;
    }
  }
  for (int colIx = 0; colIx < btMultiBody_->getNumLinks(); ++colIx) {
    auto* linkCollider = btMultiBody_->getLinkCollider(colIx);
    src.m_collisionFilterGroup =
        linkCollider->getBroadphaseHandle()->m_collisionFilterGroup;
    src.m_collisionFilterMask =
        linkCollider->getBroadphaseHandle()->m_collisionFilterMask;
    bWorld_->getCollisionWorld()->contactTest(linkCollider, src);
    if (src.bCollision) {
      return src.bCollision;
    }
  }
  return false;
}  // contactTest

// ------------------------
// Joint Motor API
// ------------------------

bool BulletArticulatedObject::supportsSingleDofJointMotor(int linkIx) const {
  bool canHaveMotor = (btMultiBody_->getLink(linkIx).m_jointType ==
                           btMultibodyLink::eRevolute ||
                       btMultiBody_->getLink(linkIx).m_jointType ==
                           btMultibodyLink::ePrismatic);
  return canHaveMotor;
}

std::unordered_map<int, int> BulletArticulatedObject::createMotorsForAllDofs(
    const JointMotorSettings& settings) {
  std::unordered_map<int, int> motorIdsToLinks;
  // Reserve space. We have an upperbound on size.
  motorIdsToLinks.reserve(btMultiBody_->getNumLinks());
  auto settingsCopy = settings;
  for (int linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
    if (supportsSingleDofJointMotor(linkIx)) {
      settingsCopy.motorType = JointMotorType::SingleDof;
    } else if (btMultiBody_->getLink(linkIx).m_jointType ==
               btMultibodyLink::eSpherical) {
      settingsCopy.motorType = JointMotorType::Spherical;
    } else {
      // skip the creation phase for unsupported joints
      continue;
    }
    int motorId = createJointMotor(linkIx, settingsCopy);
    motorIdsToLinks[motorId] = linkIx;
  }
  ESP_DEBUG() << "BulletArticulatedObject::createMotorsForAllDofs():"
              << motorIdsToLinks;
  return motorIdsToLinks;
}

int BulletArticulatedObject::createJointMotor(
    const int linkIndex,
    const JointMotorSettings& settings) {
  // check for valid configuration
  ESP_CHECK(
      links_.count(linkIndex) != 0,
      "BulletArticulatedObject::createJointMotor - no link with linkIndex ="
          << linkIndex);
  if (settings.motorType == JointMotorType::SingleDof) {
    ESP_CHECK(supportsSingleDofJointMotor(linkIndex),
              "BulletArticulatedObject::createJointMotor - "
              "JointMotorSettings.motorType==SingleDof incompatible with joint"
                  << linkIndex);
  } else {
    // JointMotorType::Spherical
    ESP_CHECK(getLinkJointType(linkIndex) == JointType::Spherical,
              "BulletArticulatedObject::createJointMotor - "
              "JointMotorSettings.motorType==Spherical incompatible with joint"
                  << linkIndex);
  }

  auto motor = JointMotor::create_unique();
  motor->settings = settings;
  motor->index = linkIndex;
  motor->motorId = nextJointMotorId_;
  jointMotors_.emplace(nextJointMotorId_,
                       std::move(motor));  // cache the Habitat structure

  if (settings.motorType == JointMotorType::SingleDof) {
    auto btMotor = std::make_unique<btMultiBodyJointMotor>(
        btMultiBody_.get(), linkIndex, settings.velocityTarget,
        settings.maxImpulse);
    btMotor->setPositionTarget(settings.positionTarget, settings.positionGain);
    btMotor->setVelocityTarget(settings.velocityTarget, settings.velocityGain);
    bWorld_->addMultiBodyConstraint(btMotor.get());
    btMotor->finalizeMultiDof();
    articulatedJointMotors.emplace(
        nextJointMotorId_, std::move(btMotor));  // cache the Bullet structure
  } else {
    // JointMotorType::Spherical
    auto btMotor = std::make_unique<btMultiBodySphericalJointMotor>(
        btMultiBody_.get(), linkIndex, settings.maxImpulse);
    btMotor->setPositionTarget(btQuaternion(settings.sphericalPositionTarget),
                               settings.positionGain);
    btMotor->setVelocityTarget(btVector3(settings.sphericalVelocityTarget),
                               settings.velocityGain);
    bWorld_->addMultiBodyConstraint(btMotor.get());
    btMotor->finalizeMultiDof();
    articulatedSphericalJointMotors.emplace(
        nextJointMotorId_, std::move(btMotor));  // cache the Bullet structure
  }
  // force activation if motors are updated
  setActive(true);
  return nextJointMotorId_++;
}  // BulletArticulatedObject::createJointMotor

void BulletArticulatedObject::removeJointMotor(const int motorId) {
  auto jointMotorIter = jointMotors_.find(motorId);
  ESP_CHECK(jointMotorIter != jointMotors_.end(),
            "BulletArticulatedObject::removeJointMotor - No motor exists with "
            "motorId ="
                << motorId);
  auto aoJointMoterIter = articulatedJointMotors.find(motorId);
  if (aoJointMoterIter != articulatedJointMotors.end()) {
    bWorld_->removeMultiBodyConstraint(aoJointMoterIter->second.get());
    articulatedJointMotors.erase(aoJointMoterIter);
  } else {
    auto aoSphrJointMoterIter = articulatedSphericalJointMotors.find(motorId);
    if (aoSphrJointMoterIter != articulatedSphericalJointMotors.end()) {
      bWorld_->removeMultiBodyConstraint(aoSphrJointMoterIter->second.get());
      articulatedSphericalJointMotors.erase(aoSphrJointMoterIter);
    } else {
      ESP_ERROR() << "Cannot remove JointMotor: invalid ID (" << motorId
                  << ").";
      return;
    }
  }
  jointMotors_.erase(jointMotorIter);
  // force activation if motors are updated
  btMultiBody_->wakeUp();
}

void BulletArticulatedObject::updateJointMotor(
    const int motorId,
    const JointMotorSettings& settings) {
  auto jointMotorIter = jointMotors_.find(motorId);
  ESP_CHECK(jointMotorIter != jointMotors_.end(),
            "BulletArticulatedObject::updateJointMotor - No motor exists with "
            "motorId ="
                << motorId);
  ESP_CHECK(jointMotorIter->second->settings.motorType == settings.motorType,
            "BulletArticulatedObject::updateJointMotor - "
            "JointMotorSettings.motorType does not match joint type.");
  jointMotorIter->second->settings = settings;

  auto aoJointMoterIter = articulatedJointMotors.find(motorId);
  if (aoJointMoterIter != articulatedJointMotors.end()) {
    aoJointMoterIter->second->setPositionTarget(settings.positionTarget,
                                                settings.positionGain);
    aoJointMoterIter->second->setVelocityTarget(settings.velocityTarget,
                                                settings.velocityGain);
    aoJointMoterIter->second->setMaxAppliedImpulse(settings.maxImpulse);
  } else {
    auto aoSphrJointMoterIter = articulatedSphericalJointMotors.find(motorId);
    if (aoSphrJointMoterIter != articulatedSphericalJointMotors.end()) {
      aoSphrJointMoterIter->second->setPositionTarget(
          btQuaternion(settings.sphericalPositionTarget),
          settings.positionGain);
      aoSphrJointMoterIter->second->setVelocityTarget(
          btVector3(settings.sphericalVelocityTarget), settings.velocityGain);
      aoSphrJointMoterIter->second->setMaxAppliedImpulse(settings.maxImpulse);
    } else {
      ESP_ERROR() << "Cannot update JointMotor. Invalid ID (" << motorId
                  << ").";
      return;
    }
  }
  // force activation if motors are updated
  setActive(true);
}

void BulletArticulatedObject::updateAllMotorTargets(
    const std::vector<float>& stateTargets,
    bool velocities) {
  ESP_CHECK(stateTargets.size() == velocities ? btMultiBody_->getNumDofs()
                                              : btMultiBody_->getNumPosVars(),
            "BulletArticulatedObject::updateAllMotorTargets - stateTargets "
            "size does not match object state size.");

  for (auto& motor : jointMotors_) {
    btMultibodyLink& btLink = btMultiBody_->getLink(motor.second->index);
    int startIndex = velocities ? btLink.m_dofOffset : btLink.m_cfgOffset;
    auto& settings = motor.second->settings;
    if (settings.motorType == JointMotorType::SingleDof) {
      auto& btMotor = articulatedJointMotors.at(motor.first);
      if (velocities) {
        settings.velocityTarget = double(stateTargets[startIndex]);
        btMotor->setVelocityTarget(settings.velocityTarget,
                                   settings.velocityGain);
      } else {
        // positions
        settings.positionTarget = double(stateTargets[startIndex]);
        btMotor->setPositionTarget(settings.positionTarget,
                                   settings.positionGain);
      }
    } else {
      // JointMotorType::Spherical
      if (velocities) {
        settings.sphericalVelocityTarget = {stateTargets[startIndex],
                                            stateTargets[startIndex + 1],
                                            stateTargets[startIndex + 2]};
        articulatedSphericalJointMotors.at(motor.first)
            ->setVelocityTarget(btVector3(settings.sphericalVelocityTarget),
                                settings.velocityGain);
      } else {
        // positions
        settings.sphericalPositionTarget =
            Mn::Quaternion(Mn::Vector3(stateTargets[startIndex],
                                       stateTargets[startIndex + 1],
                                       stateTargets[startIndex + 2]),
                           stateTargets[startIndex + 3])
                .normalized();
        articulatedSphericalJointMotors.at(motor.first)
            ->setPositionTarget(btQuaternion(settings.sphericalPositionTarget),
                                settings.positionGain);
      }
    }
  }
  // force activation when motors are updated
  setActive(true);
}

}  // namespace physics
}  // namespace esp
