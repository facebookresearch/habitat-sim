// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// Construction code adapted from Bullet3/examples/

#include "BulletArticulatedObject.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
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
  Magnum::Matrix4 converted{trans};
  node->setRotation(Magnum::Quaternion::fromMatrix(converted.rotation()));
  node->setTranslation(converted.translation());
}

///////////////////////////////////
// Class functions
///////////////////////////////////

BulletArticulatedObject::~BulletArticulatedObject() {
  // Corrade::Utility::Debug() << "deconstructing ~BulletArticulatedObject";
  if (motionType_ != MotionType::KINEMATIC) {
    // KINEMATIC objects have already been removed from the world.
    bWorld_->removeMultiBody(btMultiBody_.get());
  }
  // remove link collision objects from world
  for (int colIx = 0; colIx < btMultiBody_->getNumLinks(); ++colIx) {
    auto linkCollider = btMultiBody_->getLinkCollider(colIx);
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
  auto baseCollider = btMultiBody_->getBaseCollider();
  bWorld_->btCollisionWorld::removeCollisionObject(baseCollider);
  collisionObjToObjIds_->erase(baseCollider);
  delete baseCollider;

  // delete children of compound collisionShapes
  std::map<int, std::unique_ptr<btCollisionShape>>::iterator csIter;
  for (csIter = linkCollisionShapes_.begin();
       csIter != linkCollisionShapes_.end(); csIter++) {
    auto compoundShape = dynamic_cast<btCompoundShape*>(csIter->second.get());
    if (compoundShape) {
      for (int i = 0; i < compoundShape->getNumChildShapes(); ++i) {
        auto childShape = compoundShape->getChildShape(i);
        compoundShape->removeChildShape(childShape);
        delete childShape;
      }
    }
  }
  // remove motors from the world
  std::map<int, std::unique_ptr<btMultiBodyJointMotor>>::iterator jmIter;
  for (jmIter = articulatedJointMotors.begin();
       jmIter != articulatedJointMotors.end(); jmIter++) {
    bWorld_->removeMultiBodyConstraint(jmIter->second.get());
  }
  std::map<int, JointLimitConstraintInfo>::iterator jlIter;
  for (jlIter = jointLimitConstraints.begin();
       jlIter != jointLimitConstraints.end(); jlIter++) {
    bWorld_->removeMultiBodyConstraint(jlIter->second.con);
    delete jlIter->second.con;
  }
}

bool BulletArticulatedObject::initializeFromURDF(
    URDFImporter& urdfImporter,
    const Magnum::Matrix4& worldTransform,
    gfx::DrawableGroup* drawables,
    scene::SceneNode* physicsNode,
    bool fixedBase) {
  // TODO: should this be included as optional parameter?
  // btTransform rootTransformInWorldSpace = btTransform::getIdentity();
  Magnum::Matrix4 rootTransformInWorldSpace{worldTransform};
  // rootTransformInWorldSpace.setOrigin(btVector3{0,10.0,0});

  BulletURDFImporter& u2b = *(static_cast<BulletURDFImporter*>(&urdfImporter));
  u2b.setFixedBase(fixedBase);

  auto urdfModel = u2b.getModel();

  // TODO: are these needed? Not used in examples.
  int flags = 0;

  URDF2BulletCached cache;
  u2b.InitURDF2BulletCache(cache, flags);

  int urdfLinkIndex = u2b.getRootLinkIndex();
  // int rootIndex = u2b.getRootLinkIndex();

  // NOTE: recursive path only
  u2b.ConvertURDF2BulletInternal(cache, urdfLinkIndex,
                                 rootTransformInWorldSpace, bWorld_.get(),
                                 flags, linkCollisionShapes_);

  if (cache.m_bulletMultiBody) {
    btMultiBody* mb = cache.m_bulletMultiBody;
    jointLimitConstraints = cache.m_jointLimitConstraints;

    mb->setHasSelfCollision((flags & CUF_USE_SELF_COLLISION) !=
                            0);  // NOTE: default no

    mb->finalizeMultiDof();

    btTransform localInertialFrameRoot =
        cache.m_urdfLinkLocalInertialFrames[urdfLinkIndex];

    //?
    if (flags & CUF_USE_MJCF) {
    } else {
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

    bFixedObjectShape_ = std::make_unique<btCompoundShape>();

    // By convention, fixed links in the URDF are assigned Noncollidable, and we
    // then insert corresponding fixed rigid bodies with group Static.
    // Collisions with a fixed rigid body are cheaper than collisions with a
    // fixed link, due to problems with multibody sleeping behavior.
    {
      auto* col = mb->getBaseCollider();
      if (col->getBroadphaseHandle()->m_collisionFilterGroup ==
          int(CollisionGroup::Noncollidable)) {
        // The collider child is an aligned compound or single shape
        btTransform identity;
        identity.setIdentity();
        bFixedObjectShape_.get()->addChildShape(identity,
                                                col->getCollisionShape());
      }

      for (int m = 0; m < mb->getNumLinks(); m++) {
        btMultiBodyLinkCollider* col = mb->getLink(m).m_collider;
        if (col) {
          if (col->getBroadphaseHandle()->m_collisionFilterGroup ==
              int(CollisionGroup::Noncollidable)) {
            bFixedObjectShape_.get()->addChildShape(col->getWorldTransform(),
                                                    col->getCollisionShape());
          }
        }
      }

      if (bFixedObjectShape_.get()->getNumChildShapes()) {
        btRigidBody::btRigidBodyConstructionInfo info =
            btRigidBody::btRigidBodyConstructionInfo(0.f, nullptr,
                                                     bFixedObjectShape_.get());
        bFixedObjectRigidBody_ = std::make_unique<btRigidBody>(info);
        BulletDebugManager::get().mapCollisionObjectTo(
            bFixedObjectRigidBody_.get(),
            "URDF, " + u2b.getModel()->m_name + ", fixed body");
        bWorld_->addRigidBody(
            bFixedObjectRigidBody_.get(), int(CollisionGroup::Static),
            CollisionGroupHelper::getMaskForGroup(CollisionGroup::Static));
        collisionObjToObjIds_->emplace(bFixedObjectRigidBody_.get(), objectId_);
      } else {
        bFixedObjectShape_ = nullptr;
      }
    }

    // Attach SceneNode visual components
    int urdfLinkIx = 0;
    for (auto& link : urdfModel->m_links) {
      int bulletLinkIx = cache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIx];
      /*
      Corrade::Utility::Debug()
          << "urdfLinkIx = " << urdfLinkIx
          << ", m_name = " << link.second->m_name
          << ", m_linkIndex = " << link.second->m_linkIndex
          << ", bulletLinkIx = " << bulletLinkIx;
       */
      ArticulatedLink* linkObject = nullptr;
      if (bulletLinkIx >= 0) {
        links_[bulletLinkIx] = std::make_unique<BulletArticulatedLink>(
            &physicsNode->createChild(), resMgr_, bWorld_, bulletLinkIx,
            collisionObjToObjIds_);
        linkObject = links_[bulletLinkIx].get();
      } else {
        if (!baseLink_) {
          baseLink_ = std::make_unique<BulletArticulatedLink>(
              &node().createChild(), resMgr_, bWorld_, bulletLinkIx,
              collisionObjToObjIds_);
        }
        linkObject = baseLink_.get();
      }

      linkObject->node().setType(esp::scene::SceneNodeType::OBJECT);
      bool success =
          attachGeometry(linkObject, link.second,
                         urdfImporter.getModel()->m_materials, drawables);
      // Corrade::Utility::Debug() << "geomSuccess: " << success;

      urdfLinkIx++;
    }

    // top level only valid in initial state, but computes valid sub-part AABBs.
    node().computeCumulativeBB();

    // Build damping motors
    int dofCount = 0;
    for (int linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
      if (supportsJointMotor(linkIx)) {
        btMultibodyLink& link = btMultiBody_->getLink(linkIx);
        for (int dof = 0; dof < link.m_dofCount; ++dof) {
          JointMotorSettings settings;
          settings.maxImpulse = link.m_jointDamping;
          createJointMotor(linkIx, dof, dofCount, settings);
          dofCount++;
        }
      } else {
        dofCount += btMultiBody_->getLink(linkIx).m_dofCount;
      }
    }
  }
  return true;
}

Magnum::Matrix4 BulletArticulatedObject::getRootState() {
  return Magnum::Matrix4{btMultiBody_->getBaseWorldTransform()};
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

bool BulletArticulatedObject::attachGeometry(
    ArticulatedLink* linkObject,
    const std::shared_ptr<io::URDF::Link>& link,
    const std::map<std::string, std::shared_ptr<io::URDF::Material>>& materials,
    gfx::DrawableGroup* drawables) {
  bool geomSuccess = false;

  for (auto& visual : link->m_visualArray) {
    // create a new child for each visual component
    scene::SceneNode& visualGeomComponent = linkObject->node().createChild();
    // cache the visual node
    linkObject->visualNodes_.push_back(&visualGeomComponent);
    visualGeomComponent.setType(esp::scene::SceneNodeType::OBJECT);
    visualGeomComponent.setTransformation(
        link->m_inertia.m_linkLocalFrame.invertedRigid() *
        visual.m_linkLocalFrame);

    switch (visual.m_geometry.m_type) {
      case io::URDF::GEOM_CAPSULE:
        Corrade::Utility::Debug()
            << "Trying to add visual capsule, not implemented";
        // TODO:
        break;
      case io::URDF::GEOM_CYLINDER:
        Corrade::Utility::Debug()
            << "Trying to add visual cylinder, not implemented";
        // TODO:
        break;
      case io::URDF::GEOM_BOX:
        Corrade::Utility::Debug()
            << "Trying to add visual box, not implemented";
        // TODO:
        break;
      case io::URDF::GEOM_SPHERE:
        Corrade::Utility::Debug()
            << "Trying to add visual sphere, not implemented";
        // TODO:
        break;
      case io::URDF::GEOM_MESH: {
        // Corrade::Utility::Debug() << "visual.m_geometry.m_meshFileName = "
        //                          << visual.m_geometry.m_meshFileName;
        // visual.m_sourceFileLocation
        visualGeomComponent.scale(visual.m_geometry.m_meshScale);

        // first try to import the asset
        bool meshSuccess =
            resMgr_.importAsset(visual.m_geometry.m_meshFileName);
        if (!meshSuccess) {
          Cr::Utility::Debug() << "Failed to import the render asset: "
                               << visual.m_geometry.m_meshFileName;
          return false;
        }

        // create a modified asset if necessary
        std::shared_ptr<io::URDF::Material> material =
            visual.m_geometry.m_localMaterial;
        std::string assetMatModName = resMgr_.setupMaterialModifiedAsset(
            visual.m_geometry.m_meshFileName, material);

        // then attach
        geomSuccess = resMgr_.attachAsset(
            (assetMatModName.empty()
                 ? visual.m_geometry.m_meshFileName
                 : assetMatModName),  // use either a material modified or
                                      // original asset
            visualGeomComponent, linkObject->visualNodes_, drawables);

        // cache the visual component for later query
        if (geomSuccess) {
          linkObject->visualAttachments_.push_back(
              {&visualGeomComponent, visual.m_geometry.m_meshFileName});
        }
      } break;
      case io::URDF::GEOM_PLANE:
        Corrade::Utility::Debug()
            << "Trying to add visual plane, not implemented";
        // TODO:
        break;
      default:
        Corrade::Utility::Debug() << "BulletArticulatedObject::attachGeometry "
                                     ": Unsupported visual type.";
        break;
    }
  }

  return geomSuccess;
}

void BulletArticulatedObject::setRootState(const Magnum::Matrix4& state) {
  btTransform tr{state};
  btMultiBody_->setBaseWorldTransform(tr);
  if (bFixedObjectRigidBody_) {
    bFixedObjectRigidBody_.get()->setWorldTransform(tr);
  }
  // update the simulation state
  updateKinematicState();
}

void BulletArticulatedObject::setForces(const std::vector<float>& forces) {
  if (forces.size() != size_t(btMultiBody_->getNumDofs())) {
    Corrade::Utility::Debug()
        << "setForces - Force vector size mis-match (input: " << forces.size()
        << ", expected: " << btMultiBody_->getNumDofs() << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      btMultiBody_->addJointTorqueMultiDof(i, dof, forces[dofCount]);
      // Corrade::Utility::Debug() << "  " << forces[dofCount];
      dofCount++;
    }
  }
}

std::vector<float> BulletArticulatedObject::getForces() {
  std::vector<float> forces(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* dofForces = btMultiBody_->getJointTorqueMultiDof(i);
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      forces[dofCount] = dofForces[dof];
      dofCount++;
    }
  }
  return forces;
}

void BulletArticulatedObject::setVelocities(const std::vector<float>& vels) {
  if (vels.size() != size_t(btMultiBody_->getNumDofs())) {
    Corrade::Utility::Debug()
        << "setVelocities - Velocity vector size mis-match (input: "
        << vels.size() << ", expected: " << btMultiBody_->getNumDofs()
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

std::vector<float> BulletArticulatedObject::getVelocities() {
  std::vector<float> vels(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* dofVels = btMultiBody_->getJointVelMultiDof(i);
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      vels[dofCount] = dofVels[dof];
      dofCount++;
    }
  }
  return vels;
}

void BulletArticulatedObject::setPositions(
    const std::vector<float>& positions) {
  if (positions.size() != size_t(btMultiBody_->getNumDofs())) {
    Corrade::Utility::Debug()
        << "setPositions - Position vector size mis-match (input: "
        << positions.size() << ", expected: " << btMultiBody_->getNumDofs()
        << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (btMultiBody_->getLink(i).m_dofCount > 0) {
      btMultiBody_->setJointPosMultiDof(
          i, const_cast<float*>(&positions[dofCount]));
      dofCount += btMultiBody_->getLink(i).m_dofCount;
    }
  }

  // update the simulation state
  updateKinematicState();
}

std::vector<float> BulletArticulatedObject::getPositions() {
  std::vector<float> pos(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    btScalar* dofPos = btMultiBody_->getJointPosMultiDof(i);
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      pos[dofCount] = dofPos[dof];
      dofCount++;
    }
  }
  return pos;
}

std::vector<float> BulletArticulatedObject::getPositionLimits(
    bool upperLimits) {
  std::vector<float> dofLimits(btMultiBody_->getNumDofs());
  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (jointLimitConstraints.count(i) > 0) {
      // a joint limit constraint exists for this link's parent joint
      auto& jlc = jointLimitConstraints.at(i);
      dofLimits[dofCount] = upperLimits ? jlc.upperLimit : jlc.lowerLimit;
      dofCount++;
    } else {
      // iterate through joint dofs. Note: multi-dof joints cannot be limited,
      // so ok to skip.
      for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
        dofLimits[dofCount] = upperLimits ? INFINITY : -INFINITY;
        dofCount++;
      }
    }
  }
  CHECK(dofCount == btMultiBody_->getNumDofs());
  return dofLimits;
}

void BulletArticulatedObject::addArticulatedLinkForce(int linkId,
                                                      Magnum::Vector3 force) {
  CHECK(getNumLinks() > linkId);
  btMultiBody_->addLinkForce(linkId, btVector3{force});
}

float BulletArticulatedObject::getArticulatedLinkFriction(int linkId) {
  CHECK(getNumLinks() > linkId);
  return btMultiBody_->getLinkCollider(linkId)->getFriction();
}

void BulletArticulatedObject::setArticulatedLinkFriction(int linkId,
                                                         float friction) {
  CHECK(getNumLinks() > linkId);
  btMultiBody_->getLinkCollider(linkId)->setFriction(friction);
}

void BulletArticulatedObject::reset() {
  // reset positions and velocities to zero
  // clears forces/torques
  // Note: does not update root state TODO:?
  std::vector<float> zeros(btMultiBody_->getNumDofs(), 0);

  // also updates kinematic state
  setPositions(zeros);

  btMultiBody_->clearConstraintForces();
  btMultiBody_->clearVelocities();
  btMultiBody_->clearForcesAndTorques();
}

void BulletArticulatedObject::setSleep(bool sleep) {
  if (sleep) {
    btMultiBody_->goToSleep();
  } else {
    btMultiBody_->wakeUp();
  }
}

bool BulletArticulatedObject::getSleep() {
  return !btMultiBody_->isAwake();
}

bool BulletArticulatedObject::getCanSleep() {
  return btMultiBody_->getCanSleep();
}

void BulletArticulatedObject::setMotionType(MotionType mt) {
  if (mt == motionType_) {
    return;
  }
  if (mt == MotionType::UNDEFINED) {
    return;
  }

  // only need to change the state if the previous state was different (i.e.,
  // DYNAMIC -> other)
  if (mt == MotionType::DYNAMIC) {
    bWorld_->addMultiBody(btMultiBody_.get());
  } else if (motionType_ == MotionType::DYNAMIC) {
    // TODO: STATIC and KINEMATIC are equivalent for simplicity. Could manually
    // limit STATIC...
    bWorld_->removeMultiBody(btMultiBody_.get());
  }
  motionType_ = mt;
}

bool BulletArticulatedObject::supportsJointMotor(int linkIx) {
  bool canHaveMotor = (btMultiBody_->getLink(linkIx).m_jointType ==
                           btMultibodyLink::eRevolute ||
                       btMultiBody_->getLink(linkIx).m_jointType ==
                           btMultibodyLink::ePrismatic);
  return canHaveMotor;
}

std::map<int, int> BulletArticulatedObject::createMotorsForAllDofs(
    JointMotorSettings settings) {
  std::map<int, int> dofsToMotorIds;
  int dofCount = 0;
  for (int linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
    if (supportsJointMotor(linkIx)) {
      for (int dof = 0; dof < btMultiBody_->getLink(linkIx).m_dofCount; ++dof) {
        int motorId = createJointMotor(linkIx, dof, dofCount, settings);
        dofsToMotorIds[dofCount] = motorId;
        dofCount++;
      }
    } else {
      dofCount += btMultiBody_->getLink(linkIx).m_dofCount;
    }
  }
  Mn::Debug{} << "BulletArticulatedObject::createMotorsForAllDofs(): "
              << dofsToMotorIds;
  return dofsToMotorIds;
}

float BulletArticulatedObject::getJointMotorMaxImpulse(int motorId) {
  CHECK(articulatedJointMotors.count(motorId));
  return articulatedJointMotors.at(motorId)->getMaxAppliedImpulse();
}

int BulletArticulatedObject::createJointMotor(
    const int linkIx,
    const int linkDof,
    const int globalDof,
    const JointMotorSettings& settings) {
  auto motor = JointMotor::create_unique();
  motor->settings = settings;
  motor->dof = globalDof;
  motor->motorId = nextJointMotorId_;
  jointMotors_.emplace(nextJointMotorId_,
                       std::move(motor));  // cache the Habitat structure

  auto btMotor = std::make_unique<btMultiBodyJointMotor>(
      btMultiBody_.get(), linkIx, linkDof, settings.velocityTarget,
      settings.maxImpulse);
  btMotor->setPositionTarget(settings.positionTarget, settings.positionGain);
  btMotor->setVelocityTarget(settings.velocityTarget, settings.velocityGain);
  bWorld_->addMultiBodyConstraint(btMotor.get());
  articulatedJointMotors.emplace(
      nextJointMotorId_, std::move(btMotor));  // cache the Bullet structure
  return nextJointMotorId_++;
}

int BulletArticulatedObject::createJointMotor(
    const int dof,
    const JointMotorSettings& settings) {
  int linkIx = 0;
  int linkDof = -1;
  int dofCount = 0;
  for (; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
    if (dofCount > dof) {
      Mn::Debug{} << "BulletArticulatedObject::createJointMotor failed. " << dof
                  << " is not a valid JointMotor type.";
      return ID_UNDEFINED;
    }
    if (supportsJointMotor(linkIx)) {
      for (int _dof = 0; _dof < btMultiBody_->getLink(linkIx).m_dofCount;
           ++_dof) {
        if (dofCount == dof) {
          linkDof = _dof;
          break;
        }
        dofCount++;
      }
    } else {
      dofCount += btMultiBody_->getLink(linkIx).m_dofCount;
    }
    if (linkDof >= 0) {
      // break out of the loop if we found what we are looking for
      break;
    }
  }

  if (dof > dofCount) {
    Mn::Debug{} << "BulletArticulatedObject::createJointMotor failed. " << dof
                << " is not a valid DOF for this model.";
    return ID_UNDEFINED;
  }

  return createJointMotor(linkIx, linkDof, dof, settings);
}

void BulletArticulatedObject::removeJointMotor(const int motorId) {
  CHECK(jointMotors_.count(motorId) > 0);
  CHECK(articulatedJointMotors.count(motorId));
  bWorld_->removeMultiBodyConstraint(articulatedJointMotors.at(motorId).get());
  jointMotors_.erase(motorId);
  articulatedJointMotors.erase(motorId);
}

void BulletArticulatedObject::updateJointMotor(
    const int motorId,
    const JointMotorSettings& settings) {
  CHECK(jointMotors_.count(motorId) > 0);
  jointMotors_.at(motorId)->settings = settings;
  CHECK(articulatedJointMotors.count(motorId));
  auto& motor = articulatedJointMotors.at(motorId);
  motor->setPositionTarget(settings.positionTarget, settings.positionGain);
  motor->setVelocityTarget(settings.velocityTarget, settings.velocityGain);
  motor->setMaxAppliedImpulse(settings.maxImpulse);
}

void BulletArticulatedObject::clampJointLimits() {
  auto pose = getPositions();
  bool poseModified = false;

  // some small contrived error term for overflow
  float corrective_eps = 0.000001;

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (jointLimitConstraints.count(i) > 0) {
      // a joint limit constraint exists for this link
      auto& jlc = jointLimitConstraints.at(i);

      // position clamping:
      if (pose[dofCount] < jlc.lowerLimit - corrective_eps) {
        poseModified = true;
        pose[dofCount] = jlc.lowerLimit;
      } else if (pose[dofCount] > jlc.upperLimit + corrective_eps) {
        poseModified = true;
        pose[dofCount] = jlc.upperLimit;
      }
    }

    // continue incrementing the dof counter
    for (int dof = 0; dof < btMultiBody_->getLink(i).m_dofCount; ++dof) {
      dofCount++;
    }
  }

  if (poseModified) {
    setPositions(pose);
  }
}

void BulletArticulatedObject::updateKinematicState() {
  btMultiBody_->forwardKinematics(scratch_q_, scratch_m_);
  btMultiBody_->updateCollisionObjectWorldTransforms(scratch_q_, scratch_m_);
  // Need to update the aabbs manually also for broadphase collision detection
  for (size_t linkIx = 0; linkIx < btMultiBody_->getNumLinks(); ++linkIx) {
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
      auto co = static_cast<btCollisionObject*>(proxy0->m_clientObject);
      auto mblc = dynamic_cast<btMultiBodyLinkCollider*>(co);
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

bool BulletArticulatedObject::contactTest(bool staticAsStage) {
  AOSimulationContactResultCallback src(btMultiBody_.get(),
                                        bFixedObjectRigidBody_.get());

  auto baseCollider = btMultiBody_->getBaseCollider();
  // Do a contact test for each piece of the AO and return at soonest contact.
  // Should be cheaper to hit multiple local aabbs than to check the full scene.
  if (bFixedObjectRigidBody_) {
    src.m_collisionFilterGroup =
        bFixedObjectRigidBody_->getBroadphaseHandle()->m_collisionFilterGroup;
    src.m_collisionFilterMask =
        bFixedObjectRigidBody_->getBroadphaseHandle()->m_collisionFilterMask;

    if (!staticAsStage) {
      // consider the fixed base as "robot" instead of "static"
      src.m_collisionFilterGroup = int(CollisionGroup::Robot);
      src.m_collisionFilterMask =
          CollisionGroupHelper::getMaskForGroup(CollisionGroup::Robot);
    }

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
    auto linkCollider = btMultiBody_->getLinkCollider(colIx);
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

}  // namespace physics
}  // namespace esp
