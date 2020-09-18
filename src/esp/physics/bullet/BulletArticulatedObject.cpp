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
  Corrade::Utility::Debug() << "deconstructing ~BulletArticulatedObject";
  bWorld_->removeMultiBody(btMultiBody_);
  for (int colIx = 0; colIx < btMultiBody_->getNumLinks(); ++colIx) {
    bWorld_->removeCollisionObject(btMultiBody_->getLinkCollider(colIx));
    collisionObjToObjIds_->erase(btMultiBody_->getLinkCollider(colIx));
  }
  bWorld_->btCollisionWorld::removeCollisionObject(
      btMultiBody_->getBaseCollider());
  collisionObjToObjIds_->erase(btMultiBody_->getBaseCollider());
}

bool BulletArticulatedObject::initializeFromURDF(
    URDFImporter& urdfImporter,
    const Magnum::Matrix4& worldTransform,
    assets::ResourceManager& resourceManager,
    gfx::DrawableGroup* drawables,
    scene::SceneNode* physicsNode,
    bool fixedBase) {
  // TODO: should this be included as optional parameter?
  // btTransform rootTransformInWorldSpace = btTransform::getIdentity();
  Magnum::Matrix4 rootTransformInWorldSpace{worldTransform};
  // rootTransformInWorldSpace.setOrigin(btVector3{0,10.0,0});

  BulletURDFImporter& u2b = *(static_cast<BulletURDFImporter*>(&urdfImporter));
  u2b.setFixedBase(fixedBase);

  const io::UrdfModel& urdfModel = u2b.getModel();

  // TODO: are these needed? Not used in examples.
  int flags = 0;

  Corrade::Utility::Debug()
      << "begin - BulletArticulatedObject::initializeFromURDF";

  Corrade::Utility::Debug() << "model name: " << urdfModel.m_name;

  URDF2BulletCached cache;
  u2b.InitURDF2BulletCache(cache, flags);

  int urdfLinkIndex = u2b.getRootLinkIndex();
  int rootIndex = u2b.getRootLinkIndex();

  // NOTE: recursive path only
  u2b.ConvertURDF2BulletInternal(cache, urdfLinkIndex,
                                 rootTransformInWorldSpace, bWorld_.get(),
                                 flags, linkCollisionShapes_);

  if (cache.m_bulletMultiBody) {
    Corrade::Utility::Debug() << "post process phase";

    btMultiBody* mb = cache.m_bulletMultiBody;

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
    btAlignedObjectArray<btQuaternion> scratch_q;
    btAlignedObjectArray<btVector3> scratch_m;
    mb->forwardKinematics(scratch_q, scratch_m);
    mb->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

    bWorld_->addMultiBody(mb);
    btMultiBody_ = bWorld_->getMultiBody(bWorld_->getNumMultibodies() - 1);
    btMultiBody_->setCanSleep(true);

    // Attach SceneNode visual components
    int urdfLinkIx = 0;
    for (auto& link : urdfModel.m_links) {
      int bulletLinkIx = cache.m_urdfLinkIndices2BulletLinkIndices[urdfLinkIx];
      Corrade::Utility::Debug()
          << "urdfLinkIx = " << urdfLinkIx
          << ", m_name = " << link.second->m_name
          << ", m_linkIndex = " << link.second->m_linkIndex
          << ", bulletLinkIx = " << bulletLinkIx;
      scene::SceneNode* linkNode = &node();
      if (bulletLinkIx >= 0) {
        links_[bulletLinkIx] = std::make_unique<BulletArticulatedLink>(
            &physicsNode->createChild(), bWorld_, bulletLinkIx,
            collisionObjToObjIds_);
        linkNode = &links_[bulletLinkIx]->node();
      }

      bool success = attachGeometry(*linkNode, link.second,
                                    urdfImporter.getModel().m_materials,
                                    resourceManager, drawables);
      Corrade::Utility::Debug() << "geomSuccess: " << success;

      urdfLinkIx++;
    }

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

  Corrade::Utility::Debug()
      << "done - BulletArticulatedObject::initializeFromURDF";
  return true;
}

Magnum::Matrix4 BulletArticulatedObject::getRootState() {
  return Magnum::Matrix4{btMultiBody_->getBaseWorldTransform()};
}

void BulletArticulatedObject::updateNodes() {
  setRotationScalingFromBulletTransform(btMultiBody_->getBaseWorldTransform(),
                                        &node());

  // update link transforms
  for (auto& link : links_) {
    setRotationScalingFromBulletTransform(
        btMultiBody_->getLink(link.first).m_cachedWorldTransform,
        &link.second->node());
  }
}

////////////////////////////
// BulletArticulatedLink
////////////////////////////

bool BulletArticulatedObject::attachGeometry(
    scene::SceneNode& node,
    std::shared_ptr<io::UrdfLink> link,
    const std::map<std::string, std::shared_ptr<io::UrdfMaterial> >& materials,
    assets::ResourceManager& resourceManager,
    gfx::DrawableGroup* drawables) {
  bool geomSuccess = false;

  for (auto& visual : link->m_visualArray) {
    // create a new child for each visual component
    scene::SceneNode& visualGeomComponent = node.createChild();
    visualGeomComponent.setTransformation(
        link->m_inertia.m_linkLocalFrame.invertedRigid() *
        visual.m_linkLocalFrame);

    switch (visual.m_geometry.m_type) {
      case io::URDF_GEOM_CAPSULE:
        Corrade::Utility::Debug() << "Trying to add visual capsule";
        // TODO:
        break;
      case io::URDF_GEOM_CYLINDER:
        Corrade::Utility::Debug() << "Trying to add visual cylinder";
        // TODO:
        break;
      case io::URDF_GEOM_BOX:
        Corrade::Utility::Debug() << "Trying to add visual box";
        // TODO:
        break;
      case io::URDF_GEOM_SPHERE:
        Corrade::Utility::Debug() << "Trying to add visual sphere";
        // TODO:
        break;
      case io::URDF_GEOM_MESH: {
        Corrade::Utility::Debug() << "visual.m_geometry.m_meshFileName = "
                                  << visual.m_geometry.m_meshFileName;
        // visual.m_sourceFileLocation
        visualGeomComponent.scale(visual.m_geometry.m_meshScale);

        // first try to import the asset
        std::shared_ptr<io::UrdfMaterial> material = nullptr;
        if (materials.count(visual.m_materialName)) {
          material = materials.at(visual.m_materialName);
        }
        bool meshSuccess = resourceManager.importAsset(
            visual.m_geometry.m_meshFileName, material);
        if (!meshSuccess) {
          Cr::Utility::Debug() << "Failed to import the render asset: "
                               << visual.m_geometry.m_meshFileName;
          return false;
        }
        // then attach
        geomSuccess = resourceManager.attachAsset(
            visual.m_geometry.m_meshFileName, visualGeomComponent, drawables);
      } break;
      case io::URDF_GEOM_PLANE:
        Corrade::Utility::Debug() << "Trying to add visual plane";
        // TODO:
        break;
    }
  }

  return geomSuccess;
}

void BulletArticulatedObject::setRootState(const Magnum::Matrix4& state) {
  btMultiBody_->setBaseWorldTransform(btTransform{state});
  // update the simulation state
  // TODO: make this optional?
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  btMultiBody_->forwardKinematics(scratch_q, scratch_m);
  btMultiBody_->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
}

void BulletArticulatedObject::setForces(std::vector<float> forces) {
  if (forces.size() != btMultiBody_->getNumDofs()) {
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

void BulletArticulatedObject::setVelocities(std::vector<float> vels) {
  if (vels.size() != btMultiBody_->getNumDofs()) {
    Corrade::Utility::Debug()
        << "setVelocities - Velocity vector size mis-match (input: "
        << vels.size() << ", expected: " << btMultiBody_->getNumDofs()
        << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (btMultiBody_->getLink(i).m_dofCount > 0) {
      btMultiBody_->setJointVelMultiDof(i, &vels[dofCount]);
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

void BulletArticulatedObject::setPositions(std::vector<float> positions) {
  if (positions.size() != btMultiBody_->getNumDofs()) {
    Corrade::Utility::Debug()
        << "setPositions - Position vector size mis-match (input: "
        << positions.size() << ", expected: " << btMultiBody_->getNumDofs()
        << "), aborting.";
  }

  int dofCount = 0;
  for (int i = 0; i < btMultiBody_->getNumLinks(); ++i) {
    if (btMultiBody_->getLink(i).m_dofCount > 0) {
      btMultiBody_->setJointPosMultiDof(i, &positions[dofCount]);
      dofCount += btMultiBody_->getLink(i).m_dofCount;
    }
  }

  // update the simulation state
  // TODO: make this optional?
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  btMultiBody_->forwardKinematics(scratch_q, scratch_m);
  btMultiBody_->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
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

void BulletArticulatedObject::addArticulatedLinkForce(int linkId,
                                                      Magnum::Vector3 force) {
  CHECK(getNumLinks() > linkId);
  btMultiBody_->addLinkForce(linkId, btVector3{force});
}

void BulletArticulatedObject::reset() {
  // reset positions and velocities to zero
  // clears forces/torques
  // Note: does not update root state TODO:?
  std::vector<float> zeros(btMultiBody_->getNumDofs(), 0);
  setPositions(zeros);
  // btMultiBody_->setPosUpdated(true);
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  btMultiBody_->forwardKinematics(scratch_q, scratch_m);
  btMultiBody_->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
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

  if (mt == MotionType::DYNAMIC) {
    bWorld_->addMultiBody(btMultiBody_);
  } else if (mt == MotionType::KINEMATIC) {
    bWorld_->removeMultiBody(btMultiBody_);
  } else {
    return;
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
  btMultiBodyJointMotor* motor = articulatedJointMotors.at(motorId);
  return motor->getMaxAppliedImpulse();
}

int BulletArticulatedObject::createJointMotor(
    const int linkIx,
    const int linkDof,
    const int globalDof,
    const JointMotorSettings& settings) {
  auto motor = JointMotor::create_unique();
  motor->settings = settings;
  motor->dof = globalDof;

  btMultiBodyJointMotor* btMotor =
      new btMultiBodyJointMotor(btMultiBody_, linkIx, linkDof,
                                settings.velocityTarget, settings.maxImpulse);
  btMotor->setPositionTarget(settings.positionTarget, settings.positionGain);
  btMotor->setVelocityTarget(settings.velocityTarget, settings.velocityGain);
  bWorld_->addMultiBodyConstraint(btMotor);
  articulatedJointMotors[nextJointMotorId_] = btMotor;
  motor->motorId = nextJointMotorId_;
  jointMotors_.emplace(motor->motorId, std::move(motor));

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
  btMultiBodyJointMotor* motor = articulatedJointMotors.at(motorId);
  bWorld_->removeMultiBodyConstraint(motor);
  delete motor;
  jointMotors_.erase(motorId);
}

void BulletArticulatedObject::updateJointMotor(
    const int motorId,
    const JointMotorSettings& settings) {
  CHECK(jointMotors_.count(motorId) > 0);
  jointMotors_.at(motorId)->settings = settings;
  CHECK(articulatedJointMotors.count(motorId));
  btMultiBodyJointMotor* motor = articulatedJointMotors.at(motorId);
  motor->setPositionTarget(settings.positionTarget, settings.positionGain);
  motor->setVelocityTarget(settings.velocityTarget, settings.velocityGain);
  motor->setMaxAppliedImpulse(settings.maxImpulse);
}

}  // namespace physics
}  // namespace esp
