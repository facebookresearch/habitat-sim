// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletArticulatedObject.h"
#include "BulletRigidObject.h"
#include "BulletURDFImporter.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

BulletPhysicsManager::~BulletPhysicsManager() {
  LOG(INFO) << "Deconstructing BulletPhysicsManager";

  existingObjects_.clear();
  existingArticulatedObjects_.clear();
  staticSceneObject_.reset(nullptr);
}

bool BulletPhysicsManager::initPhysicsFinalize() {
  activePhysSimLib_ = BULLET;

  //! We can potentially use other collision checking algorithms, by
  //! uncommenting the line below
  // btGImpactCollisionAlgorithm::registerAlgorithm(&bDispatcher_);
  bWorld_ = std::make_shared<btMultiBodyDynamicsWorld>(
      &bDispatcher_, &bBroadphase_, &bSolver_, &bCollisionConfig_);

  debugDrawer_.setMode(
      Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe |
      Magnum::BulletIntegration::DebugDraw::Mode::DrawConstraints);
  bWorld_->setDebugDrawer(&debugDrawer_);

  // currently GLB meshes are y-up
  bWorld_->setGravity(btVector3(physicsManagerAttributes->getVec3("gravity")));

  //! Create new scene node
  staticSceneObject_ = physics::BulletRigidScene::create_unique(
      &physicsNode_->createChild(), bWorld_);

  return true;
}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addSceneFinalize(
    const assets::PhysicsSceneAttributes::ptr physicsSceneAttributes) {
  //! Initialize scene
  bool sceneSuccess =
      staticSceneObject_->initialize(resourceManager_, physicsSceneAttributes);

  return sceneSuccess;
}

bool BulletPhysicsManager::makeAndAddRigidObject(
    int newObjectID,
    assets::PhysicsObjectAttributes::ptr physicsObjectAttributes,
    scene::SceneNode* objectNode) {
  auto ptr = physics::BulletRigidObject::create_unique(objectNode, bWorld_);
  bool objSuccess = ptr->initialize(resourceManager_, physicsObjectAttributes);
  if (objSuccess) {
    existingObjects_.emplace(newObjectID, std::move(ptr));
  }
  return objSuccess;
}

int BulletPhysicsManager::addArticulatedObjectFromURDF(std::string filepath,
                                                       DrawableGroup* drawables,
                                                       bool fixedBase) {
  // import the URDF
  BulletURDFImporter urdfImporter_(resourceManager_);

  if (!urdfImporter_.loadURDF(filepath)) {
    Corrade::Utility::Debug() << "E - failed to parse URDF file";
    return ID_UNDEFINED;
  }

  // parse succeeded, attempt to create the articulated object
  scene::SceneNode* objectNode = &staticSceneObject_->node().createChild();
  BulletArticulatedObject::uptr articulatedObject =
      BulletArticulatedObject::create_unique(objectNode, bWorld_);

  bool objectSuccess = articulatedObject->initializeFromURDF(
      urdfImporter_, {}, resourceManager_, drawables, physicsNode_, fixedBase);

  if (!objectSuccess) {
    delete objectNode;
    return ID_UNDEFINED;
  }

  int nextObjectID_ = allocateObjectID();
  existingArticulatedObjects_.emplace(nextObjectID_,
                                      std::move(articulatedObject));

  return nextObjectID_;
}

//! Check if mesh primitive is compatible with physics
bool BulletPhysicsManager::isMeshPrimitiveValid(
    const assets::CollisionMeshData& meshData) {
  if (meshData.primitive == Magnum::MeshPrimitive::Triangles) {
    //! Only triangle mesh works
    return true;
  } else {
    switch (meshData.primitive) {
      case Magnum::MeshPrimitive::Lines:
        LOG(ERROR) << "Invalid primitive: Lines";
        break;
      case Magnum::MeshPrimitive::Points:
        LOG(ERROR) << "Invalid primitive: Points";
        break;
      case Magnum::MeshPrimitive::LineLoop:
        LOG(ERROR) << "Invalid primitive Line loop";
        break;
      case Magnum::MeshPrimitive::LineStrip:
        LOG(ERROR) << "Invalid primitive Line Strip";
        break;
      case Magnum::MeshPrimitive::TriangleStrip:
        LOG(ERROR) << "Invalid primitive Triangle Strip";
        break;
      case Magnum::MeshPrimitive::TriangleFan:
        LOG(ERROR) << "Invalid primitive Triangle Fan";
        break;
      default:
        LOG(ERROR) << "Invalid primitive " << int(meshData.primitive);
    }
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return false;
  }
}

void BulletPhysicsManager::setGravity(const Magnum::Vector3& gravity) {
  bWorld_->setGravity(btVector3(gravity));
  // After gravity change, need to reactive all bullet objects
  for (std::map<int, physics::RigidObject::uptr>::iterator it =
           existingObjects_.begin();
       it != existingObjects_.end(); ++it) {
    it->second->setActive();
  }
}

Magnum::Vector3 BulletPhysicsManager::getGravity() const {
  return Magnum::Vector3(bWorld_->getGravity());
}

void BulletPhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_) {
    return;
  }
  if (dt <= 0) {
    dt = fixedTimeStep_;
  }

  // set specified control velocities
  for (auto& objectItr : existingObjects_) {
    VelocityControl::ptr velControl = objectItr.second->getVelocityControl();
    if (objectItr.second->getMotionType() == MotionType::KINEMATIC) {
      // kinematic velocity control intergration
      if (velControl->controllingAngVel || velControl->controllingLinVel) {
        objectItr.second->setRigidState(velControl->integrateTransform(
            dt, objectItr.second->getRigidState()));
        objectItr.second->setActive();
      }
    } else if (objectItr.second->getMotionType() == MotionType::DYNAMIC) {
      if (velControl->controllingLinVel) {
        if (velControl->linVelIsLocal) {
          setLinearVelocity(objectItr.first,
                            objectItr.second->node().rotation().transformVector(
                                velControl->linVel));
        } else {
          setLinearVelocity(objectItr.first, velControl->linVel);
        }
      }
      if (velControl->controllingAngVel) {
        if (velControl->angVelIsLocal) {
          setAngularVelocity(
              objectItr.first,
              objectItr.second->node().rotation().transformVector(
                  velControl->angVel));
        } else {
          setAngularVelocity(objectItr.first, velControl->angVel);
        }
      }
    }
  }

  // ==== Physics stepforward ======
  // NOTE: worldTime_ will always be a multiple of sceneMetaData_.timestep
  int numSubStepsTaken =
      bWorld_->stepSimulation(dt, /*maxSubSteps*/ 10000, fixedTimeStep_);
  worldTime_ += numSubStepsTaken * fixedTimeStep_;

  // update the multi-body SceneNodes TODO: could the be wrapped into some
  // automated update?
  for (auto& ao : existingArticulatedObjects_) {
    ao.second->updateNodes();
  }
}

void BulletPhysicsManager::setMargin(const int physObjectID,
                                     const double margin) {
  assertIDValidity(physObjectID);
  static_cast<BulletRigidObject*>(existingObjects_.at(physObjectID).get())
      ->setMargin(margin);
}

void BulletPhysicsManager::setSceneFrictionCoefficient(
    const double frictionCoefficient) {
  staticSceneObject_->setFrictionCoefficient(frictionCoefficient);
}

void BulletPhysicsManager::setSceneRestitutionCoefficient(
    const double restitutionCoefficient) {
  staticSceneObject_->setRestitutionCoefficient(restitutionCoefficient);
}

double BulletPhysicsManager::getMargin(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return static_cast<BulletRigidObject*>(
             existingObjects_.at(physObjectID).get())
      ->getMargin();
}

double BulletPhysicsManager::getSceneFrictionCoefficient() const {
  return staticSceneObject_->getFrictionCoefficient();
}

double BulletPhysicsManager::getSceneRestitutionCoefficient() const {
  return staticSceneObject_->getRestitutionCoefficient();
}

const Magnum::Range3D BulletPhysicsManager::getCollisionShapeAabb(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return static_cast<BulletRigidObject*>(
             existingObjects_.at(physObjectID).get())
      ->getCollisionShapeAabb();
}

const Magnum::Range3D BulletPhysicsManager::getSceneCollisionShapeAabb() const {
  return static_cast<BulletRigidScene*>(staticSceneObject_.get())
      ->getCollisionShapeAabb();
}

void BulletPhysicsManager::debugDraw(const Magnum::Matrix4& projTrans) const {
  debugDrawer_.setTransformationProjectionMatrix(projTrans);
  bWorld_->debugDrawWorld();
}

bool BulletPhysicsManager::contactTest(const int physObjectID) {
  assertIDValidity(physObjectID);
  bWorld_->getCollisionWorld()->performDiscreteCollisionDetection();
  return static_cast<BulletRigidObject*>(
             existingObjects_.at(physObjectID).get())
      ->contactTest();
}

btCollisionWorld::AllHitsRayResultCallback BulletPhysicsManager::castRay(
    Magnum::Vector3 origin,
    Magnum::Vector3 direction) {
  btVector3 from(origin);
  btVector3 to(origin + direction * 100.0);
  // m_dynamicsWorld->getDebugDrawer()->drawLine(from, to, btVector4(0, 0, 0,
  // 1));
  btCollisionWorld::AllHitsRayResultCallback allResults(from, to);
  bWorld_->rayTest(from, to, allResults);
  return allResults;
}

int BulletPhysicsManager::getObjectIDFromCollisionObject(
    const btCollisionObject* collisionObject) {
  for (auto& obj : existingObjects_) {
    BulletRigidObject* bro = static_cast<BulletRigidObject*>(obj.second.get());
    if (bro->isMe(collisionObject)) {
      return obj.first;
    }
  }
  return ID_UNDEFINED;
}

// TODO: this logic belongs in BulletArticulatedLink
int BulletPhysicsManager::getLinkIDFromCollisionObject(
    int objectId,
    const btCollisionObject* collisionObject) {
  CHECK(existingArticulatedObjects_.count(objectId));

  BulletArticulatedObject* bao = static_cast<BulletArticulatedObject*>(
      existingArticulatedObjects_.at(objectId).get());

  std::map<int, btCollisionShape*>::iterator it =
      bao->linkCollisionShapes_.begin();

  while (it != bao->linkCollisionShapes_.end()) {
    if (it->second == collisionObject->getCollisionShape()) {
      return it->first;
    }
    it++;
  }
  return ID_UNDEFINED;
}

int BulletPhysicsManager::createRigidP2PConstraint(
    int objectId,
    Magnum::Vector3 localOffset) {
  CHECK(existingObjects_.count(objectId));
  if (existingObjects_.at(objectId)->getMotionType() == MotionType::DYNAMIC) {
    btRigidBody* rb =
        static_cast<BulletRigidObject*>(existingObjects_.at(objectId).get())
            ->bObjectRigidBody_.get();
    rb->setActivationState(DISABLE_DEACTIVATION);
    btPoint2PointConstraint* p2p =
        new btPoint2PointConstraint(*rb, btVector3(localOffset));
    bWorld_->addConstraint(p2p);
    rigidP2ps.emplace(nextP2PId_, p2p);
    return nextP2PId_++;
  } else {
    Corrade::Utility::Debug()
        << "Cannot create a dynamic point-2-point constraint for object with "
           "MotionType != DYNAMIC";
    return ID_UNDEFINED;
  }
}

int BulletPhysicsManager::createRigidP2PConstraintFromPickPoint(
    int objectId,
    Magnum::Vector3 pickPoint) {
  CHECK(existingObjects_.count(objectId));
  Magnum::Vector3 localOffset(pickPoint);
  if (existingObjects_.at(objectId)->getMotionType() == MotionType::DYNAMIC) {
    btRigidBody* rb =
        static_cast<BulletRigidObject*>(existingObjects_.at(objectId).get())
            ->bObjectRigidBody_.get();
    btVector3 localPivot =
        rb->getCenterOfMassTransform().inverse() * btVector3(pickPoint);
    localOffset = Magnum::Vector3(localPivot);
  }
  return createRigidP2PConstraint(objectId, localOffset);
}

int BulletPhysicsManager::createArticulatedP2PConstraint(
    int articulatedObjectId,
    int linkId,
    Magnum::Vector3 linkOffset,
    Magnum::Vector3 pickPos) {
  CHECK(existingArticulatedObjects_.count(articulatedObjectId));
  CHECK(existingArticulatedObjects_.at(articulatedObjectId)->getNumLinks() >
        linkId);
  btMultiBody* mb =
      static_cast<BulletArticulatedObject*>(
          existingArticulatedObjects_.at(articulatedObjectId).get())
          ->btMultiBody_;
  mb->setCanSleep(false);
  btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(
      mb, linkId, 0, btVector3(linkOffset), btVector3(pickPos));
  btScalar scaling = 1;
  p2p->setMaxAppliedImpulse(2 * scaling);
  bWorld_->addMultiBodyConstraint(p2p);
  articulatedP2ps.emplace(nextP2PId_, p2p);
  return nextP2PId_++;
}

int BulletPhysicsManager::createArticulatedP2PConstraint(
    int articulatedObjectId,
    int linkId,
    Magnum::Vector3 pickPos) {
  CHECK(existingArticulatedObjects_.count(articulatedObjectId));
  CHECK(existingArticulatedObjects_.at(articulatedObjectId)->getNumLinks() >
        linkId);
  btMultiBody* mb =
      static_cast<BulletArticulatedObject*>(
          existingArticulatedObjects_.at(articulatedObjectId).get())
          ->btMultiBody_;
  btVector3 pivotInA = mb->worldPosToLocal(linkId, btVector3(pickPos));
  return createArticulatedP2PConstraint(articulatedObjectId, linkId,
                                        Magnum::Vector3(pivotInA), pickPos);
}

void BulletPhysicsManager::updateP2PConstraintPivot(int p2pId,
                                                    Magnum::Vector3 pivot) {
  if (articulatedP2ps.count(p2pId)) {
    articulatedP2ps.at(p2pId)->setPivotInB(btVector3(pivot));
  } else if (rigidP2ps.count(p2pId)) {
    rigidP2ps.at(p2pId)->setPivotB(btVector3(pivot));
  } else {
    Corrade::Utility::Debug() << "No P2P constraint with ID: " << p2pId;
  }
}

void BulletPhysicsManager::removeP2PConstraint(int p2pId) {
  if (articulatedP2ps.count(p2pId)) {
    articulatedP2ps.at(p2pId)->getMultiBodyA()->setCanSleep(true);
    bWorld_->removeMultiBodyConstraint(articulatedP2ps.at(p2pId));
    delete articulatedP2ps.at(p2pId);
    articulatedP2ps.erase(p2pId);
  } else if (rigidP2ps.count(p2pId)) {
    rigidP2ps.at(p2pId)->getRigidBodyA().setActivationState(WANTS_DEACTIVATION);
    bWorld_->removeConstraint(rigidP2ps.at(p2pId));
    delete rigidP2ps.at(p2pId);
    rigidP2ps.erase(p2pId);
  } else {
    Corrade::Utility::Debug() << "No P2P constraint with ID: " << p2pId;
  }
};

}  // namespace physics
}  // namespace esp
