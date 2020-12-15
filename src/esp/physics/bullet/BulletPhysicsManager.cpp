// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletArticulatedObject.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletRigidObject.h"
#include "BulletURDFImporter.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

BulletPhysicsManager::~BulletPhysicsManager() {
  LOG(INFO) << "Deconstructing BulletPhysicsManager";

  existingObjects_.clear();
  existingArticulatedObjects_.clear();
  staticStageObject_.reset(nullptr);
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
  bWorld_->setGravity(btVector3(physicsManagerAttributes_->getVec3("gravity")));

  Corrade::Utility::Debug() << "creating staticStageObject_";
  //! Create new scene node
  staticStageObject_ = physics::BulletRigidStage::create_unique(
      &physicsNode_->createChild(), resourceManager_, bWorld_,
      collisionObjToObjIds_);
  Corrade::Utility::Debug() << "creating staticStageObject_ .. done";

  m_recentNumSubStepsTaken = -1;

  return true;
}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addStageFinalize(const std::string& handle) {
  //! Initialize scene
  bool sceneSuccess = staticStageObject_->initialize(handle);

  return sceneSuccess;
}

bool BulletPhysicsManager::makeAndAddRigidObject(int newObjectID,
                                                 const std::string& handle,
                                                 scene::SceneNode* objectNode) {
  auto ptr = physics::BulletRigidObject::create_unique(
      objectNode, newObjectID, resourceManager_, bWorld_,
      collisionObjToObjIds_);
  bool objSuccess = ptr->initialize(handle);
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
  scene::SceneNode* objectNode = &staticStageObject_->node().createChild();
  BulletArticulatedObject::uptr articulatedObject =
      BulletArticulatedObject::create_unique(objectNode, resourceManager_,
                                             bWorld_, collisionObjToObjIds_);

  bool objectSuccess = articulatedObject->initializeFromURDF(
      urdfImporter_, {}, drawables, physicsNode_, fixedBase);

  if (!objectSuccess) {
    delete objectNode;
    return ID_UNDEFINED;
  }

  // Cr::Utility::Debug() << "Articulated Link Indices: "
  //                     << articulatedObject->getLinkIds();

  int articulatedObjectID_ = allocateObjectID();

  // allocate ids for links
  for (int linkIx = 0; linkIx < articulatedObject->btMultiBody_->getNumLinks();
       ++linkIx) {
    int linkObjectId = allocateObjectID();
    articulatedObject->objectIdToLinkId_[linkObjectId] = linkIx;
    collisionObjToObjIds_->emplace(
        articulatedObject->btMultiBody_->getLinkCollider(linkIx), linkObjectId);
  }
  // base collider refers to the articulated object's id
  collisionObjToObjIds_->emplace(
      articulatedObject->btMultiBody_->getBaseCollider(), articulatedObjectID_);

  existingArticulatedObjects_.emplace(articulatedObjectID_,
                                      std::move(articulatedObject));

  return articulatedObjectID_;
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
  m_recentNumSubStepsTaken = numSubStepsTaken;

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

void BulletPhysicsManager::setStageFrictionCoefficient(
    const double frictionCoefficient) {
  staticStageObject_->setFrictionCoefficient(frictionCoefficient);
}

void BulletPhysicsManager::setStageRestitutionCoefficient(
    const double restitutionCoefficient) {
  staticStageObject_->setRestitutionCoefficient(restitutionCoefficient);
}

double BulletPhysicsManager::getMargin(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return static_cast<BulletRigidObject*>(
             existingObjects_.at(physObjectID).get())
      ->getMargin();
}

double BulletPhysicsManager::getStageFrictionCoefficient() const {
  return staticStageObject_->getFrictionCoefficient();
}

double BulletPhysicsManager::getStageRestitutionCoefficient() const {
  return staticStageObject_->getRestitutionCoefficient();
}

const Magnum::Range3D BulletPhysicsManager::getCollisionShapeAabb(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return static_cast<BulletRigidObject*>(
             existingObjects_.at(physObjectID).get())
      ->getCollisionShapeAabb();
}

const Magnum::Range3D BulletPhysicsManager::getStageCollisionShapeAabb() const {
  return static_cast<BulletRigidStage*>(staticStageObject_.get())
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

void BulletPhysicsManager::overrideCollisionGroup(const int physObjectID,
                                                  CollisionGroup group) const {
  assertIDValidity(physObjectID);
  static_cast<BulletRigidObject*>(existingObjects_.at(physObjectID).get())
      ->overrideCollisionGroup(group);
}

int BulletPhysicsManager::createRigidP2PConstraint(
    int objectId,
    const Magnum::Vector3& position,
    bool positionLocal) {
  CHECK(existingObjects_.count(objectId));
  if (existingObjects_.at(objectId)->getMotionType() == MotionType::DYNAMIC) {
    btRigidBody* rb =
        static_cast<BulletRigidObject*>(existingObjects_.at(objectId).get())
            ->bObjectRigidBody_.get();
    rb->setActivationState(DISABLE_DEACTIVATION);

    Magnum::Vector3 localOffset = position;
    if (!positionLocal) {
      btVector3 localPivot =
          rb->getCenterOfMassTransform().inverse() * btVector3(position);
      localOffset = Magnum::Vector3(localPivot);
    }

    btPoint2PointConstraint* p2p =
        new btPoint2PointConstraint(*rb, btVector3(localOffset));
    bWorld_->addConstraint(p2p);
    rigidP2ps.emplace(nextConstraintId_, p2p);
    return nextConstraintId_++;
  } else {
    Corrade::Utility::Debug()
        << "Cannot create a dynamic point-2-point constraint for object with "
           "MotionType != DYNAMIC";
    return ID_UNDEFINED;
  }
}

int BulletPhysicsManager::createArticulatedP2PConstraint(
    int articulatedObjectId,
    int linkId,
    const Magnum::Vector3& linkOffset,
    const Magnum::Vector3& pickPos) {
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
  articulatedP2ps.emplace(nextConstraintId_, p2p);
  return nextConstraintId_++;
}

int BulletPhysicsManager::createArticulatedP2PConstraint(
    int articulatedObjectId,
    int linkId,
    const Magnum::Vector3& pickPos) {
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

void BulletPhysicsManager::updateP2PConstraintPivot(
    int constraintId,
    const Magnum::Vector3& pivot) {
  if (articulatedP2ps.count(constraintId)) {
    articulatedP2ps.at(constraintId)->setPivotInB(btVector3(pivot));
  } else if (rigidP2ps.count(constraintId)) {
    rigidP2ps.at(constraintId)->setPivotB(btVector3(pivot));
  } else {
    Corrade::Utility::Debug() << "No P2P constraint with ID: " << constraintId;
  }
}

void BulletPhysicsManager::removeConstraint(int constraintId) {
  if (articulatedP2ps.count(constraintId)) {
    articulatedP2ps.at(constraintId)->getMultiBodyA()->setCanSleep(true);
    bWorld_->removeMultiBodyConstraint(articulatedP2ps.at(constraintId));
    delete articulatedP2ps.at(constraintId);
    articulatedP2ps.erase(constraintId);
  } else if (rigidP2ps.count(constraintId)) {
    rigidP2ps.at(constraintId)
        ->getRigidBodyA()
        .setActivationState(WANTS_DEACTIVATION);
    bWorld_->removeConstraint(rigidP2ps.at(constraintId));
    delete rigidP2ps.at(constraintId);
    rigidP2ps.erase(constraintId);
  } else if (articulatedFixedConstraints.count(constraintId)) {
    bWorld_->removeMultiBodyConstraint(
        articulatedFixedConstraints.at(constraintId));
    delete articulatedFixedConstraints.at(constraintId);
    articulatedFixedConstraints.erase(constraintId);
  } else {
    Corrade::Utility::Debug() << "No constraint with ID: " << constraintId;
  }
};

int BulletPhysicsManager::createArticulatedP2PConstraint(
    int articulatedObjectId,
    int linkId,
    int objectId,
    float maxImpulse,
    const Corrade::Containers::Optional<Magnum::Vector3>& pivotA,
    const Corrade::Containers::Optional<Magnum::Vector3>& pivotB) {
  CHECK(existingArticulatedObjects_.count(articulatedObjectId));
  CHECK(existingArticulatedObjects_.at(articulatedObjectId)->getNumLinks() >
        linkId);
  CHECK(existingObjects_.count(objectId));

  btRigidBody* rb = nullptr;
  if (existingObjects_.at(objectId)->getMotionType() == MotionType::DYNAMIC) {
    rb = static_cast<BulletRigidObject*>(existingObjects_.at(objectId).get())
             ->bObjectRigidBody_.get();
  } else {
    Corrade::Utility::Debug()
        << "Cannot create a dynamic P2P constraint for object with "
           "MotionType != DYNAMIC";
    return ID_UNDEFINED;
  }

  btMultiBody* mb =
      static_cast<BulletArticulatedObject*>(
          existingArticulatedObjects_.at(articulatedObjectId).get())
          ->btMultiBody_;
  mb->setCanSleep(false);

  // use origin if not specified
  btVector3 pivotInB = pivotB ? btVector3(*pivotB) : btVector3(0.f, 0.f, 0.f);

  btVector3 pivotInA;
  if (pivotA) {
    pivotInA = btVector3(*pivotA);
  } else {
    // hold object at it's current position relative to link
    btVector3 pivotWorld = rb->getCenterOfMassTransform() * pivotInB;
    pivotInA = mb->worldPosToLocal(linkId, pivotWorld);
  }

  btMultiBodyPoint2Point* p2p =
      new btMultiBodyPoint2Point(mb, linkId, rb, pivotInA, pivotInB);
  p2p->setMaxAppliedImpulse(maxImpulse);
  bWorld_->addMultiBodyConstraint(p2p);
  articulatedP2ps.emplace(nextConstraintId_, p2p);
  return nextConstraintId_++;
}

int BulletPhysicsManager::createArticulatedFixedConstraint(
    int articulatedObjectId,
    int linkId,
    int objectId,
    float maxImpulse,
    const Corrade::Containers::Optional<Magnum::Vector3>& pivotA,
    const Corrade::Containers::Optional<Magnum::Vector3>& pivotB) {
  CHECK(existingArticulatedObjects_.count(articulatedObjectId));
  CHECK(existingArticulatedObjects_.at(articulatedObjectId)->getNumLinks() >
        linkId);
  CHECK(existingObjects_.count(objectId));

  btRigidBody* rb = nullptr;
  if (existingObjects_.at(objectId)->getMotionType() == MotionType::DYNAMIC) {
    rb = static_cast<BulletRigidObject*>(existingObjects_.at(objectId).get())
             ->bObjectRigidBody_.get();
  } else {
    Corrade::Utility::Debug()
        << "Cannot create a dynamic fixed constraint for object with "
           "MotionType != DYNAMIC";
    return ID_UNDEFINED;
  }

  btMultiBody* mb =
      static_cast<BulletArticulatedObject*>(
          existingArticulatedObjects_.at(articulatedObjectId).get())
          ->btMultiBody_;
  mb->setCanSleep(false);

  // use origin if not specified
  // todo: avoid code duplication here and in createArticulatedP2PConstraint
  btVector3 pivotInB = pivotB ? btVector3(*pivotB) : btVector3(0.f, 0.f, 0.f);
  btVector3 pivotInA;
  if (pivotA) {
    pivotInA = btVector3(*pivotA);
  } else {
    // hold object at it's current position relative to link
    btVector3 pivotWorld = rb->getCenterOfMassTransform() * pivotInB;
    btVector3 pivotInA = mb->worldPosToLocal(linkId, pivotWorld);
  }

  // We constrain the relative orientation of the link and object to match their
  // current relative orientation.
  btMatrix3x3 frameInA = btMatrix3x3::getIdentity();
  btMatrix3x3 frameWorld = mb->localFrameToWorld(linkId, frameInA);
  // note btMatrix3x3 tranpose() equivalent to inverse because mat is
  // orthonormal
  btMatrix3x3 frameInB =
      (frameWorld * rb->getCenterOfMassTransform().getBasis().transpose())
          .transpose();

  auto* constraint = new btMultiBodyFixedConstraint(
      mb, linkId, rb, pivotInA, pivotInB, frameInA, frameInB);
  constraint->setMaxAppliedImpulse(maxImpulse);
  bWorld_->addMultiBodyConstraint(constraint);
  articulatedFixedConstraints.emplace(nextConstraintId_, constraint);
  return nextConstraintId_++;
}

RaycastResults BulletPhysicsManager::castRay(const esp::geo::Ray& ray,
                                             double maxDistance) {
  RaycastResults results;
  results.ray = ray;
  double rayLength = ray.direction.length();
  if (rayLength == 0) {
    LOG(ERROR) << "BulletPhysicsManager::castRay : Cannot case ray with zero "
                  "length, aborting. ";
    return results;
  }
  btVector3 from(ray.origin);
  btVector3 to(ray.origin + ray.direction * maxDistance);

  btCollisionWorld::AllHitsRayResultCallback allResults(from, to);
  bWorld_->rayTest(from, to, allResults);

  // convert to RaycastResults
  for (int i = 0; i < allResults.m_hitPointWorld.size(); ++i) {
    RayHitInfo hit;

    hit.normal = Magnum::Vector3{allResults.m_hitNormalWorld[i]};
    hit.point = Magnum::Vector3{allResults.m_hitPointWorld[i]};
    hit.rayDistance = (allResults.m_hitFractions[i] * maxDistance) / rayLength;
    // default to -1 for "scene collision" if we don't know which object was
    // involved
    hit.objectId = -1;
    if (collisionObjToObjIds_->count(allResults.m_collisionObjects[i]) > 0) {
      hit.objectId =
          collisionObjToObjIds_->at(allResults.m_collisionObjects[i]);
    }
    results.hits.push_back(hit);
  }
  results.sortByDistance();
  return results;
}

// todo: unit test for this
void BulletPhysicsManager::lookUpObjectIdAndLinkId(
    const btCollisionObject* colObj,
    int* objectId,
    int* linkId) const {
  ASSERT(objectId);
  ASSERT(linkId);

  *linkId = -1;
  // If the lookup fails, default to the stage. TODO: better error-handling.
  *objectId = -1;

  if (collisionObjToObjIds_->count(colObj)) {
    int rawObjectId = collisionObjToObjIds_->at(colObj);
    if (existingObjects_.count(rawObjectId)) {
      *objectId = rawObjectId;
      return;
    } else if (existingArticulatedObjects_.count(rawObjectId)) {
      *objectId = rawObjectId;
      return;
    } else {
      // search articulated objects to see if this is a link
      for (const auto& pair : existingArticulatedObjects_) {
        if (pair.second->objectIdToLinkId_.count(rawObjectId)) {
          *objectId = pair.first;
          *linkId = pair.second->objectIdToLinkId_.at(rawObjectId);
          return;
        }
      }
    }
  }

  // lookup failed
}

std::vector<ContactPointData> BulletPhysicsManager::getContactPoints() const {
  if (m_recentNumSubStepsTaken != 1) {
    if (m_recentNumSubStepsTaken == -1) {
      LOG(WARNING) << "getContactPoints: no previous call to stepPhysics";
    } else {
      // todo: proper logging-throttling API
      static int count = 0;
      if (count++ < 5) {
        LOG(WARNING)
            << "getContactPoints: the previous call to stepPhysics performed "
            << m_recentNumSubStepsTaken
            << " substeps, so getContactPoints's behavior may be unexpected.";
      }
      if (count == 5) {
        LOG(WARNING)
            << "getContactPoints: additional warnings will be suppressed.";
      }
    }
  }

  std::vector<ContactPointData> contactPoints;

  // sloppy: assume fixedTimeStep_ hasn't changed since last call to stepPhysics
  const float recentSubstepDt = fixedTimeStep_;

  auto* dispatcher = bWorld_->getDispatcher();
  int numContactManifolds = dispatcher->getNumManifolds();
  contactPoints.reserve(numContactManifolds * 4);
  for (int i = 0; i < numContactManifolds; i++) {
    const btPersistentManifold* manifold =
        dispatcher->getInternalManifoldPointer()[i];

    int objectIdA = -2;  // stage is -1
    int objectIdB = -2;
    int linkIndexA = -1;  // -1 if not a multibody
    int linkIndexB = -1;

    const btCollisionObject* colObj0 = manifold->getBody0();
    const btCollisionObject* colObj1 = manifold->getBody1();

    lookUpObjectIdAndLinkId(colObj0, &objectIdA, &linkIndexA);
    lookUpObjectIdAndLinkId(colObj1, &objectIdB, &linkIndexB);

    // logic copied from btSimulationIslandManager::buildIslands. We count
    // manifolds as active only if related to non-sleeping bodies.
    bool isActive =
        (((colObj0) && colObj0->getActivationState() != ISLAND_SLEEPING) ||
         ((colObj1) && colObj1->getActivationState() != ISLAND_SLEEPING));

    for (int p = 0; p < manifold->getNumContacts(); p++) {
      ContactPointData pt;
      pt.objectIdA = objectIdA;
      pt.objectIdB = objectIdB;
      const btManifoldPoint& srcPt = manifold->getContactPoint(p);
      pt.contactDistance = srcPt.getDistance();
      pt.linkIndexA = linkIndexA;
      pt.linkIndexB = linkIndexB;
      pt.contactNormalOnBInWS = Mn::Vector3(srcPt.m_normalWorldOnB);
      pt.positionOnAInWS = Mn::Vector3(srcPt.getPositionWorldOnA());
      pt.positionOnBInWS = Mn::Vector3(srcPt.getPositionWorldOnB());

      pt.normalForce = srcPt.getAppliedImpulse() / recentSubstepDt;

      pt.linearFrictionForce1 =
          srcPt.m_appliedImpulseLateral1 / recentSubstepDt;
      pt.linearFrictionForce2 =
          srcPt.m_appliedImpulseLateral2 / recentSubstepDt;

      pt.linearFrictionDirection1 = Mn::Vector3(srcPt.m_lateralFrictionDir1);
      pt.linearFrictionDirection2 = Mn::Vector3(srcPt.m_lateralFrictionDir2);

      pt.isActive = isActive;

      contactPoints.push_back(pt);
    }
  }

  return contactPoints;
}

}  // namespace physics
}  // namespace esp
