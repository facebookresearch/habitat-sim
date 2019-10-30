// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletRigidObject.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

bool BulletPhysicsManager::initPhysics(
    scene::SceneNode* node,
    const assets::PhysicsManagerAttributes& physicsManagerAttributes) {
  activePhysSimLib_ = BULLET;

  //! We can potentially use other collision checking algorithms, by
  //! uncommenting the line below
  // btGImpactCollisionAlgorithm::registerAlgorithm(&bDispatcher_);
  bWorld_ = std::make_shared<btDiscreteDynamicsWorld>(
      &bDispatcher_, &bBroadphase_, &bSolver_, &bCollisionConfig_);

  debugDrawer_.setMode(
      Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe |
      Magnum::BulletIntegration::DebugDraw::Mode::DrawConstraints);
  bWorld_->setDebugDrawer(&debugDrawer_);

  // Copy over relevant configuration
  fixedTimeStep_ = physicsManagerAttributes.getDouble("timestep");
  // currently GLB meshes are y-up
  bWorld_->setGravity(
      btVector3(physicsManagerAttributes.getMagnumVec3("gravity")));

  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = static_cast<RigidObject*>(new BulletRigidObject(physicsNode_));

  initialized_ = true;
  return true;
}

BulletPhysicsManager::~BulletPhysicsManager() {
  // remove all leftover physical objects
  for (auto& bro : existingObjects_) {
    bro.second->removeObject();
  }
  // remove the physical scene from the world
  sceneNode_->removeObject();
}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addScene(
    const assets::PhysicsSceneAttributes& physicsSceneAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  const assets::MeshMetaData& metaData = resourceManager_->getMeshMetaData(
      physicsSceneAttributes.getString("collisionMeshHandle"));

  //! Initialize scene
  bool sceneSuccess = static_cast<BulletRigidObject*>(sceneNode_)
                          ->initializeScene(physicsSceneAttributes, metaData,
                                            meshGroup, bWorld_);

  return sceneSuccess;
}

int BulletPhysicsManager::makeRigidObject(
    const std::vector<assets::CollisionMeshData>& meshGroup,
    assets::PhysicsObjectAttributes physicsObjectAttributes) {
  //! Create new physics object (child node of sceneNode_)
  int newObjectID = allocateObjectID();
  existingObjects_[newObjectID] = new BulletRigidObject(sceneNode_);

  const assets::MeshMetaData& metaData = resourceManager_->getMeshMetaData(
      physicsObjectAttributes.getString("collisionMeshHandle"));
  bool objectSuccess =
      static_cast<BulletRigidObject*>(existingObjects_.at(newObjectID))
          ->initializeObject(physicsObjectAttributes, bWorld_, metaData,
                             meshGroup);

  if (!objectSuccess) {
    LOG(ERROR) << "Object load failed";
    deallocateObjectID(newObjectID);
    existingObjects_.erase(newObjectID);
    return -1;
  }
  return newObjectID;
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
  for (std::map<int, physics::RigidObject*>::iterator it =
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
  if (dt < 0) {
    dt = fixedTimeStep_;
  }

  // ==== Physics stepforward ======
  // NOTE: worldTime_ will always be a multiple of sceneMetaData_.timestep
  int numSubStepsTaken =
      bWorld_->stepSimulation(dt, maxSubSteps_, fixedTimeStep_);
  worldTime_ += numSubStepsTaken * fixedTimeStep_;
}

void BulletPhysicsManager::setMargin(const int physObjectID,
                                     const double margin) {
  assertIDValidity(physObjectID);
  static_cast<BulletRigidObject*>(existingObjects_.at(physObjectID))
      ->setMargin(margin);
}

void BulletPhysicsManager::setSceneFrictionCoefficient(
    const double frictionCoefficient) {
  static_cast<BulletRigidObject*>(sceneNode_)
      ->setFrictionCoefficient(frictionCoefficient);
}

void BulletPhysicsManager::setSceneRestitutionCoefficient(
    const double restitutionCoefficient) {
  static_cast<BulletRigidObject*>(sceneNode_)
      ->setRestitutionCoefficient(restitutionCoefficient);
}

double BulletPhysicsManager::getMargin(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return static_cast<BulletRigidObject*>(existingObjects_.at(physObjectID))
      ->getMargin();
}

double BulletPhysicsManager::getSceneFrictionCoefficient() const {
  return static_cast<BulletRigidObject*>(sceneNode_)->getFrictionCoefficient();
}

double BulletPhysicsManager::getSceneRestitutionCoefficient() const {
  return static_cast<BulletRigidObject*>(sceneNode_)
      ->getRestitutionCoefficient();
}

void BulletPhysicsManager::debugDraw(const Magnum::Matrix4& projTrans) const {
  debugDrawer_.setTransformationProjectionMatrix(projTrans);
  bWorld_->debugDrawWorld();
}

}  // namespace physics
}  // namespace esp
