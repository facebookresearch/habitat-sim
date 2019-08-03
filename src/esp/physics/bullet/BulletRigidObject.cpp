// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletRigidObject.h"

//!  A Few considerations in construction
//!  Bullet Mesh conversion adapted from:
//!      https://github.com/mosra/magnum-integration/issues/20
//!      https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=11001
//!  Bullet object margin (p15):
//!      https://facultyfp.salisbury.edu/despickler/personal/Resources/
//!        GraphicsExampleCodeGLSL_SFML/InterfaceDoc/Bullet/Bullet_User_Manual.pdf
//!      It's okay to set margin down to 1mm
//!        (1) Bullet/MJCF example
//!      Another solution:
//!        (1) Keep 4cm margin
//!        (2) Use examples/Importers/ImportBsp

namespace esp {
namespace physics {

BulletRigidObject::BulletRigidObject(scene::SceneNode* parent)
    : RigidObject{parent} {};

BulletRigidObject::~BulletRigidObject() {
  if (initialized_) {
    LOG(INFO) << "Deleting object ";
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

bool BulletRigidObject::initializeScene(
    assets::PhysicsSceneMetaData& sceneMetaData,
    std::vector<assets::CollisionMeshData> meshGroup,
    std::shared_ptr<btDiscreteDynamicsWorld> bWorld) {
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isObject_) {
    return false;
  }
  isScene_ = true;
  objectMotionType_ = STATIC;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Object Physical Parameters
  LOG(INFO) << "Creating Instance object meshGroups: " << meshGroup.size();

  //! Iterate through all mesh components for one scene
  //! All components are registered as static objects
  bSceneArray_ = std::make_unique<btTriangleIndexVertexArray>();
  for (assets::CollisionMeshData& meshData : meshGroup) {
    //! Here we convert Magnum's unsigned int indices to
    //! signed indices in bullet. Assuming that it's save to
    //! cast uint to int
    Corrade::Containers::ArrayView<Magnum::Vector3> v_data = meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data =
        meshData.indices;
    LOG(INFO) << "Instance Mesh v data count " << v_data.size();
    LOG(INFO) << "Instance Mesh triangle count " << ui_data.size() / 3;
    LOG(INFO) << "Last mesh face index: " << ui_data[ui_data.size() - 1];
    LOG(INFO) << "Last mesh face vertex: "
              << v_data[ui_data[ui_data.size() - 1]][0] << " "
              << v_data[ui_data[ui_data.size() - 1]][1] << " "
              << v_data[ui_data[ui_data.size() - 1]][2];

    //! Configure Bullet Mesh
    //! This part is very likely to cause segfault, if done incorrectly
    bulletMesh.m_numTriangles = ui_data.size() / 3;
    bulletMesh.m_triangleIndexBase =
        reinterpret_cast<const unsigned char*>(ui_data.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices = v_data.size();
    bulletMesh.m_vertexBase =
        reinterpret_cast<const unsigned char*>(v_data.data());
    bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;
    bSceneArray_->addIndexedMesh(bulletMesh, PHY_INTEGER);  // exact shape

    //! Embed 3D mesh into bullet shape
    //! btBvhTriangleMeshShape is the most generic/slow choice
    bSceneShapes_.emplace_back(
        std::make_unique<btBvhTriangleMeshShape>(bSceneArray_.get(), true));
    double mass = 0.0;
    btVector3 bInertia(0.0, 0.0, 0.0);
    bSceneShapes_.back()->calculateLocalInertia(mass, bInertia);

    //! Bullet rigid body setup
    bSceneCollisionObjects_.emplace_back(std::make_unique<btCollisionObject>());
    bSceneCollisionObjects_.back()->setCollisionShape(
        bSceneShapes_.back().get());
    /*bSceneCollisionObjects_.back()->setCollisionFlags(
        bSceneCollisionObjects_.back()->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT);*/

    bSceneCollisionObjects_.back()->setFriction(
        sceneMetaData.frictionCoefficient_);
    bWorld->addCollisionObject(bSceneCollisionObjects_.back().get());
  }

  LOG(INFO) << "Instance body: initialized";
  syncPose();
  bWorld_ = bWorld;
  initialized_ = true;
  return true;
}  // end BulletRigidObject::initializeScene

bool BulletRigidObject::initializeObject(
    assets::PhysicsObjectMetaData& metaData,
    std::vector<assets::CollisionMeshData> meshGroup,
    std::shared_ptr<btDiscreteDynamicsWorld> bWorld) {
  // TODO (JH): Handling static/kinematic object type
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isScene_) {
    return false;
  }
  isObject_ = true;
  objectMotionType_ = DYNAMIC;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Physical parameters
  double margin = metaData.margin_;

  //! Iterate through all mesh components for one object
  //! The components are combined into a convex compound shape
  bObjectShape_ = std::make_unique<btCompoundShape>();
  for (assets::CollisionMeshData& meshData : meshGroup) {
    Corrade::Containers::ArrayView<Magnum::Vector3> v_data = meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data =
        meshData.indices;
    LOG(INFO) << "Object mesh indices count " << ui_data.size();

    //! Configure Bullet Mesh
    //! This part is very likely to cause segfault, if done incorrectly
    bulletMesh.m_numTriangles = ui_data.size() / 3;
    bulletMesh.m_triangleIndexBase =
        reinterpret_cast<const unsigned char*>(ui_data.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices = v_data.size();
    //! Get the pointer to the first float of the first triangle
    bulletMesh.m_vertexBase =
        reinterpret_cast<const unsigned char*>(v_data.data()->data());
    bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;

    //! Check dimension of the data
    // float dx, dy, dz;
    // getDimensions(meshData, &dx, &dy, &dz);
    // LOG(INFO) << "Dimensions dx " << dx << " dy " << dy << " dz " << dz;

    btTransform t;  // position and rotation
    t.setIdentity();
    t.setOrigin(btVector3(0, 0, 0));
    // t.setOrigin(btVector3(-metaData.COM[0], -metaData.COM[1],
    // -metaData.COM[2]));
    //! TODO (JH): assume that the object is convex, otherwise game over
    //! Create convex component
    bObjectConvexShapes_.emplace_back(std::make_unique<btConvexHullShape>(
        static_cast<const btScalar*>(meshData.positions.data()->data()),
        meshData.positions.size(), sizeof(Magnum::Vector3)));
    bObjectConvexShapes_.back()->setMargin(margin);
    //! Add to compound shape stucture
    bObjectShape_->addChildShape(t, bObjectConvexShapes_.back().get());
  }
  //! Set properties
  bObjectShape_->setMargin(margin);

  btVector3 bInertia = btVector3(metaData.inertia_);

  if (bInertia[0] == 0. && bInertia[1] == 0. && bInertia[2] == 0.) {
    // Alex TODO: allow bullet to compute the inertia tensor if we don't have
    // one
    bObjectShape_->calculateLocalInertia(metaData.mass_,
                                         bInertia);  // overrides bInertia
    LOG(INFO) << "Automatic object inertia computed: " << bInertia.x() << " "
              << bInertia.y() << " " << bInertia.z();
  } else {
    LOG(INFO) << "User provided object inertia " << bInertia.x() << " "
              << bInertia.y() << " " << bInertia.z();
  }

  //! Bullet rigid body setup
  bObjectMotionState_ = new Magnum::BulletIntegration::MotionState(*this);
  btRigidBody::btRigidBodyConstructionInfo info =
      btRigidBody::btRigidBodyConstructionInfo(
          metaData.mass_, &(bObjectMotionState_->btMotionState()),
          bObjectShape_.get(), bInertia);
  info.m_friction = metaData.frictionCoefficient_;
  info.m_restitution = metaData.restitutionCoefficient_;
  info.m_linearDamping = metaData.linDamping_;
  info.m_angularDamping = metaData.angDamping_;
  // Magnum::Vector3 inertia = metaData.inertia;
  // info.m_localInertia   = bInertia(inertia.x(), inertia.y(), inertia.z());

  //! Create rigid body
  bObjectRigidBody_ = std::make_unique<btRigidBody>(info);
  LOG(INFO) << "Setting collision mass " << metaData.mass_ << " flags "
            << bObjectRigidBody_->getCollisionFlags();

  //! Add to world
  bWorld->addRigidBody(bObjectRigidBody_.get());
  // LOG(INFO) << "Body Construction test: after";
  // LOG(INFO) << "Rigid body: initialized";

  //! Sync render pose with physics
  syncPose();
  bWorld_ = bWorld;
  initialized_ = true;
  return true;

}  // end BulletRigidObject::initializeObject

bool BulletRigidObject::removeObject() {
  bWorld_->removeRigidBody(bObjectRigidBody_.get());
  bWorld_ = nullptr;
  initialized_ = false;
  return true;
}

bool BulletRigidObject::isActive() {
  if (!initialized_) {
    LOG(INFO) << "Node not initialized";
    return false;
  }
  if (isScene_) {
    return false;
  }
  if (isObject_) {
    return bObjectRigidBody_->isActive();
  }

  return false;
}

void BulletRigidObject::setActive() {
  if (isScene_ || !initialized_) {
    return;
  }
  //! dynamic_cast is safe
  bObjectRigidBody_->activate(true);
}

void BulletRigidObject::applyForce(Magnum::Vector3 force,
                                   Magnum::Vector3 relPos) {
  setActive();
  bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
}

void BulletRigidObject::applyImpulse(Magnum::Vector3 impulse,
                                     Magnum::Vector3 relPos) {
  if (isScene_ || !initialized_) {
    return;
  }
  setActive();
  bObjectRigidBody_->applyImpulse(btVector3(impulse), btVector3(relPos));
}

//! Synchronize Physics transformations
//! Needed after changing the pose from Magnum side
void BulletRigidObject::syncPose() {
  if (initialized_) {
    if (isScene_) {
      //! You shouldn't need to set scene transforms manually
      //! Scenes are loaded as is
      return;
    } else {
      // LOG(INFO) << "Rigid object sync pose";
      //! For syncing objects
      bObjectRigidBody_->setWorldTransform(btTransform(transformationMatrix()));
    }
  } else {
    LOG(INFO) << "BulletRigidObject::syncPose - Object not initialized";
  }
}

void BulletRigidObject::setMargin(const double margin) {
  if (isScene_) {
    return;
  } else {
    for (int i = 0; i < bObjectConvexShapes_.size(); i++) {
      bObjectConvexShapes_[i]->setMargin(margin);
    }
    bObjectShape_->setMargin(margin);
  }
}

void BulletRigidObject::setMass(const double mass) {}

void BulletRigidObject::setCOM(const Magnum::Vector3d COM) {}

void BulletRigidObject::setInertia(const Magnum::Vector3 inertia) {}

void BulletRigidObject::setScale(const double scale) {}

void BulletRigidObject::setFrictionCoefficient(
    const double frictionCoefficient) {}

void BulletRigidObject::setRestitutionCoeffcient(
    const double restitutionCoeffcient) {}

void BulletRigidObject::setLinearDamping(const double linearDamping) {}

void BulletRigidObject::setAngularDamping(const double angularDamping) {}

const double BulletRigidObject::getMargin() {
  if (isScene_) {
    return -1.0;
  } else {
    return bObjectShape_->getMargin();
  }
}

const double BulletRigidObject::getMass() {
  return 0.0;
}

const Magnum::Vector3d BulletRigidObject::getCOM() {
  return Magnum::Vector3d();
}

const Magnum::Vector3 BulletRigidObject::getInertia() {
  return Magnum::Vector3();
}

const double BulletRigidObject::getScale() {
  return 0.0;
}

const double BulletRigidObject::getFrictionCoefficient() {
  return 0.0;
}

const double BulletRigidObject::getRestitutionCoeffcient() {
  return 0.0;
}

const double BulletRigidObject::getLinearDamping() {
  return 0.0;
}

const double BulletRigidObject::getAngularDamping() {
  return 0.0;
}

}  // namespace physics
}  // namespace esp
