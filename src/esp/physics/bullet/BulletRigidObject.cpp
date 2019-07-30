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
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isObject_) {
    return false;
  }
  isScene_ = true;
  objectMotionType = STATIC;

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

    bWorld.addCollisionObject(bSceneCollisionObjects_.back().get());
  }

  LOG(INFO) << "Instance body: initialized";
  syncPose();
  initialized_ = true;
  return true;
}  // end BulletRigidObject::initializeScene

bool BulletRigidObject::initializeObject(
    assets::PhysicsObjectMetaData& metaData,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {
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
  objectMotionType = DYNAMIC;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Physical parameters
  // LOG(INFO) << "Creating object mass: " << metaData.mass;
  // float restitution = metaData.restitutionCoefficient;
  float margin = metaData.margin;
  // float linDamping  = metaData.linDamping;
  // float angDamping  = metaData.angDamping;

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

  btVector3 bInertia = btVector3(metaData.inertia);

  if (bInertia[0] == 0. && bInertia[1] == 0. && bInertia[2] == 0.) {
    // Alex TODO: allow bullet to compute the inertia tensor if we don't have
    // one
    bObjectShape_->calculateLocalInertia(metaData.mass,
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
          metaData.mass, &(bObjectMotionState_->btMotionState()),
          bObjectShape_.get(), bInertia);
  info.m_friction = metaData.frictionCoefficient;
  info.m_restitution = metaData.restitutionCoefficient;
  info.m_linearDamping = metaData.linDamping;
  info.m_angularDamping = metaData.angDamping;
  // Magnum::Vector3 inertia = metaData.inertia;
  // info.m_localInertia   = bInertia(inertia.x(), inertia.y(), inertia.z());

  //! Create rigid body
  bObjectRigidBody_ = std::make_unique<btRigidBody>(info);
  LOG(INFO) << "Setting collision mass " << metaData.mass << " flags "
            << bObjectRigidBody_->getCollisionFlags();

  //! Add to world
  bWorld.addRigidBody(bObjectRigidBody_.get());
  // LOG(INFO) << "Body Construction test: after";
  // LOG(INFO) << "Rigid body: initialized";

  //! Sync render pose with physics
  syncPose();
  initialized_ = true;
  return true;

}  // end BulletRigidObject::initializeObject

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

void BulletRigidObject::applyForce(Magnum::Vector3 force,
                                   Magnum::Vector3 relPos) {
  if (isScene_ || !initialized_) {
    return;
  }
  //! dynamic_cast is safe
  bObjectRigidBody_->activate();
  bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
}

void BulletRigidObject::applyImpulse(Magnum::Vector3 impulse,
                                     Magnum::Vector3 relPos) {
  if (isScene_ || !initialized_) {
    return;
  }
  bObjectRigidBody_->activate();
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

}  // namespace physics
}  // namespace esp
