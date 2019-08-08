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
    assets::PhysicsSceneAttributes& physicsSceneAttributes,
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
    // btVector3 bInertia(0.0, 0.0, 0.0);
    // bSceneShapes_.back()->calculateLocalInertia(mass, bInertia);

    //! Bullet rigid body setup
    bSceneCollisionObjects_.emplace_back(std::make_unique<btCollisionObject>());
    bSceneCollisionObjects_.back()->setCollisionShape(
        bSceneShapes_.back().get());
    /*bSceneCollisionObjects_.back()->setCollisionFlags(
        bSceneCollisionObjects_.back()->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT);*/

    bSceneCollisionObjects_.back()->setFriction(
        physicsSceneAttributes.getDouble("frictionCoefficient"));
    bSceneCollisionObjects_.back()->setRestitution(
        physicsSceneAttributes.getDouble("restitutionCoefficient"));
    bWorld->addCollisionObject(bSceneCollisionObjects_.back().get());
  }

  LOG(INFO) << "Instance body: initialized";
  bWorld_ = bWorld;
  initialized_ = true;
  syncPose();
  return true;
}  // end BulletRigidObject::initializeScene

bool BulletRigidObject::initializeObject(
    assets::PhysicsObjectAttributes& physicsObjectAttributes,
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
  double margin = physicsObjectAttributes.getDouble("margin");

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

  btVector3 bInertia =
      btVector3(physicsObjectAttributes.getMagnumVec3("inertia"));

  if (bInertia[0] == 0. && bInertia[1] == 0. && bInertia[2] == 0.) {
    // Alex TODO: allow bullet to compute the inertia tensor if we don't have
    // one
    bObjectShape_->calculateLocalInertia(
        physicsObjectAttributes.getDouble("mass"),
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
          physicsObjectAttributes.getDouble("mass"),
          &(bObjectMotionState_->btMotionState()), bObjectShape_.get(),
          bInertia);
  info.m_friction = physicsObjectAttributes.getDouble("frictionCoefficient");
  info.m_restitution =
      physicsObjectAttributes.getDouble("restitutionCoefficient");
  info.m_linearDamping = physicsObjectAttributes.getDouble("linDamping");
  info.m_angularDamping = physicsObjectAttributes.getDouble("angDamping");
  // Magnum::Vector3 inertia = metaData.inertia;
  // info.m_localInertia   = bInertia(inertia.x(), inertia.y(), inertia.z());

  //! Create rigid body
  bObjectRigidBody_ = std::make_unique<btRigidBody>(info);
  LOG(INFO) << "Setting collision mass "
            << physicsObjectAttributes.getDouble("mass") << " flags "
            << bObjectRigidBody_->getCollisionFlags();

  //! Add to world
  bWorld->addRigidBody(bObjectRigidBody_.get());
  //! Sync render pose with physics
  bWorld_ = bWorld;
  initialized_ = true;
  syncPose();
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

void BulletRigidObject::applyForce(Magnum::Vector3& force,
                                   Magnum::Vector3& relPos) {
  setActive();
  bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
}

void BulletRigidObject::applyImpulse(Magnum::Vector3& impulse,
                                     Magnum::Vector3& relPos) {
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

void BulletRigidObject::setMass(const double mass) {
  if (isScene_)
    return;
  else
    bObjectRigidBody_->setMassProps(mass, btVector3(getInertia()));
}

void BulletRigidObject::setCOM(const Magnum::Vector3& COM) {
  // Current not supported
  /*if (isScene_)
    return;
  else
    bObjectRigidBody_->setCenterOfMassTransform(
        btTransform(Magnum::Math::Matrix4<float>::translation(COM)));*/
}

void BulletRigidObject::setInertia(const Magnum::Vector3& inertia) {
  if (isScene_)
    return;
  else
    bObjectRigidBody_->setMassProps(getMass(), btVector3(inertia));
}

void BulletRigidObject::setScale(const double scale) {
  // Currently not supported
  /*if (isScene_)
    return;
  else
    bObjectRigidBody_->setLinearFactor(btVector3(scale, scale, scale));*/
}

void BulletRigidObject::setFrictionCoefficient(
    const double frictionCoefficient) {
  if (isScene_) {
    for (int i = 0; i < bSceneCollisionObjects_.size(); i++) {
      bSceneCollisionObjects_[i]->setFriction(frictionCoefficient);
    }
  } else {
    bObjectRigidBody_->setFriction(frictionCoefficient);
  }
}

void BulletRigidObject::setRestitutionCoefficient(
    const double restitutionCoefficient) {
  if (isScene_) {
    for (int i = 0; i < bSceneCollisionObjects_.size(); i++) {
      bSceneCollisionObjects_[i]->setRestitution(restitutionCoefficient);
    }
  } else {
    bObjectRigidBody_->setRestitution(restitutionCoefficient);
  }
}

void BulletRigidObject::setLinearDamping(const double linearDamping) {
  if (isScene_)
    return;
  else
    bObjectRigidBody_->setDamping(linearDamping, getAngularDamping());
}

void BulletRigidObject::setAngularDamping(const double angularDamping) {
  if (isScene_)
    return;
  else
    bObjectRigidBody_->setDamping(getLinearDamping(), angularDamping);
}

const double BulletRigidObject::getMargin() {
  if (isScene_)
    return -1.0;
  else
    return bObjectShape_->getMargin();
}

const double BulletRigidObject::getMass() {
  if (isScene_)
    return 0.0;
  else
    return 1.0 / bObjectRigidBody_->getInvMass();
}

const Magnum::Vector3& BulletRigidObject::getCOM() {
  // TODO: double check the position if there is any implicit transformation
  // done
  if (isScene_) {
    const Magnum::Vector3 com = Magnum::Vector3();
    return com;
  } else {
    const Magnum::Vector3 com =
        Magnum::Vector3(bObjectRigidBody_->getCenterOfMassPosition());
    return com;
  }
}

const Magnum::Vector3& BulletRigidObject::getInertia() {
  if (isScene_) {
    const Magnum::Vector3 inertia = Magnum::Vector3();
    return inertia;
  } else {
    const Magnum::Vector3 inertia =
        1.0 / Magnum::Vector3(bObjectRigidBody_->getInvInertiaDiagLocal());
    return inertia;
  }
}

const double BulletRigidObject::getScale() {
  if (isScene_)
    return 1.0;
  // Assume uniform scale for 3D objects
  else
    return bObjectRigidBody_->getLinearFactor().x();
}

const double BulletRigidObject::getFrictionCoefficient() {
  if (isScene_) {
    if (bSceneCollisionObjects_.size() == 0) {
      return 0.0;
    } else {
      // Assume uniform friction in scene parts
      return bSceneCollisionObjects_.back()->getFriction();
    }
  } else {
    return bObjectRigidBody_->getFriction();
  }
}

const double BulletRigidObject::getRestitutionCoefficient() {
  // Assume uniform restitution in scene parts
  if (isScene_) {
    if (bSceneCollisionObjects_.size() == 0) {
      return 0.0;
    } else {
      return bSceneCollisionObjects_.back()->getRestitution();
    }
  } else {
    return bObjectRigidBody_->getRestitution();
  }
}

const double BulletRigidObject::getLinearDamping() {
  if (isScene_)
    return 0.0;
  else
    return bObjectRigidBody_->getLinearDamping();
}

const double BulletRigidObject::getAngularDamping() {
  if (isScene_)
    return 0.0;
  else
    return bObjectRigidBody_->getAngularDamping();
}

}  // namespace physics
}  // namespace esp
