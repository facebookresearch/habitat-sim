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

BulletRigidObject::~BulletRigidObject() {}

bool BulletRigidObject::initializeScene(
    const assets::PhysicsSceneAttributes& physicsSceneAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    std::shared_ptr<btDiscreteDynamicsWorld> bWorld) {
  if (rigidObjectType_ != RigidObjectType::NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = RigidObjectType::SCENE;
  objectMotionType_ = MotionType::STATIC;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Iterate through all mesh components for one scene
  //! All components are registered as static objects
  bSceneArray_ = std::make_unique<btTriangleIndexVertexArray>();
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    //! Here we convert Magnum's unsigned int indices to
    //! signed indices in bullet. Assuming that it's save to
    //! cast uint to int
    Corrade::Containers::ArrayView<Magnum::Vector3> v_data = meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data =
        meshData.indices;

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
    // double mass = 0.0;
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

  bWorld_ = bWorld;
  syncPose();
  return true;
}  // end BulletRigidObject::initializeScene

bool BulletRigidObject::initializeObject(
    const assets::PhysicsObjectAttributes& physicsObjectAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    std::shared_ptr<btDiscreteDynamicsWorld> bWorld) {
  // TODO (JH): Handling static/kinematic object type
  if (rigidObjectType_ != RigidObjectType::NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = RigidObjectType::OBJECT;
  objectMotionType_ = MotionType::DYNAMIC;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Physical parameters
  double margin = physicsObjectAttributes.getDouble("margin");

  //! Iterate through all mesh components for one object
  //! The components are combined into a convex compound shape
  bObjectShape_ = std::make_unique<btCompoundShape>();
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    Corrade::Containers::ArrayView<Magnum::Vector3> v_data = meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data =
        meshData.indices;

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

    btTransform t;  // position and rotation
    t.setIdentity();
    t.setOrigin(btVector3(0, 0, 0));
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
    // allow bullet to compute the inertia tensor if we don't have one
    bObjectShape_->calculateLocalInertia(
        physicsObjectAttributes.getDouble("mass"),
        bInertia);  // overrides bInertia
    LOG(INFO) << "Automatic object inertia computed: " << bInertia.x() << " "
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
  //! Add to world
  bWorld->addRigidBody(bObjectRigidBody_.get());
  //! Sync render pose with physics
  bWorld_ = bWorld;
  syncPose();
  return true;
}  // end BulletRigidObject::initializeObject

bool BulletRigidObject::removeObject() {
  if (rigidObjectType_ == RigidObjectType::OBJECT) {
    // remove rigid body from the world
    bWorld_->removeRigidBody(bObjectRigidBody_.get());
  } else if (rigidObjectType_ == RigidObjectType::SCENE) {
    // remove collision objects from the world
    for (auto& co : bSceneCollisionObjects_) {
      bWorld_->removeCollisionObject(co.get());
    }
  }
  bWorld_.reset();  // release shared ownership of the world
  rigidObjectType_ = RigidObjectType::NONE;
  return true;
}

bool BulletRigidObject::isActive() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return false;
  } else if (rigidObjectType_ == RigidObjectType::OBJECT) {
    return bObjectRigidBody_->isActive();
  } else {
    return false;
  }
}

void BulletRigidObject::setActive() {
  if (rigidObjectType_ == RigidObjectType::OBJECT) {
    bObjectRigidBody_->activate(true);
  }
}

bool BulletRigidObject::setMotionType(MotionType mt) {
  if (mt == objectMotionType_) {
    return true;  // no work
  }

  if (rigidObjectType_ == RigidObjectType::OBJECT) {
    if (mt == MotionType::KINEMATIC) {
      bWorld_->removeRigidBody(bObjectRigidBody_.get());
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() |
          btCollisionObject::CF_KINEMATIC_OBJECT);
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() &
          ~btCollisionObject::CF_STATIC_OBJECT);
      objectMotionType_ = MotionType::KINEMATIC;
      bWorld_->addRigidBody(bObjectRigidBody_.get());
      return true;
    } else if (mt == MotionType::STATIC) {
      bWorld_->removeRigidBody(bObjectRigidBody_.get());
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() |
          btCollisionObject::CF_STATIC_OBJECT);
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() &
          ~btCollisionObject::CF_KINEMATIC_OBJECT);
      objectMotionType_ = MotionType::STATIC;
      bWorld_->addRigidBody(bObjectRigidBody_.get());
      return true;
    } else if (mt == MotionType::DYNAMIC) {
      bWorld_->removeRigidBody(bObjectRigidBody_.get());
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() &
          ~btCollisionObject::CF_STATIC_OBJECT);
      bObjectRigidBody_->setCollisionFlags(
          bObjectRigidBody_->getCollisionFlags() &
          ~btCollisionObject::CF_KINEMATIC_OBJECT);
      objectMotionType_ = MotionType::DYNAMIC;
      bWorld_->addRigidBody(bObjectRigidBody_.get());
      return true;
    }
    return false;
  }
  return false;
}

void BulletRigidObject::applyForce(const Magnum::Vector3& force,
                                   const Magnum::Vector3& relPos) {
  if (rigidObjectType_ == RigidObjectType::OBJECT &&
      objectMotionType_ == MotionType::DYNAMIC) {
    setActive();
    bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
  }
}

void BulletRigidObject::applyImpulse(const Magnum::Vector3& impulse,
                                     const Magnum::Vector3& relPos) {
  if (rigidObjectType_ == RigidObjectType::OBJECT &&
      objectMotionType_ == MotionType::DYNAMIC) {
    setActive();
    bObjectRigidBody_->applyImpulse(btVector3(impulse), btVector3(relPos));
  }
}

//! Torque interaction
void BulletRigidObject::applyTorque(const Magnum::Vector3& torque) {
  if (rigidObjectType_ == RigidObjectType::OBJECT &&
      objectMotionType_ == MotionType::DYNAMIC) {
    setActive();
    bObjectRigidBody_->applyTorque(btVector3(torque));
  }
}

// Impulse Torque interaction
void BulletRigidObject::applyImpulseTorque(const Magnum::Vector3& impulse) {
  if (rigidObjectType_ == RigidObjectType::OBJECT &&
      objectMotionType_ == MotionType::DYNAMIC) {
    setActive();
    bObjectRigidBody_->applyTorqueImpulse(btVector3(impulse));
  }
}

//! Synchronize Physics transformations
//! Needed after changing the pose from Magnum side
void BulletRigidObject::syncPose() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    //! You shouldn't need to set scene transforms manually
    //! Scenes are loaded as is
    return;
  } else if (rigidObjectType_ == RigidObjectType::OBJECT) {
    //! For syncing objects
    bObjectRigidBody_->setWorldTransform(btTransform(transformationMatrix()));
  }
}

void BulletRigidObject::setMargin(const double margin) {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return;
  } else {
    for (std::size_t i = 0; i < bObjectConvexShapes_.size(); i++) {
      bObjectConvexShapes_[i]->setMargin(margin);
    }
    bObjectShape_->setMargin(margin);
  }
}

void BulletRigidObject::setMass(const double mass) {
  if (rigidObjectType_ == RigidObjectType::SCENE)
    return;
  else
    bObjectRigidBody_->setMassProps(mass, btVector3(getInertiaVector()));
}

void BulletRigidObject::setCOM(const Magnum::Vector3&) {
  // Current not supported
  /*if (rigidObjectType_ == RigidObjectType::SCENE)
    return;
  else
    bObjectRigidBody_->setCenterOfMassTransform(
        btTransform(Magnum::Matrix4<float>::translation(COM)));*/
}

void BulletRigidObject::setInertiaVector(const Magnum::Vector3& inertia) {
  if (rigidObjectType_ == RigidObjectType::SCENE)
    return;
  else
    bObjectRigidBody_->setMassProps(getMass(), btVector3(inertia));
}

void BulletRigidObject::setScale(const double) {
  // Currently not supported
  /*if (rigidObjectType_ == RigidObjectType::SCENE)
    return;
  else
    bObjectRigidBody_->setLinearFactor(btVector3(scale, scale, scale));*/
}

void BulletRigidObject::setFrictionCoefficient(
    const double frictionCoefficient) {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    for (std::size_t i = 0; i < bSceneCollisionObjects_.size(); i++) {
      bSceneCollisionObjects_[i]->setFriction(frictionCoefficient);
    }
  } else {
    bObjectRigidBody_->setFriction(frictionCoefficient);
  }
}

void BulletRigidObject::setRestitutionCoefficient(
    const double restitutionCoefficient) {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    for (std::size_t i = 0; i < bSceneCollisionObjects_.size(); i++) {
      bSceneCollisionObjects_[i]->setRestitution(restitutionCoefficient);
    }
  } else {
    bObjectRigidBody_->setRestitution(restitutionCoefficient);
  }
}

void BulletRigidObject::setLinearDamping(const double linearDamping) {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return;
  } else {
    bObjectRigidBody_->setDamping(linearDamping, getAngularDamping());
  }
}

void BulletRigidObject::setAngularDamping(const double angularDamping) {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return;
  } else {
    bObjectRigidBody_->setDamping(getLinearDamping(), angularDamping);
  }
}

double BulletRigidObject::getMargin() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return 0.0;
  } else {
    return bObjectShape_->getMargin();
  }
}

double BulletRigidObject::getMass() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return 0.0;
  } else {
    return 1.0 / bObjectRigidBody_->getInvMass();
  }
}

Magnum::Vector3 BulletRigidObject::getCOM() {
  // TODO: double check the position if there is any implicit transformation
  // done
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    const Magnum::Vector3 com = Magnum::Vector3();
    return com;
  } else {
    const Magnum::Vector3 com =
        Magnum::Vector3(bObjectRigidBody_->getCenterOfMassPosition());
    return com;
  }
}

Magnum::Vector3 BulletRigidObject::getInertiaVector() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    const Magnum::Vector3 inertia = Magnum::Vector3();
    return inertia;
  } else {
    const Magnum::Vector3 inertia =
        1.0 / Magnum::Vector3(bObjectRigidBody_->getInvInertiaDiagLocal());
    return inertia;
  }
}

Magnum::Matrix3 BulletRigidObject::getInertiaMatrix() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    const Magnum::Matrix3 inertia = Magnum::Matrix3();
    return inertia;
  } else {
    const Magnum::Vector3 vecInertia = getInertiaVector();
    const Magnum::Matrix3 inertia = Magnum::Matrix3::fromDiagonal(vecInertia);
    return inertia;
  }
}

double BulletRigidObject::getScale() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return 1.0;
    // Assume uniform scale for 3D objects
  } else {
    return bObjectRigidBody_->getLinearFactor().x();
  }
}

double BulletRigidObject::getFrictionCoefficient() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
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

double BulletRigidObject::getRestitutionCoefficient() {
  // Assume uniform restitution in scene parts
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    if (bSceneCollisionObjects_.size() == 0) {
      return 0.0;
    } else {
      return bSceneCollisionObjects_.back()->getRestitution();
    }
  } else {
    return bObjectRigidBody_->getRestitution();
  }
}

double BulletRigidObject::getLinearDamping() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return 0.0;
  } else {
    return bObjectRigidBody_->getLinearDamping();
  }
}

double BulletRigidObject::getAngularDamping() {
  if (rigidObjectType_ == RigidObjectType::SCENE) {
    return 0.0;
  } else {
    return bObjectRigidBody_->getAngularDamping();
  }
}

}  // namespace physics
}  // namespace esp
