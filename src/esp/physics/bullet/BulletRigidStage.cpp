// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>

#include <utility>

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollisionHelper.h"
#include "BulletRigidStage.h"
#include "esp/assets/ResourceManager.h"
#include "esp/physics/CollisionGroupHelper.h"

namespace esp {
namespace physics {

BulletRigidStage::BulletRigidStage(
    scene::SceneNode* rigidBodyNode,
    const assets::ResourceManager& resMgr,
    std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
    std::shared_ptr<std::map<const btCollisionObject*, int> >
        collisionObjToObjIds)
    : BulletBase(std::move(bWorld), std::move(collisionObjToObjIds)),
      RigidStage{rigidBodyNode, resMgr} {}

BulletRigidStage::~BulletRigidStage() {
  // remove collision objects from the world
  for (auto& co : bStaticCollisionObjects_) {
    bWorld_->removeRigidBody(co.get());
    collisionObjToObjIds_->erase(co.get());
  }
}
bool BulletRigidStage::initialization_LibSpecific() {
  isCollidable_ = getInitializationAttributes()->getIsCollidable();

  if (isCollidable_) {
    // defer construction until necessary
    constructAndAddCollisionObjects();
  }

  return true;

}  // initialization_LibSpecific

void BulletRigidStage::setCollidable(bool collidable) {
  if (collidable == isCollidable_) {
    // no work
    return;
  }

  isCollidable_ = collidable;
  if (isCollidable_) {
    constructAndAddCollisionObjects();
  } else {
    // remove existing collision objects
    for (auto& object : bStaticCollisionObjects_) {
      bWorld_->removeCollisionObject(object.get());
    }
  }
}

void BulletRigidStage::constructAndAddCollisionObjects() {
  if (bStaticCollisionObjects_.empty()) {
    // construct the objects first time
    const auto collisionAssetHandle =
        initializationAttributes_->getCollisionAssetHandle();

    const std::vector<assets::CollisionMeshData>& meshGroup =
        resMgr_.getCollisionMesh(collisionAssetHandle);

    const assets::MeshMetaData& metaData =
        resMgr_.getMeshMetaData(collisionAssetHandle);

    constructBulletSceneFromMeshes(Magnum::Matrix4{}, meshGroup, metaData.root);

    for (auto& object : bStaticCollisionObjects_) {
      object->setFriction(initializationAttributes_->getFrictionCoefficient());
      object->setRestitution(
          initializationAttributes_->getRestitutionCoefficient());
      collisionObjToObjIds_->emplace(object.get(), objectId_);
    }
  }

  // add the objects to the world
  for (auto& object : bStaticCollisionObjects_) {
    bWorld_->addRigidBody(object.get(), int(CollisionGroup::Static),
                          uint32_t(CollisionGroupHelper::getMaskForGroup(
                              CollisionGroup::Static)));
  }
}

void BulletRigidStage::constructBulletSceneFromMeshes(
    const Magnum::Matrix4& transformFromParentToWorld,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    const assets::MeshTransformNode& node) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  const assets::CollisionMeshData* mesh = nullptr;
  if (node.meshIDLocal != ID_UNDEFINED)
    mesh = &meshGroup[node.meshIDLocal];
  // TODO TriangleStrip and TriangleFan would work
  if (mesh && mesh->primitive != Mn::MeshPrimitive::Triangles) {
    ESP_WARNING() << "Unsupported collision mesh primitive" << mesh->primitive
                  << Mn::Debug::nospace << ", skipping";
    mesh = nullptr;
  }

  if (mesh) {
    // SCENE: create a concave static mesh
    btIndexedMesh bulletMesh;

    Corrade::Containers::ArrayView<Magnum::Vector3> v_data = mesh->positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data = mesh->indices;

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
    std::unique_ptr<btTriangleIndexVertexArray> indexedVertexArray =
        std::make_unique<btTriangleIndexVertexArray>();
    indexedVertexArray->addIndexedMesh(bulletMesh, PHY_INTEGER);  // exact shape

    //! Embed 3D mesh into bullet shape
    //! btBvhTriangleMeshShape is the most generic/slow choice
    //! which allows concavity if the object is static
    std::unique_ptr<btBvhTriangleMeshShape> meshShape =
        std::make_unique<btBvhTriangleMeshShape>(indexedVertexArray.get(),
                                                 true);
    meshShape->setMargin(initializationAttributes_->getMargin());
    meshShape->setLocalScaling(
        btVector3{transformFromLocalToWorld
                      .scaling()});  // scale is a property of the shape

    // re-build the bvh after setting margin
    meshShape->buildOptimizedBvh();
    // mass == 0 to indicate static. See isStaticObject assert below. See also
    // examples/MultiThreadedDemo/CommonRigidBodyMTBase.h
    btVector3 localInertia(0, 0, 0);
    btRigidBody::btRigidBodyConstructionInfo cInfo(
        /*mass*/ 0.0, nullptr, meshShape.get(), localInertia);
    cInfo.m_startWorldTransform =
        btTransform{btMatrix3x3{transformFromLocalToWorld.rotation()},
                    btVector3{transformFromLocalToWorld.translation()}};
    std::unique_ptr<btRigidBody> sceneCollisionObject =
        std::make_unique<btRigidBody>(cInfo);
    CORRADE_INTERNAL_ASSERT(sceneCollisionObject->isStaticObject());
    BulletCollisionHelper::get().mapCollisionObjectTo(
        sceneCollisionObject.get(),
        getCollisionDebugName(bStaticCollisionObjects_.size()));
    bStageArrays_.emplace_back(std::move(indexedVertexArray));
    bStageShapes_.emplace_back(std::move(meshShape));
    bStaticCollisionObjects_.emplace_back(std::move(sceneCollisionObject));
  }

  for (const auto& child : node.children) {
    constructBulletSceneFromMeshes(transformFromLocalToWorld, meshGroup, child);
  }
}  // constructBulletSceneFromMeshes

void BulletRigidStage::setFrictionCoefficient(
    const double frictionCoefficient) {
  for (std::size_t i = 0; i < bStaticCollisionObjects_.size(); ++i) {
    bStaticCollisionObjects_[i]->setFriction(frictionCoefficient);
  }
}

void BulletRigidStage::setRestitutionCoefficient(
    const double restitutionCoefficient) {
  for (std::size_t i = 0; i < bStaticCollisionObjects_.size(); ++i) {
    bStaticCollisionObjects_[i]->setRestitution(restitutionCoefficient);
  }
}

double BulletRigidStage::getFrictionCoefficient() const {
  if (bStaticCollisionObjects_.empty()) {
    return 0.0;
  } else {
    // Assume uniform friction in scene parts
    return static_cast<double>(bStaticCollisionObjects_.back()->getFriction());
  }
}

double BulletRigidStage::getRestitutionCoefficient() const {
  // Assume uniform restitution in scene parts
  if (bStaticCollisionObjects_.empty()) {
    return 0.0;
  } else {
    return static_cast<double>(
        bStaticCollisionObjects_.back()->getRestitution());
  }
}

Magnum::Range3D BulletRigidStage::getCollisionShapeAabb() const {
  Magnum::Range3D combinedAABB;
  // concatenate all component AABBs
  for (const auto& object : bStaticCollisionObjects_) {
    btVector3 localAabbMin, localAabbMax;
    object->getCollisionShape()->getAabb(object->getWorldTransform(),
                                         localAabbMin, localAabbMax);
    if (combinedAABB == Magnum::Range3D{}) {
      // override an empty range instead of combining it
      combinedAABB = Magnum::Range3D{Magnum::Vector3{localAabbMin},
                                     Magnum::Vector3{localAabbMax}};
    } else {
      combinedAABB = Magnum::Math::join(
          combinedAABB, Magnum::Range3D{Magnum::Vector3{localAabbMin},
                                        Magnum::Vector3{localAabbMax}});
    }
  }
  return combinedAABB;
}  // getCollisionShapeAabb

std::string BulletRigidStage::getCollisionDebugName(int subpartId) {
  return "Stage, subpart " + std::to_string(subpartId);
}

}  // namespace physics
}  // namespace esp
