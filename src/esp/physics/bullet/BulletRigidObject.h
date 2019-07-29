// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <btBulletDynamicsCommon.h>
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/MeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/PhysicsObjectMetaData.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/core/esp.h"
#include "esp/scene/SceneNode.h"
#include <Magnum/DebugTools/ForceRenderer.h>

#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

class BulletRigidObject : public RigidObject {
public:
    BulletRigidObject(scene::SceneNode* parent);

    ~BulletRigidObject();

    bool initializeScene(
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

    bool initializeObject(
      assets::PhysicsObjectMetaData& metaData,
      physics::PhysicalObjectType objectType,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

    //! Check whether object is being actively simulated, or sleeping
    bool isActive();

    //! Force interaction
    void applyForce(Magnum::Vector3 force,
                  Magnum::Vector3 relPos);

    // Impulse interaction
    void applyImpulse(Magnum::Vector3 impulse,
                    Magnum::Vector3 relPos);

protected:
    //! Needed after changing the pose from Magnum side
    //! Not exposed to end user
    void syncPose();

private:
    //! Physical scene
    //! Scene data: triangular mesh shape
    //! All components are stored as a vector of bCollisionBody_
    std::unique_ptr<btTriangleIndexVertexArray>          bSceneArray_;
    std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneShapes_;
    std::vector<std::unique_ptr<btCollisionObject>>      bSceneCollisionObjects_;

    // Physical object
    //! Object data: Composite convex collision shape
    //! All components are wrapped into one rigidBody_
    std::vector<std::unique_ptr<btConvexHullShape>>      bObjectConvexShapes_;
    std::unique_ptr<btCompoundShape>                     bObjectShape_;
    std::unique_ptr<btRigidBody>                         bObjectRigidBody_;
    Magnum::BulletIntegration::MotionState*              bObjectMotionState_;

};

}
}