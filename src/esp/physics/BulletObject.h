#pragma once

#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <btBulletDynamicsCommon.h>
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/core/esp.h"

namespace esp {
namespace physics {

class BulletRigidObject : public scene::SceneNode {
 public:
  BulletRigidObject(scene::SceneNode* parent);

  bool initialize(Magnum::Float mass,
                  Magnum::Trade::MeshData3D& meshData,
                  btDynamicsWorld& bWorld);

  bool initializeFRL(Magnum::Float mass,
                     assets::GenericInstanceMeshData* meshData,
                     btDynamicsWorld& bWorld);

  ~BulletRigidObject();
  btRigidBody& rigidBody();
  /* needed after changing the pose from Magnum side */
  void syncPose();

 private:
  bool _initialized = false;
  btDynamicsWorld* _bWorld;
  // Magnum::Containers::Pointer<btRigidBody> _bRigidBody;
  btRigidBody* _bRigidBody;

  float _mass;

  void getDimensions(Magnum::Trade::MeshData3D& meshData,
                     float* x,
                     float* y,
                     float* z);
};

}  // namespace physics
}  // namespace esp
