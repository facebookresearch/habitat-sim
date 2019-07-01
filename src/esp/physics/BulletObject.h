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
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/core/esp.h"

namespace esp {
namespace physics {

class BulletRigidObject : public scene::SceneNode {
 public:
  BulletRigidObject(scene::SceneNode* parent);

  // TODO (JH) Currently a BulletRigidObject is either a scene
  // or an object, but cannot be both (tracked by _initialized)
  // there is probably a better way to abstract this
  bool initializeScene(
      const assets::AssetInfo& info,
      Magnum::Float mass,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  bool initializeObject(
      const assets::AssetInfo& info,
      Magnum::Float mass,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  ~BulletRigidObject();
  btRigidBody& rigidBody();
  /* needed after changing the pose from Magnum side */
  void syncPose();

  bool isActive();

 private:
  bool _initialized = false;
  btDynamicsWorld* _bWorld;
  // Magnum::Containers::Pointer<btRigidBody> _bRigidBody;
  //btRigidBody* _bRigidBody;
  btCollisionObject* _bCollisionBody;

  float _mass;
  float _restitution;

  void getDimensions(assets::CollisionMeshData& meshData, 
      float* x, float* y, float* z);
};

}  // namespace physics
}  // namespace esp
