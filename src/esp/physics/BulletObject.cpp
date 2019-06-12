
#include <Corrade/Containers/Array.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/PixelFormat.h>
//#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <mutex>

#include "esp/geo/geo.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneGraph.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletObject.h"

namespace esp {
namespace physics {

// 	Bullet Mesh conversion adapted from:
//			https://github.com/mosra/magnum-integration/issues/20
//			https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=11001
BulletRigidObject::BulletRigidObject(scene::SceneNode* parent)
    : scene::SceneNode{*parent} {}

bool BulletRigidObject::initialize(Magnum::Float mass,
                                   Magnum::Trade::MeshData3D& meshData,
                                   btDynamicsWorld& bWorld) {
  _bWorld = &bWorld;
  /* Create Bullet Object */
  btCollisionShape* bShape = nullptr;

  LOG(INFO) << "Creating object mass: " << mass;
  // Calculate inertia so the object reacts as it should with
  //   rotation and everything
  btVector3 bInertia(0.0f, 0.0f, 0.0f);
  // btVector3 bInertia(2.0f, 2.0f, 2.0f);

  // this code only works for Triangles only meshes
  if (meshData.primitive() == Magnum::MeshPrimitive::Triangles) {
    // this is a collision mesh, convert to bullet mesh //
    btIndexedMesh bulletMesh;
    LOG(INFO) << "Mesh indices count " << meshData.indices().size();

    bulletMesh.m_numTriangles = meshData.indices().size() / 3;
    bulletMesh.m_triangleIndexBase =
        reinterpret_cast<const unsigned char*>(meshData.indices().data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices = meshData.positions(0).size();
    bulletMesh.m_vertexBase =
        reinterpret_cast<const unsigned char*>(meshData.positions(0).data());
    bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;
    auto tivArray = new btTriangleIndexVertexArray();
    tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);

    float dx, dy, dz;
    getDimensions(meshData, &dx, &dy, &dz);
    LOG(INFO) << "Dimensions dx " << dx << " dy " << dy << " dz " << dz;
    if (mass != 0.0f) {
      // TODO (JH) this should be replaced by more general collision-mesh loader
      // bShape = new btBoxShape(btVector3(dx/2, dy/2, dz/2));		//
      // Cheezit box
      btTransform t;  // position and rotation
      t.setIdentity();
      // t.setOrigin(btVector3(dx/2, dy/2, dz/2));
      bShape = new btCompoundShape();
      btConvexHullShape* b_Convex_Shape = new btConvexHullShape(
          (const btScalar*)meshData.positions(0).data(),
          meshData.positions(0).size(), sizeof(Magnum::Vector3));
      (static_cast<btCompoundShape*>(bShape))->addChildShape(t, b_Convex_Shape);
      // bShape = new btBoxShape(btVector3(0.3f,0.3f,0.3f));
      // bShape = new btGImpactMeshShape(tivArray);
      bShape->calculateLocalInertia(mass, bInertia);
    } else {
      // exact shape, but worse performance
      bShape = new btBvhTriangleMeshShape(tivArray, true);
      // bShape = new btBoxShape(btVector3(1.0f,1.0f,1.0f));
      // bShape = new btGImpactMeshShape(tivArray);
      // // Never worked for me bShape = new btBoxShape(btVector3(200.0f, 0.05f,
      // 200.0f));
      bShape->calculateLocalInertia(mass, bInertia);
    }
    //} else {
    // convex hull, but better performance
    // bShape = new btConvexTriangleMeshShape(tivArray, true);
    //} // btConvexHullShape can be even more performant

    // Bullet rigid body setup
    auto* motionState = new Magnum::BulletIntegration::MotionState{*this};

    _bRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo{
        mass, &motionState->btMotionState(), bShape, bInertia});

    if (mass != 0.0f) {
      // TODO (JH) hardcoded for cheezit
      _bRigidBody->setRestitution(20.0f);
    }

    if (mass == 0.0f) {
      LOG(INFO) << "Setting collision mass " << mass << " flags "
                << _bRigidBody->getCollisionFlags();
      _bRigidBody->setCollisionFlags(_bRigidBody->getCollisionFlags() |
                                     btCollisionObject::CF_STATIC_OBJECT);
    } else {
      LOG(INFO) << "Setting collision mass " << mass << " flags "
                << _bRigidBody->getCollisionFlags();
    }

    LOG(INFO) << "Body Construction test: after";
    _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
    _bRigidBody->activate(true);
    LOG(INFO) << "Add rigid: before";
    bWorld.addRigidBody(_bRigidBody);

  } else {
    LOG(ERROR) << "Unsupported mesh primitive type while converting Magnum "
                  "mesh to Bullet mesh.";
  }

  LOG(INFO) << "Rigid body: initialized";

  _initialized = true;
  return true;
}

// Helper function to find object center
void BulletRigidObject::getDimensions(Magnum::Trade::MeshData3D& meshData,
                                      float* x,
                                      float* y,
                                      float* z) {
  float minX = 999999.9f;
  float maxX = -999999.9f;
  float minY = 999999.9f;
  float maxY = -999999.9f;
  float minZ = 999999.9f;
  float maxZ = -999999.9f;
  for (int vi = 0; vi < meshData.positions(0).size(); vi++) {
    Magnum::Vector3 pos = meshData.positions(0)[vi];
    if (pos.x() < minX) {
      minX = pos.x();
    }
    if (pos.x() > maxX) {
      maxX = pos.x();
    }
    if (pos.y() < minY) {
      minY = pos.y();
    }
    if (pos.y() > maxY) {
      maxY = pos.y();
    }
    if (pos.z() < minZ) {
      minZ = pos.z();
    }
    if (pos.z() > maxZ) {
      maxZ = pos.z();
    }
  }
  *x = maxX - minX;
  *y = maxY - minY;
  *z = maxZ - minZ;
}

bool BulletRigidObject::initializeFRL(Magnum::Float mass,
                                      assets::GenericInstanceMeshData* meshData,
                                      btDynamicsWorld& bWorld) {
  _bWorld = &bWorld;
  _mass = mass;
  btCollisionShape* bShape = nullptr;

  LOG(INFO) << "Creating FRL object mass: " << mass;
  btVector3 bInertia(0.0f, 0.0f, 0.0f);
  btIndexedMesh bulletMesh;

  Magnum::GL::Mesh* mesh = &meshData->getRenderingBuffer()->mesh;
  Magnum::GL::Buffer* vbo = &meshData->getRenderingBuffer()->vbo;
  Magnum::GL::Buffer* cbo = &meshData->getRenderingBuffer()->cbo;
  Magnum::GL::Buffer* ibo = &meshData->getRenderingBuffer()->ibo;

  Corrade::Containers::Array<char> v_data = vbo->data();
  Corrade::Containers::Array<char> i_data = ibo->data();

  LOG(INFO) << "FRL Mesh indices count " << mesh->count();
  bulletMesh.m_numTriangles = mesh->count() / 3;
  bulletMesh.m_triangleIndexBase = (unsigned char*)(i_data.data());
  bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
  bulletMesh.m_numVertices = mesh->count();
  bulletMesh.m_vertexBase = (unsigned char*)(v_data.data());
  bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
  bulletMesh.m_indexType = PHY_INTEGER;
  bulletMesh.m_vertexType = PHY_FLOAT;
  btCollisionShape* shape = nullptr;
  auto tivArray = new btTriangleIndexVertexArray();
  tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);
  if (mass != 0.0f) {
    bShape = new btBoxShape(btVector3(0.13f, 0.13f, 0.13f));  // Cheezit box
    bShape->calculateLocalInertia(mass, bInertia);
  } else {
    // exact shape, but worse performance
    bShape = new btBvhTriangleMeshShape(tivArray, true);
    bShape->calculateLocalInertia(mass, bInertia);
  }
  // Bullet rigid body setup
  auto* motionState = new Magnum::BulletIntegration::MotionState{*this};
  _bRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo{
      mass, &motionState->btMotionState(), bShape, bInertia});

  if (mass != 0.0f) {
    // TODO (JH) hardcoded for cheezit
    _bRigidBody->setRestitution(20.0f);
  }
  if (mass == 0.0f) {
    LOG(INFO) << "Setting FRL collision mass " << mass << " flags "
              << _bRigidBody->getCollisionFlags();
    _bRigidBody->setCollisionFlags(_bRigidBody->getCollisionFlags() |
                                   btCollisionObject::CF_STATIC_OBJECT);
  } else {
    LOG(INFO) << "Setting FRL  collision mass " << mass << " flags "
              << _bRigidBody->getCollisionFlags();
  }

  LOG(INFO) << "FRL Body Construction test: after";

  _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
  _bRigidBody->activate(true);
  LOG(INFO) << "Add rigid: before";

  bWorld.addRigidBody(_bRigidBody);
  LOG(INFO) << "FRL body: initialized";
  _initialized = true;
  return true;
}

BulletRigidObject::~BulletRigidObject() {
  if (_initialized) {
    LOG(INFO) << "Deleting object " << _mass;
    //_bWorld->removeRigidBody(_bRigidBody);
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

btRigidBody& BulletRigidObject::rigidBody() {
  return *_bRigidBody;
}

/* needed after changing the pose from Magnum side */
void BulletRigidObject::syncPose() {
  if (_initialized) {
    _bRigidBody->setWorldTransform(btTransform(transformationMatrix()));
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

}  // namespace physics
}  // namespace esp
