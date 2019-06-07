
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
//#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#include "esp/geo/geo.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneGraph.h"

#include "BulletObject.h"

namespace esp {
namespace physics {

// TODO (JH) what role does Object3D{parent} play in example?
BulletRigidObject::BulletRigidObject(scene::SceneNode* parent): 
  scene::SceneNode{*parent} {}

bool BulletRigidObject::initialize(Magnum::Float mass, 
																	 Magnum::Trade::MeshData3D& meshData,
															     btDynamicsWorld& bWorld){

	_bWorld = &bWorld;
  /* Create Bullet Object */
  /*
  // this is a collision mesh, convert to bullet mesh //
  // TODO (JH) would this introduce memory leak?
  btIndexedMesh bulletMesh;
  LOG(INFO) << "Mesh indices count " << meshData.indices().size();
  
  bulletMesh.m_numTriangles = meshData.indices().size()/3;
  bulletMesh.m_triangleIndexBase = reinterpret_cast<const unsigned char *>(meshData.indices().data());
  bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
  bulletMesh.m_numVertices = meshData.positions(0).size();
  bulletMesh.m_vertexBase = reinterpret_cast<const unsigned char *>(meshData.positions(0).data());
  bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
  bulletMesh.m_indexType = PHY_INTEGER;
  bulletMesh.m_vertexType = PHY_FLOAT;


  btCollisionShape* shape = nullptr;
  auto tivArray = new btTriangleIndexVertexArray();
  tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);
  if(shapeType == "TriangleMeshShape") {
      // exact shape, but worse performance
      shape = new btBvhTriangleMeshShape(tivArray, true);
  } else {
      // convex hull, but better performance
      shape = new btConvexTriangleMeshShape(tivArray, true);
  } // btConvexHullShape can be even more performant */
  btCollisionShape* bShape = nullptr;

  /* this code only works for Triangles only meshes */
  if (meshData.primitive() == Magnum::MeshPrimitive::Triangles) {
    /* create a bullet indexed mesh from our mesh data */
    btIndexedMesh bulletMesh;
    bulletMesh.m_numTriangles = meshData.indices().size()/3;
    bulletMesh.m_triangleIndexBase = (const unsigned char *)meshData.indices().data();
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices = meshData.positions(0).size();
    bulletMesh.m_vertexBase = (const unsigned char *)meshData.positions(0).data();
    bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;

    btTriangleIndexVertexArray *pTriMesh = new btTriangleIndexVertexArray();
    pTriMesh->addIndexedMesh(bulletMesh, PHY_INTEGER);

    bShape = new btBvhTriangleMeshShape(pTriMesh, true, true);

	  LOG(INFO) << "Creating object mass: " << mass;
	  /* Calculate inertia so the object reacts as it should with
	     rotation and everything */
	  btVector3 bInertia(0.0f, 0.0f, 0.0f);
	  //btVector3 bInertia(2.0f, 2.0f, 2.0f);
	  if(mass != 0.0f) bShape->calculateLocalInertia(mass, bInertia);

	  /* Bullet rigid body setup */
	  auto* motionState = new Magnum::BulletIntegration::MotionState{*this};

	  _bRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo{
      mass, &motionState->btMotionState(), bShape, bInertia});

	  LOG(INFO) << "Body Construction test: after";    
	  LOG(INFO) << "Emplace: before " << & _bRigidBody;

	  // _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
	  //     mass, &motionState->btMotionState(), bShape, bInertia});

	  LOG(INFO) << "Emplace: after";

	  _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
	  
	  LOG(INFO) << "Add rigid: before";
	  
	  bWorld.addRigidBody(_bRigidBody);

  } else {
    LOG(ERROR) << "Unsupported mesh primitive type while converting Magnum mesh to Bullet mesh.";
  }

	LOG(INFO) << "Rigid body: initialized";

  _initialized = true;
  return true;
}

BulletRigidObject::~BulletRigidObject() {
	if (_initialized) {
	  _bWorld->removeRigidBody(_bRigidBody);
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


}
}