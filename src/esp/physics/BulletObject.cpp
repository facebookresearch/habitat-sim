
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

BulletRigidObject::BulletRigidObject(scene::SceneNode* parent)
    : scene::SceneNode{*parent} {}


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


bool BulletRigidObject::initializeScene(
    const assets::AssetInfo& info,
    Magnum::Float mass,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {
  if (_initialized) {
    LOG(ERROR) << "Cannot initialized a BulletRigidObject more than once";
    return false;
  }

  _bWorld = &bWorld;
  // Create Bullet Object
  btIndexedMesh bulletMesh;
  btCollisionShape* bShape = nullptr;

  // Object Physical Parameters
  _mass = mass;
  LOG(INFO) << "Creating Instance object mass: " << mass;
  LOG(INFO) << "Creating Instance object meshGroups: " << meshGroup.size();
  btVector3 bInertia(0.0f, 0.0f, 0.0f);
  

  // Iterate through all mesh components for one scene
  // All components are registered as static objects
  for (assets::CollisionMeshData& meshData: meshGroup) {    
    //std::vector<Magnum::Vector3>     v_data  = meshData.positions;
    //std::vector<Magnum::UnsignedInt> ui_data = meshData.indices;
    //std::vector<int> i_data;
    
    // Here we convert Magnum's unsigned int indices to 
    // signed indices in bullet. Assuming that it's save to 
    // cast uint to int  
    /*for (uint i = 0; i < ui_data.size(); i++) {
      i_data.push_back(int(ui_data[i]));
    }*/

    LOG(INFO) << "Instance Mesh v data count "   << meshData.positions.size();
    LOG(INFO) << "Instance Mesh triangle count " << meshData.indices.size() / 3;
    LOG(INFO) << "Mesh faces -1 " << meshData.indices[meshData.indices.size()-1];

    // Configure Bullet Mesh
    // This part is very likely to cause segfault, if done incorrectly
    // IMPORTANT: GL::Mesh.count() is not the number of vertices
    bulletMesh.m_numTriangles        = meshData.indices.size() / 3;
    bulletMesh.m_triangleIndexBase   = 
        reinterpret_cast<const unsigned char*>(meshData.indices.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices         = meshData.positions.size();
    // Get the pointer to the first float of the first triangle
    bulletMesh.m_vertexBase          = 
        reinterpret_cast<const unsigned char*>(meshData.positions.data()->data());
    bulletMesh.m_vertexStride        = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType           = PHY_INTEGER;
    bulletMesh.m_vertexType          = PHY_FLOAT;  
    auto tivArray = new btTriangleIndexVertexArray();
    tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);   // exact shape
    
    // Embed 3D mesh into bullet shape
    // btBvhTriangleMeshShape is the most generic/slow choice
    bShape = new btBvhTriangleMeshShape(tivArray, true);
    bShape->calculateLocalInertia(mass, bInertia);

    // Bullet rigid body setup
    _bCollisionBody = new btCollisionObject();
    _bCollisionBody->setCollisionShape(bShape);
    _bCollisionBody->setCollisionFlags(_bCollisionBody->getCollisionFlags() |
                                     btCollisionObject::CF_STATIC_OBJECT);
    _bWorld->addCollisionObject(_bCollisionBody);
  }

  LOG(INFO) << "Instance body: initialized";
  _initialized = true;
  return true;

}


bool BulletRigidObject::initializeObject(
    const assets::AssetInfo& info,
    Magnum::Float mass,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {

  if (_initialized) {
    LOG(ERROR) << "Cannot initialized a BulletRigidObject more than once";
    return false;
  }

  // Create Bullet Object
  _bWorld = &bWorld;
  btCollisionShape* bShape = nullptr;
  btIndexedMesh bulletMesh;

  // Physical parameters
  LOG(INFO) << "Creating object mass: " << mass;
  _mass = mass;
  _restitution = 0.0f;
  float margin = 0.01f;
  btVector3 bInertia(2.0f, 2.0f, 2.0f);

  // Iterate through all mesh components for one object
  // The components are combined into a convex compound shape
  for (assets::CollisionMeshData& meshData: meshGroup) { 

    //std::vector<Magnum::Vector3>     v_data  = meshData.positions;
    //std::vector<Magnum::UnsignedInt> ui_data = meshData.indices;
    //std::vector<vec3i> i_data;
    //LOG(INFO) << "Mesh indices count " << ui_data.size();

    // Configure Bullet Mesh
    // This part is very likely to cause segfault, if done incorrectly
    bulletMesh.m_numTriangles        = meshData.indices.size() / 3;
    bulletMesh.m_triangleIndexBase   =
        reinterpret_cast<const unsigned char*>(meshData.indices.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices         = meshData.positions.size();
    // Get the pointer to the first float of the first triangle
    bulletMesh.m_vertexBase          =
        reinterpret_cast<const unsigned char*>(meshData.positions.data()->data());
    bulletMesh.m_vertexStride        = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType           = PHY_INTEGER;
    bulletMesh.m_vertexType          = PHY_FLOAT;
    //auto tivArray = new btTriangleIndexVertexArray();
    //tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);

    // Check dimension of the data
    float dx, dy, dz;
    getDimensions(meshData, &dx, &dy, &dz);
    LOG(INFO) << "Dimensions dx " << dx << " dy " << dy << " dz " << dz;
    
    // Cheezit box
    btTransform t;        // position and rotation
    t.setIdentity();      
    //t.setOrigin(btVector3(dx/2, dy/2, dz/2));
    t.setOrigin(btVector3(0, 0, 0));
    bShape = new btCompoundShape();

    // TODO (JH): assume that the object is convex, otherwise game over
    btConvexHullShape* b_convex_shape = new btConvexHullShape(
        static_cast<const btScalar*>(meshData.positions.data()->data()),
        meshData.positions.size(), sizeof(Magnum::Vector3));
    b_convex_shape->setMargin(margin);
    (dynamic_cast<btCompoundShape*>(bShape))->addChildShape(t, b_convex_shape);
  }

  bShape->setMargin(margin);
  bShape->calculateLocalInertia(mass, bInertia);
  //bShape = new btBoxShape(btVector3(0.3f,0.3f,0.3f));
  //bShape = new btGImpactMeshShape(tivArray);
  //bShape = new btBvhTriangleMeshShape(tivArray, true);
  
  // Bullet rigid body setup
  auto* motionState = new Magnum::BulletIntegration::MotionState{*this};
  _bCollisionBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo{
      mass, &motionState->btMotionState(), bShape, bInertia});
  _bCollisionBody->setRestitution(_restitution);
  LOG(INFO) << "Setting collision mass " << mass << " flags "
            << _bCollisionBody->getCollisionFlags();

  bWorld.addRigidBody(dynamic_cast<btRigidBody*>(_bCollisionBody));
  //} else {
  // convex hull, but better performance
  // bShape = new btConvexTriangleMeshShape(tivArray, true);
  //} // btConvexHullShape can be even more performant

  LOG(INFO) << "Body Construction test: after";
  LOG(INFO) << "Rigid body: initialized";

  _initialized = true;
  return true;
}

// Helper function to find object center
void BulletRigidObject::getDimensions(
      assets::CollisionMeshData& meshData,
      float* x,
      float* y,
      float* z) {
  float minX = 999999.9f;
  float maxX = -999999.9f;
  float minY = 999999.9f;
  float maxY = -999999.9f;
  float minZ = 999999.9f;
  float maxZ = -999999.9f;
  for (uint vi = 0; vi < meshData.positions.size(); vi++) {
    Magnum::Vector3 pos = meshData.positions[vi];
    if (pos.x() < minX) { minX = pos.x(); }
    if (pos.x() > maxX) { maxX = pos.x(); }
    if (pos.y() < minY) { minY = pos.y(); }
    if (pos.y() > maxY) { maxY = pos.y(); }
    if (pos.z() < minZ) { minZ = pos.z(); }
    if (pos.z() > maxZ) { maxZ = pos.z(); }
  }
  *x = maxX - minX;
  *y = maxY - minY;
  *z = maxZ - minZ;
  LOG(INFO) << "Dimensions minX " << minX << " maxX " << maxX << " minY " 
      << minY << " maxY " << maxY << " minZ " << minZ << " maxZ " << maxZ;
}

bool BulletRigidObject::isActive() {
  if (!_initialized) {
    LOG(INFO) << "Node not initialized";
    return false;
  }
  return _bCollisionBody->isActive();
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
  LOG(INFO) << "Returning rigid body";
  return *dynamic_cast<btRigidBody*>(_bCollisionBody);
}

/* needed after changing the pose from Magnum side */
void BulletRigidObject::syncPose() {
  LOG(INFO) << "Rigid object sync pose";
  if (_initialized) {
    _bCollisionBody->setWorldTransform(btTransform(transformationMatrix()));
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

}  // namespace physics
}  // namespace esp

/*
if (info.type == assets::AssetType::INSTANCE_MESH) {
    LOG(INFO) << "Creating Instance object mass: " << mass;
    btVector3 bInertia(0.0f, 0.0f, 0.0f);
    btIndexedMesh bulletMesh;

    Magnum::GL::Mesh* mesh  = &meshData->getRenderingBuffer()->mesh;
    const std::vector<vec3f> v_data = meshData->getVertexBufferObjectCPU();
    const std::vector<vec3ui> i_data_ = meshData->getIndexBufferObjectCPU();
    std::vector<vec3i> i_data;
    for (uint vi = 0; vi < (int)i_data_.size(); vi++) {
      vec3ui iu = i_data_[vi];
      i_data.push_back(vec3i(static_cast<int>(iu.x()), static_cast<int>(iu.y()), 
          static_cast<int>(iu.z())));
    }

    LOG(INFO) << "Mesh vector 0 " << v_data[0];
    LOG(INFO) << "Mesh vector 1 " << v_data[1];

    int scale = 1;
    LOG(INFO) << "Instance Mesh v data count " << v_data.size();
    LOG(INFO) << "Instance Mesh indices count " << mesh->count();
    LOG(INFO) << "Instance Mesh i data count " << i_data.size();
    LOG(INFO) << "Instance Mesh triangle count " << mesh->count() / (3 * scale);
    LOG(INFO) << "Mesh faces -1 " << i_data[mesh->count()/ (3 * scale) -1];
    bulletMesh.m_numTriangles = mesh->count() / (3 * scale);
    bulletMesh.m_triangleIndexBase = 
        reinterpret_cast<const unsigned char*>(i_data.data());
    //bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    //bulletMesh.m_triangleIndexStride = 3 * 
    //    sizeof(Magnum::GL::MeshIndexType::UnsignedInt);
    bulletMesh.m_triangleIndexStride = sizeof(vec3ui); //3 * sizeof(unsigned int);
    // IMPORTANT: mesh->count() is not the number of vertices
    bulletMesh.m_numVertices = v_data.size();//mesh->count();
    //bulletMesh.m_vertexBase = 
    //    reinterpret_cast<const unsigned char*>(v_data.data());
    bulletMesh.m_vertexBase = 
        reinterpret_cast<const unsigned char*>(v_data.data());
    //bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_vertexStride = sizeof(vec3f);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;
    
    auto tivArray = new btTriangleIndexVertexArray();
    tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);
  
    // exact shape, but worse performance
    bShape = new btBvhTriangleMeshShape(tivArray, true);
    bShape->calculateLocalInertia(mass, bInertia);
    // Bullet rigid body setup
    _bCollisionBody = new btCollisionObject();
    _bCollisionBody->setCollisionShape(bShape);

  } 
  else if (info.type == assets::AssetType::FRL_INSTANCE_MESH) 
  {
    _mass = mass;
    btCollisionShape* bShape = nullptr;

    LOG(INFO) << "Creating FRL object mass: " << mass;
    btVector3 bInertia(0.0f, 0.0f, 0.0f);
    btIndexedMesh bulletMesh;

    Magnum::GL::Mesh* mesh  = &meshData->getRenderingBuffer()->mesh;
    Magnum::GL::Buffer* vbo = &meshData->getRenderingBuffer()->vbo;
    Magnum::GL::Buffer* cbo = &meshData->getRenderingBuffer()->cbo;
    Magnum::GL::Buffer* ibo = &meshData->getRenderingBuffer()->ibo;

    //Corrade::Containers::Array<char> v_data = vbo->data();
    const std::vector<vec3f> v_data = meshData->get_vbo();
    const std::vector<int> i_data   = meshData->get_ibo();

    LOG(INFO) << "Mesh vector 0 " << v_data[0];
    LOG(INFO) << "Mesh vector 1 " << v_data[1];

    //Corrade::Containers::Array<char> i_data = ibo->data();
    int scale = 5;

    LOG(INFO) << "FRL Mesh indices count " << mesh->count();
    bulletMesh.m_numTriangles = mesh->count() / (3 * scale);
    bulletMesh.m_triangleIndexBase = 
        reinterpret_cast<const unsigned char*>(i_data.data());
    //bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    //bulletMesh.m_triangleIndexStride = 3 * 
    //    sizeof(Magnum::GL::MeshIndexType::UnsignedInt);
    bulletMesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
    bulletMesh.m_numVertices = v_data.size();
    //bulletMesh.m_vertexBase = 
    //    reinterpret_cast<const unsigned char*>(v_data.data());
    bulletMesh.m_vertexBase = 
        reinterpret_cast<const unsigned char*>(v_data.data());
    //bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
    bulletMesh.m_vertexStride = sizeof(vec3f);
    bulletMesh.m_indexType = PHY_INTEGER;
    bulletMesh.m_vertexType = PHY_FLOAT;
    btCollisionShape* shape = nullptr;
    auto tivArray = new btTriangleIndexVertexArray();
    tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);
    if (mass != 0.0f) {
      return false;
      bShape = new btBoxShape(btVector3(0.13f, 0.13f, 0.13f));  // Cheezit box
      bShape->calculateLocalInertia(mass, bInertia);
    } else {
      // exact shape, but worse performance
      bShape = new btBvhTriangleMeshShape(tivArray, true);
      bShape->calculateLocalInertia(mass, bInertia);
    }
    // Bullet rigid body setup
    auto* motionState = new Magnum::BulletIntegration::MotionState{*this};

    _bCollisionBody = new btCollisionObject();
    _bCollisionBody->setCollisionShape(bShape);

  }
  else 
  {
    LOG(INFO) << "GLB Object Construction";
    bShape = new btBvhTriangleMeshShape(tivArray, true);  // exact shape, but worse performance
    //bShape = new btBoxShape(btVector3(100.0f,0.1f, 100.0f)); // plane shape
    //bShape = new btGImpactMeshShape(tivArray); // Never worked for me 
    bShape->calculateLocalInertia(mass, bInertia);
    // Bullet rigid body setup
    auto* motionState = new Magnum::BulletIntegration::MotionState{*this};

    _bCollisionBody = new btCollisionObject();
    _bCollisionBody->setCollisionShape(bShape);
  }*/
