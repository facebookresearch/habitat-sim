
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
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a BulletRigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isObject_) {return false;}
  isScene_ = true;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Object Physical Parameters
  mass_ = mass;
  LOG(INFO) << "Creating Instance object mass: " << mass;
  LOG(INFO) << "Creating Instance object meshGroups: " << meshGroup.size();
  btVector3 bInertia(0.0f, 0.0f, 0.0f);
  

  //! Iterate through all mesh components for one scene
  //! All components are registered as static objects
  tivArray_ = std::make_unique<btTriangleIndexVertexArray>();
  for (assets::CollisionMeshData& meshData: meshGroup) {    
    //! Here we convert Magnum's unsigned int indices to 
    //! signed indices in bullet. Assuming that it's save to 
    //! cast uint to int  
    Corrade::Containers::ArrayView<Magnum::Vector3>     v_data  = 
        meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data = 
        meshData.indices;
    LOG(INFO) << "Instance Mesh v data count "   << v_data.size();
    LOG(INFO) << "Instance Mesh triangle count " << ui_data.size() / 3;
    LOG(INFO) << "Last mesh face index: " << ui_data[ui_data.size()-1];
    LOG(INFO) << "Last mesh face vertex: " << v_data[ui_data[ui_data.size()-1]][0]
        << " " << v_data[ui_data[ui_data.size()-1]][1] 
        << " " << v_data[ui_data[ui_data.size()-1]][2];

    //! Configure Bullet Mesh
    //! This part is very likely to cause segfault, if done incorrectly
    bulletMesh.m_numTriangles        = ui_data.size() / 3;
    bulletMesh.m_triangleIndexBase   = 
        reinterpret_cast<const unsigned char*>(ui_data.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices         = v_data.size();
    bulletMesh.m_vertexBase          = 
        reinterpret_cast<const unsigned char*>(v_data.data());
    bulletMesh.m_vertexStride        = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType           = PHY_INTEGER;
    bulletMesh.m_vertexType          = PHY_FLOAT;  
    tivArray_->addIndexedMesh(bulletMesh, PHY_INTEGER);   // exact shape
    
    //! Embed 3D mesh into bullet shape
    //! btBvhTriangleMeshShape is the most generic/slow choice
    bSceneShapes_.emplace_back(std::make_unique<btBvhTriangleMeshShape>(
        tivArray_.get(), true));
    bSceneShapes_.back()->calculateLocalInertia(mass, bInertia);

    //! Bullet rigid body setup
    bCollisionBodies_.emplace_back(std::make_unique<btCollisionObject>());
    bCollisionBodies_.back()->setCollisionShape(bSceneShapes_.back().get());
    bCollisionBodies_.back()->setCollisionFlags(
        bCollisionBodies_.back()->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT);
    bWorld.addCollisionObject(bCollisionBodies_.back().get());
  }

  LOG(INFO) << "Instance body: initialized";
  initialized_ = true;
  return true;

}


bool BulletRigidObject::initializeObject(
    const assets::AssetInfo& info,
    Magnum::Float mass,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {

  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a BulletRigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isScene_) {return false;}
  isObject_ = true;

  //! Create Bullet Object
  btIndexedMesh bulletMesh;

  //! Physical parameters
  LOG(INFO) << "Creating object mass: " << mass;
  mass_ = mass;
  float restitution = defaultRestitution_;
  float margin = defaultMargin_;
  float linDamping = defaultLinDamping_;
  float angDamping = defaultAngDamping_;
  btVector3 bInertia(2.0f, 2.0f, 2.0f);

  //! Iterate through all mesh components for one object
  //! The components are combined into a convex compound shape
  bObjectShape_ = std::make_unique<btCompoundShape>();
  for (assets::CollisionMeshData& meshData: meshGroup) { 

    Corrade::Containers::ArrayView<Magnum::Vector3>     v_data  = 
        meshData.positions;
    Corrade::Containers::ArrayView<Magnum::UnsignedInt> ui_data = 
        meshData.indices;
    LOG(INFO) << "Object mesh indices count " << ui_data.size();

    //! Configure Bullet Mesh
    //! This part is very likely to cause segfault, if done incorrectly
    bulletMesh.m_numTriangles        = ui_data.size() / 3;
    bulletMesh.m_triangleIndexBase   =
        reinterpret_cast<const unsigned char*>(ui_data.data());
    bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
    bulletMesh.m_numVertices         = v_data.size();
    //! Get the pointer to the first float of the first triangle
    bulletMesh.m_vertexBase          =
        reinterpret_cast<const unsigned char*>(v_data.data()->data());
    bulletMesh.m_vertexStride        = sizeof(Magnum::Vector3);
    bulletMesh.m_indexType           = PHY_INTEGER;
    bulletMesh.m_vertexType          = PHY_FLOAT;

    //! Check dimension of the data
    float dx, dy, dz;
    getDimensions(meshData, &dx, &dy, &dz);
    LOG(INFO) << "Dimensions dx " << dx << " dy " << dy << " dz " << dz;
    
    //! Cheezit box
    btTransform t;        // position and rotation
    t.setIdentity();
    t.setOrigin(btVector3(0, 0, 0));

    //! TODO (JH): assume that the object is convex, otherwise game over
    bConvexShapes_.emplace_back(std::make_unique<btConvexHullShape>(
        static_cast<const btScalar*>(meshData.positions.data()->data()),
        meshData.positions.size(), sizeof(Magnum::Vector3)));
    bConvexShapes_.back()->setMargin(margin);
    bObjectShape_->addChildShape(t, bConvexShapes_.back().get());
  }

  bObjectShape_->calculateLocalInertia(mass, bInertia);
  
  //! Bullet rigid body setup
  motionState_ = new Magnum::BulletIntegration::MotionState(*this);
  rigidBody_ = std::make_unique<btRigidBody>(
      btRigidBody::btRigidBodyConstructionInfo{mass, 
      &(motionState_->btMotionState()), bObjectShape_.get(), bInertia});
  rigidBody_->setRestitution(restitution);
  rigidBody_->setDamping(linDamping, angDamping);
  LOG(INFO) << "Setting collision mass " << mass << " flags "
            << rigidBody_->getCollisionFlags();

  bWorld.addRigidBody(rigidBody_.get());
  bCollisionBody_ = rigidBody_;

  LOG(INFO) << "Body Construction test: after";
  LOG(INFO) << "Rigid body: initialized";

  initialized_ = true;
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
  if (!initialized_) {
    LOG(INFO) << "Node not initialized";
    return false;
  }
  if (isScene_) {return false;}
  if (isObject_) {return bCollisionBody_->isActive();}
  
  return false;
}

void BulletRigidObject::debugForce(
    Magnum::SceneGraph::DrawableGroup3D& debugDrawables) {
  //! DEBUG draw
  debugRender_ = new Magnum::DebugTools::ForceRenderer3D(
      *this, {0.0f, 0.0f, 0.0f}, debugExternalForce_, "bulletForce", 
      &debugDrawables);
  LOG(INFO) << "Force render" << debugExternalForce_.x();
}

void BulletRigidObject::setDebugForce(
    Magnum::Vector3 force) {
  debugExternalForce_ = force;
}

BulletRigidObject::~BulletRigidObject() {
  if (initialized_) {
    LOG(INFO) << "Deleting object " << mass_;
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

btRigidBody& BulletRigidObject::rigidBody() {
  LOG(INFO) << "Returning rigid body";
  //! TODO (JH) if this is not rigid body, the function is suicidal
  return *dynamic_cast<btRigidBody*>(bCollisionBody_.get());
}

/* needed after changing the pose from Magnum side */
void BulletRigidObject::syncPose() {
  LOG(INFO) << "Rigid object sync pose";
  if (initialized_) {
    bCollisionBody_->setWorldTransform(btTransform(transformationMatrix()));
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

}  // namespace physics
}  // namespace esp
