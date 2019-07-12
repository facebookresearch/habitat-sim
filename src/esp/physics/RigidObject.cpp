
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
#include "esp/assets/CollisionMeshData.h"

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "RigidObject.h"


namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* parent)
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


bool RigidObject::initializeScene(
    const assets::AssetInfo& info,
    Magnum::Float mass,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
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
  bSceneArray_ = std::make_unique<btTriangleIndexVertexArray>();
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
    bSceneArray_->addIndexedMesh(bulletMesh, PHY_INTEGER);   // exact shape
    
    //! Embed 3D mesh into bullet shape
    //! btBvhTriangleMeshShape is the most generic/slow choice
    bSceneShapes_.emplace_back(std::make_unique<btBvhTriangleMeshShape>(
        bSceneArray_.get(), true));
    bSceneShapes_.back()->calculateLocalInertia(mass, bInertia);

    //! Bullet rigid body setup
    bSceneCollisionObjects_.emplace_back(std::make_unique<btCollisionObject>());
    bSceneCollisionObjects_.back()->setCollisionShape(bSceneShapes_.back().get());
    bSceneCollisionObjects_.back()->setCollisionFlags(
        bSceneCollisionObjects_.back()->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT);
    bWorld.addCollisionObject(bSceneCollisionObjects_.back().get());
  }

  LOG(INFO) << "Instance body: initialized";
  syncPose();
  initialized_ = true;
  return true;
}


bool RigidObject::initializeObject(
    const assets::AssetInfo& info,
    Magnum::Float mass,
    std::vector<assets::CollisionMeshData> meshGroup,
    btDynamicsWorld& bWorld) {

  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
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
    //! Create convex component
    bObjectConvexShapes_.emplace_back(std::make_unique<btConvexHullShape>(
        static_cast<const btScalar*>(meshData.positions.data()->data()),
        meshData.positions.size(), sizeof(Magnum::Vector3)));
    bObjectConvexShapes_.back()->setMargin(margin);

    //! Add to compound shape stucture
    bObjectShape_->addChildShape(t, bObjectConvexShapes_.back().get());
  }

  bObjectShape_->calculateLocalInertia(mass, bInertia);
  
  //! Bullet rigid body setup
  bObjectMotionState_ = new Magnum::BulletIntegration::MotionState(*this);
  bObjectRigidBody_ = std::make_unique<btRigidBody>(
      btRigidBody::btRigidBodyConstructionInfo{mass, 
      &(bObjectMotionState_->btMotionState()), bObjectShape_.get(), bInertia});
  bObjectRigidBody_->setRestitution(restitution);
  bObjectRigidBody_->setDamping(linDamping, angDamping);
  LOG(INFO) << "Setting collision mass " << mass << " flags "
            << bObjectRigidBody_->getCollisionFlags();

  bWorld.addRigidBody(bObjectRigidBody_.get());

  LOG(INFO) << "Body Construction test: after";
  LOG(INFO) << "Rigid body: initialized";

  syncPose();
  initialized_ = true;
  return true;
}

// Helper function to find object center
void RigidObject::getDimensions(
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

bool RigidObject::isActive() {
  if (!initialized_) {
    LOG(INFO) << "Node not initialized";
    return false;
  }
  if (isScene_) {return false;}
  if (isObject_) {return bObjectRigidBody_->isActive();}
  
  return false;
}

void RigidObject::debugForce(
    Magnum::SceneGraph::DrawableGroup3D& debugDrawables) {
  //! DEBUG draw
  debugRender_ = new Magnum::DebugTools::ForceRenderer3D(
      *this, {0.0f, 0.0f, 0.0f}, debugExternalForce_, "bulletForce", 
      &debugDrawables);
  LOG(INFO) << "Force render" << debugExternalForce_.x();
}

void RigidObject::setDebugForce(
    Magnum::Vector3 force) {
  debugExternalForce_ = force;
}

RigidObject::~RigidObject() {
  if (initialized_) {
    LOG(INFO) << "Deleting object " << mass_;
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

void RigidObject::applyForce(Magnum::Vector3 force,
                                   Magnum::Vector3 relPos) {
  if (isScene_ || !initialized_) {return;}
  //! dynamic_cast is safe
  bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
}

void RigidObject::applyImpulse(Magnum::Vector3 impulse,
                                     Magnum::Vector3 relPos) {
  if (isScene_ || !initialized_) {return;}
  bObjectRigidBody_->applyImpulse(btVector3(impulse), btVector3(relPos));
}

//! Synchronize Physics transformations
//! Needed after changing the pose from Magnum side
void RigidObject::syncPose() {
  LOG(INFO) << "Rigid object sync pose";
  if (initialized_) {
    if (isScene_) {
      //! You shouldn't need to set scene transforms manually
      //! Scenes are loaded as is
      return;
    } else {
      //! For syncing objects
      bObjectRigidBody_->setWorldTransform(btTransform(transformationMatrix()));
    }
  } else {
    LOG(INFO) << "Object not initialized";
  }
}


scene::SceneNode& RigidObject::setTransformation(
    const Eigen::Ref<const mat4f> transformation) {
  scene::SceneNode::setTransformation(transformation);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setTransformation(
    const Eigen::Ref<const vec3f> position,
    const Eigen::Ref<const vec3f> target,
    const Eigen::Ref<const vec3f> up) {
  scene::SceneNode::setTransformation(position, target, up);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setTranslation(
    const Eigen::Ref<const vec3f> vector) {
  scene::SceneNode::setTranslation(vector);
  syncPose();
  return *this; 
}

scene::SceneNode& RigidObject::setRotation(
    const quatf& quaternion) {
  scene::SceneNode::setRotation(quaternion);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::resetTransformation() {
  scene::SceneNode::resetTransformation();
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translate(
    const Eigen::Ref<const vec3f> vector) {
  scene::SceneNode::translate(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translateLocal(
    const Eigen::Ref<const vec3f> vector) {
  scene::SceneNode::translateLocal(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotate(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  scene::SceneNode::rotate(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateLocal(
    float angleInRad,
    const Eigen::Ref<const vec3f> normalizedAxis) {
  scene::SceneNode::rotateLocal(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateX(float angleInRad) {
  scene::SceneNode::rotateX(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateXInDegree(float angleInDeg) {
  scene::SceneNode::rotateXInDegree(angleInDeg);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateXLocal(float angleInRad) {
  scene::SceneNode::rotateXLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateY(float angleInRad) {
  scene::SceneNode::rotateY(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateYLocal(float angleInRad) {
  scene::SceneNode::rotateYLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZ(float angleInRad) {
  scene::SceneNode::rotateZ(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZLocal(float angleInRad) {
  scene::SceneNode::rotateZLocal(angleInRad);
  syncPose();
  return *this;
}


}  // namespace physics
}  // namespace esp
