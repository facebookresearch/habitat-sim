// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

/* Bullet Physics Integration */
#include <Magnum/Trade/MeshData3D.h>

#include "RigidObject.h"
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/MeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/scene/SceneNode.h"

namespace esp {

namespace assets {
class ResourceManager;
}

namespace physics {

class PhysicsManager {
 public:
  // explicit PhysicsManager(assets::ResourceManager& _resourceManager) :
  // resourceManager(_resourceManager) {};
  explicit PhysicsManager(assets::ResourceManager* _resourceManager) {
    resourceManager_ = _resourceManager;
  };

  virtual ~PhysicsManager();

  //============ Initialization =============
  // load physical properties and setup the world
  // do_profile indicates timing for FPS
  virtual bool initPhysics(
      scene::SceneNode* node,
      const assets::PhysicsManagerAttributes& physicsManagerAttributes);

  virtual void reset(){
      /* TODO: reset object states or clear them? Reset worldTime? Other? */};

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;

  //============ Object/Scene Instantiation =============
  //! Initialize scene given mesh data
  //! Only one scene per simulation
  //! The scene could contain several components
  virtual bool addScene(
      const assets::AssetInfo& info,
      const assets::PhysicsSceneAttributes& physicsSceneAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  //! Initialize object given mesh data
  //! The object could contain several parts
  int addObject(const std::string& configFile, DrawableGroup* drawables);
  // calls the above...
  int addObject(const int resObjectID, DrawableGroup* drawables);
  //! Remove added object by physics object ID
  virtual int removeObject(const int physObjectID);

  // return the number of tracked existingObjects_
  const int getNumRigidObjects() { return existingObjects_.size(); };

  // get/set MotionType
  bool setObjectMotionType(const int physObjectID, MotionType mt);
  const MotionType getObjectMotionType(const int physObjectID);

  //============ Simulator functions =============
  virtual void stepPhysics();
  virtual void stepPhysics(double dt);

  // =========== Global Setter functions ===========
  virtual void setTimestep(double dt);
  virtual void setGravity(const Magnum::Vector3& gravity);

  // =========== Global Getter functions ===========
  virtual const double getTimestep() { return fixedTimeStep_; };
  virtual const double getWorldTime() { return worldTime_; };
  virtual const Magnum::Vector3 getGravity();

  // =========== Scene Getter/Setter functions ===========
  virtual const double getSceneFrictionCoefficient() { return 0.0; };
  virtual void setSceneFrictionCoefficient(const double frictionCoefficient){};
  virtual const double getSceneRestitutionCoefficient() { return 0.0; };
  virtual void setSceneRestitutionCoefficient(
      const double restitutionCoefficient){};

  // ============ Object Transformation functions =============
  void setTransformation(const int physObjectID, const Magnum::Matrix4& trans);
  void setTranslation(const int physObjectID, const Magnum::Vector3& vector);
  void setRotation(const int physObjectID,
                   const Magnum::Quaternion& quaternion);
  void resetTransformation(const int physObjectID);

  void translate(const int physObjectID, const Magnum::Vector3& vector);
  void translateLocal(const int physObjectID, const Magnum::Vector3& vector);

  void rotate(const int physObjectID,
              const Magnum::Rad angleInRad,
              const Magnum::Vector3& normalizedAxis);
  void rotateX(const int physObjectID, const Magnum::Rad angleInRad);
  void rotateY(const int physObjectID, const Magnum::Rad angleInRad);
  void rotateZ(const int physObjectID, const Magnum::Rad angleInRad);
  void rotateXLocal(const int physObjectID, const Magnum::Rad angleInRad);
  void rotateYLocal(const int physObjectID, const Magnum::Rad angleInRad);
  void rotateZLocal(const int physObjectID, const Magnum::Rad angleInRad);

  const Magnum::Matrix4 getTransformation(const int physObjectID);
  const Magnum::Vector3 getTranslation(const int physObjectID);
  const Magnum::Quaternion getRotation(const int physObjectID);

  // ============ Object Setter functions =============
  // Setters that interface with physics need to take
  void setMass(const int physObjectID, const double mass);
  void setCOM(const int physObjectID, const Magnum::Vector3& COM);
  void setInertia(const int physObjectID, const Magnum::Vector3& inertia);
  void setScale(const int physObjectID, const double scale);
  void setFrictionCoefficient(const int physObjectID,
                              const double frictionCoefficient);
  void setRestitutionCoefficient(const int physObjectID,
                                 const double restitutionCoefficient);
  void setLinearDamping(const int physObjectID, const double linDamping);
  void setAngularDamping(const int physObjectID, const double angDamping);

  // ============ Object Getter functions =============
  const double getMass(const int physObjectID);
  const Magnum::Vector3 getCOM(const int physObjectID);
  const Magnum::Vector3 getInertiaVector(const int physObjectID);
  const Magnum::Matrix3 getInertiaMatrix(const int physObjectID);
  const double getScale(const int physObjectID);
  const double getFrictionCoefficient(const int physObjectID);
  const double getRestitutionCoefficient(const int physObjectID);
  const double getLinearDamping(const int physObjectID);
  const double getAngularDamping(const int physObjectID);

  // ============= Platform dependent function =============
  virtual const double getMargin(const int physObjectID) { return 0.0; };
  virtual void setMargin(const int physObjectID, const double margin){};

  // =========== Debug functions ===========
  int checkActiveObjects();

  //============ Interact with objects =============
  // NOTE: engine specifics handled by objects themselves...
  void applyForce(const int physObjectID,
                  const Magnum::Vector3& force,
                  const Magnum::Vector3& relPos);

  void applyImpulse(const int physObjectID,
                    const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos);

  void applyTorque(const int physObjectID, const Magnum::Vector3& torque);

  void applyImpulseTorque(const int physObjectID,
                          const Magnum::Vector3& impulse);

 protected:
  //! Check if mesh primitive type is valid for bullet physics engine
  virtual bool isMeshPrimitiveValid(const assets::CollisionMeshData& meshData);

  // acquires an ID from either the recycledObjectIDs_ or by incrementing
  // nextObjectID_
  int allocateObjectID();

  // recycle an physObjectID
  int deallocateObjectID(int physObjectID);

  //! Create and initialize rigid object
  virtual int makeRigidObject(
      const std::vector<assets::CollisionMeshData>& meshGroup,
      assets::PhysicsObjectAttributes physicsObjectAttributes);

  // use this to instantiate physics objects from the physicsObjectLibrary_
  assets::ResourceManager* resourceManager_;

  //! ==== physics engines ====
  enum PhysicsSimulationLibrary { NONE, BULLET };
  PhysicsSimulationLibrary activePhysSimLib_ = NONE;  // default

  //! Used to keep track of all sceneNodes that have physical properties
  scene::SceneNode* physicsNode_ = nullptr;
  physics::RigidObject* sceneNode_ = nullptr;

  //! ==== dynamic object resources ===
  std::map<int, physics::RigidObject*> existingObjects_;
  int nextObjectID_ = 0;
  std::vector<int>
      recycledObjectIDs_;  // removed object IDs are pushed here and popped
                           // first when constructing new objects.

  //! ==== Rigid object memory management ====

  //! Utilities
  bool initialized_ = false;
  float total_time_ = 0.0f;
  int total_frames_ = 0;
  int maxSubSteps_ = 10;
  double fixedTimeStep_ = 1.0 / 240.0;
  // assets::PhysicsSceneMetaData sceneMetaData_;
  double worldTime_ = 0.0;
};

}  // namespace physics

}  // namespace esp
