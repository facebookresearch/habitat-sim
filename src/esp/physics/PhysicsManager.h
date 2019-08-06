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
      assets::PhysicsManagerAttributes physicsManagerAttributes);

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;

  //============ Object/Scene Instantiation =============
  //! Initialize scene given mesh data
  //! Only one scene per simulation
  //! The scene could contain several components
  virtual bool addScene(const assets::AssetInfo& info,
                        assets::PhysicsSceneAttributes& physicsSceneAttributes,
                        std::vector<assets::CollisionMeshData> meshGroup);

  //! Initialize object given mesh data
  //! The object could contain several parts
  int addObject(const std::string configFile, DrawableGroup* drawables);
  // calls the above...
  int addObject(const int resObjectID, DrawableGroup* drawables);
  //! Remove added object by physics object ID
  virtual int removeObject(const int physObjectID);

  //============ Simulator functions =============
  virtual void stepPhysics(double dt = -1.0);

  // =========== Global Setter functions ===========
  virtual void setTimestep(double dt);
  virtual void setGravity(Magnum::Vector3 gravity);

  // =========== Global Getter functions ===========
  virtual const double getTimestep() { return fixedTimeStep_; };
  virtual const double getWorldTime() { return worldTime_; };
  virtual const Magnum::Vector3 getGravity();

  // =========== Scene Getter/Setter functions ===========
  virtual double getSceneFrictionCoefficient() { return 0.0; };
  virtual void setSceneFrictionCoefficient(const double frictionCoefficient){};

  //============ Object Transformation functions =============
  void setTransformation(const int physObjectID,
                         const Magnum::Math::Matrix4<float> trans);
  void setTranslation(const int physObjectID,
                      const Magnum::Math::Vector3<float> vector);
  void setRotation(const int physObjectID,
                   const Magnum::Math::Quaternion<float>& quaternion);
  void resetTransformation(const int physObjectID);
  void translate(const int physObjectID,
                 const Magnum::Math::Vector3<float> vector);
  void translateLocal(const int physObjectID,
                      const Magnum::Math::Vector3<float> vector);
  void rotate(const int physObjectID,
              const Magnum::Math::Rad<float> angleInRad,
              const Magnum::Math::Vector3<float> normalizedAxis);
  void rotateX(const int physObjectID,
               const Magnum::Math::Rad<float> angleInRad);
  void rotateY(const int physObjectID,
               const Magnum::Math::Rad<float> angleInRad);
  void rotateZ(const int physObjectID,
               const Magnum::Math::Rad<float> angleInRad);
  void rotateXLocal(const int physObjectID,
                    const Magnum::Math::Rad<float> angleInRad);
  void rotateYLocal(const int physObjectID,
                    const Magnum::Math::Rad<float> angleInRad);
  void rotateZLocal(const int physObjectID,
                    const Magnum::Math::Rad<float> angleInRad);

  //============ Object Setter functions =============
  void setMass(const int physObjectID, const double mass);
  void setCOM(const int physObjectID, const Magnum::Vector3 COM);
  void setInertia(const int physObjectID, const Magnum::Vector3 inertia);
  void setScale(const int physObjectID, const double scale);
  void setFrictionCoefficient(const int physObjectID,
                              const double frictionCoefficient);
  void setRestitutionCoefficient(const int physObjectID,
                                 const double restitutionCoefficient);
  void setLinearDamping(const int physObjectID, const double linDamping);
  void setAngularDamping(const int physObjectID, const double angDamping);

  //============ Object Getter functions =============
  const double getMass(const int physObjectID);
  const Magnum::Vector3 getCOM(const int physObjectID);
  const Magnum::Vector3 getInertia(const int physObjectID);
  const double getScale(const int physObjectID);
  const double getFrictionCoefficient(const int physObjectID);
  const double getRestitutionCoefficient(const int physObjectID);
  const double getLinearDamping(const int physObjectID);
  const double getAngularDamping(const int physObjectID);

  //============= Platform dependent function =============
  virtual const double getMargin(const int physObjectID) { return 0.0; };
  virtual void setMargin(const int physObjectID, const double margin){};

  // =========== Debug functions ===========
  int checkActiveObjects();

  //============ Interact with objects =============
  // Alex NOTE: engine specifics handled by objects themselves...
  void applyForce(const int physObjectID,
                  Magnum::Vector3 force,
                  Magnum::Vector3 relPos);

  void applyImpulse(const int physObjectID,
                    Magnum::Vector3 impulse,
                    Magnum::Vector3 relPos);
  /*
  Magnum::SceneGraph::DrawableGroup3D& getDrawables() { return debugDrawables; }
  const Magnum::SceneGraph::DrawableGroup3D& getDrawables() const {
    return debugDrawables;
  }
  */

 protected:
  //! Check if mesh primitive type is valid for bullet physics engine
  virtual bool isMeshPrimitiveValid(assets::CollisionMeshData& meshData);

  // acquires an ID from either the recycledObjectIDs_ or by incrementing
  // nextObjectID_
  int allocateObjectID();

  // recycle an physObjectID
  int deallocateObjectID(int physObjectID);

  //! Create and initialize rigid object
  virtual int makeRigidObject(
      std::vector<assets::CollisionMeshData> meshGroup,
      assets::PhysicsObjectAttributes physicsObjectAttributes);

  // use this to instantiate physics objects from the physicsObjectLibrary_
  assets::ResourceManager* resourceManager_;

  //! ==== physics engines ====
  enum PhysicsSimulationLibrary { NONE, BULLET };
  PhysicsSimulationLibrary activePhysSimLib_ = NONE;  // default

  //! Used to keep track of all sceneNodes that have physical properties
  scene::SceneNode* physicsNode_ = nullptr;
  std::shared_ptr<physics::RigidObject> sceneNode_ = nullptr;

  //! ==== dynamic object resources ===
  std::map<int, std::shared_ptr<physics::RigidObject>> existingObjects_;
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
