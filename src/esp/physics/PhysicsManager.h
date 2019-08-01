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
    resourceManager = _resourceManager;
  };

  virtual ~PhysicsManager();

  //============ Initialization =============
  // load physical properties and setup the world
  // do_profile indicates timing for FPS
  virtual bool initPhysics(scene::SceneNode* node,
                           assets::PhysicsSceneMetaData sceneMetaData,
                           bool do_profile = false);

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;

  //============ Object/Scene Instantiation =============
  //! Initialize scene given mesh data
  //! Only one scene per simulation
  //! The scene could contain several components
  virtual bool addScene(const assets::AssetInfo& info,
                        assets::PhysicsSceneMetaData& sceneMetaData,
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

  virtual void setTimestep(double dt);

  virtual void setGravity(const Magnum::Vector3d gravity);

  double getTimestep() { return fixedTimeStep_; };

  double getWorldTime() { return worldTime_; };

  const Magnum::Vector3d getGravity();

  int checkActiveObjects();

  //============ Interact with objects =============
  // Alex NOTE: engine specifics handled by objects themselves...
  void applyForce(const int objectID,
                  Magnum::Vector3 force,
                  Magnum::Vector3 relPos);

  void applyImpulse(const int objectID,
                    Magnum::Vector3 impulse,
                    Magnum::Vector3 relPos);

  //============ Set/Get object states =============
  void setTransformation(const int objectID,
                         const Magnum::Math::Matrix4<float> trans);
  void setTranslation(const int objectID,
                      const Magnum::Math::Vector3<float> vector);
  void setRotation(const int objectID,
                   const Magnum::Math::Quaternion<float>& quaternion);
  void resetTransformation(const int objectID);
  void translate(const int objectID, const Magnum::Math::Vector3<float> vector);
  void translateLocal(const int objectID,
                      const Magnum::Math::Vector3<float> vector);
  void rotate(const int objectID,
              const Magnum::Math::Rad<float> angleInRad,
              const Magnum::Math::Vector3<float> normalizedAxis);
  void rotateX(const int objectID, const Magnum::Math::Rad<float> angleInRad);
  void rotateY(const int objectID, const Magnum::Math::Rad<float> angleInRad);
  void rotateZ(const int objectID, const Magnum::Math::Rad<float> angleInRad);
  void rotateXLocal(const int objectID,
                    const Magnum::Math::Rad<float> angleInRad);
  void rotateYLocal(const int objectID,
                    const Magnum::Math::Rad<float> angleInRad);
  void rotateZLocal(const int objectID,
                    const Magnum::Math::Rad<float> angleInRad);

  /*
  Magnum::SceneGraph::DrawableGroup3D& getDrawables() { return debugDrawables; }
  const Magnum::SceneGraph::DrawableGroup3D& getDrawables() const {
    return debugDrawables;
  }
  */

 protected:
  //! Check if mesh primitive type is valid for bullet physics engine
  virtual bool isMeshPrimitiveValid(assets::CollisionMeshData& meshData);

  //! Create and initialize rigid object
  virtual const int makeRigidObject(
      std::vector<assets::CollisionMeshData> meshGroup,
      assets::PhysicsObjectMetaData objMetaData);

  // use this to instantiate physics objects from the physicsObjectLibrary_
  assets::ResourceManager* resourceManager;

  //! ==== physics engines ====
  enum PhysicsSimulationLibrary { NONE, BULLET };
  PhysicsSimulationLibrary activePhysSimLib_ = NONE;  // default

  //! Used to keep track of all sceneNodes that have physical properties
  scene::SceneNode* physicsNode_ = nullptr;
  std::shared_ptr<physics::RigidObject> sceneNode_ = nullptr;

  //! ==== dynamic object resources ===
  std::vector<std::unique_ptr<physics::RigidObject>> existingObjects_;
  std::map<int, std::string> existingObjNames_;

  //! ==== Rigid object memory management ====

  //! Utilities
  bool initialized_ = false;
  bool do_profile_ = false;
  float total_time_ = 0.0f;
  int total_frames_ = 0;
  int maxSubSteps_ = 10;
  double fixedTimeStep_ = 1.0 / 240.0;
  assets::PhysicsSceneMetaData sceneMetaData_;

  double worldTime_ = 0.0;
};

}  // namespace physics

}  // namespace esp
