// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/core/random.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"

#include "esp/assets/ResourceManager.h"

#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/WindowlessContext.h"

namespace esp {
namespace nav {
class PathFinder;
class NavMeshSettings;
class ActionSpacePathFinder;
}  // namespace nav
namespace scene {
class SemanticScene;
}  // namespace scene
namespace gfx {
class Renderer;
}  // namespace gfx
namespace physics {
enum class MotionType : int;
}  // namespace physics
}  // namespace esp

namespace esp {
namespace sim {

struct SimulatorConfiguration {
  scene::SceneConfiguration scene;
  int defaultAgentId = 0;
  int gpuDeviceId = 0;
  std::string defaultCameraUuid = "rgba_camera";
  bool compressTextures = false;
  bool createRenderer = true;
  // Whether or not the agent can slide on collisions
  bool allowSliding = true;
  // enable or disable the frustum culling
  bool frustumCulling = true;
  bool enablePhysics = false;
  std::string physicsConfigFile =
      "./data/default.phys_scene_config.json";  // should we instead link a
                                                // PhysicsManagerConfiguration
                                                // object here?

  ESP_SMART_POINTERS(SimulatorConfiguration)
};
bool operator==(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);
bool operator!=(const SimulatorConfiguration& a,
                const SimulatorConfiguration& b);

class Simulator {
 public:
  explicit Simulator(const SimulatorConfiguration& cfg);
  virtual ~Simulator();

  virtual void reconfigure(const SimulatorConfiguration& cfg);

  virtual void reset();

  virtual void seed(uint32_t newSeed);

  std::shared_ptr<gfx::Renderer> getRenderer();
  std::shared_ptr<physics::PhysicsManager> getPhysicsManager();
  std::shared_ptr<scene::SemanticScene> getSemanticScene();

  scene::SceneGraph& getActiveSceneGraph();
  scene::SceneGraph& getActiveSemanticSceneGraph();

  void saveFrame(const std::string& filename);

  /**
   * @brief The ID of the CUDA device of the OpenGL context owned by the
   * simulator.  This will only be nonzero if the simulator is built in
   * --headless mode on linux
   */
  int gpuDevice() const { return context_->gpuDevice(); }

  // === Physics Simulator Functions ===
  // TODO: support multi-scene physics (default sceneID=0 currently).

  /**
   * @brief Instance an object from a template index in @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_. See @ref
   * esp::physics::PhysicsManager::addObject().
   * @param objectLibIndex The index of the object's template in @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_.
   * @param sceneID !! Not used currently !! Specifies which physical scene to
   * add an object to.
   * @return The ID assigned to new object which identifies it in @ref
   * esp::physics::PhysicsManager::existingObjects_ or @ref esp::ID_UNDEFINED if
   * instancing fails.
   */
  int addObject(int objectLibIndex, int sceneID = 0);

  /** @overload */
  int addObject(int objectLibIndex,
                const std::string& lightSetupKey,
                int sceneID = 0);

  /**
   * @brief Get the current size of the physics object library. Objects [0,size)
   * can be instanced with @ref addObject.
   * @return The current number of templates stored in @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_.
   */
  int getPhysicsObjectLibrarySize();

  /**
   * @brief Remove an instanced object by ID. See @ref
   * esp::physics::PhysicsManager::removeObject().
   * @param objectID The ID of the object identifying it in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene to
   * remove the object from.
   */
  void removeObject(const int objectID, const int sceneID = 0);

  /**
   * @brief Get the IDs of the physics objects instanced in a physical scene.
   * See @ref esp::physics::PhysicsManager::getExistingObjectIDs.
   * @param sceneID !! Not used currently !! Specifies which physical scene to
   * query.
   * @return A vector of ID keys into @ref
   * esp::physics::PhysicsManager::existingObjects_.
   */
  std::vector<int> getExistingObjectIDs(const int sceneID = 0);

  /**
   * @brief Get the @ref esp::physics::MotionType of an object.
   * See @ref esp::physics::PhysicsManager::getExistingObjectIDs.
   * @param objectID The ID of the object identifying it in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene to
   * query.
   * @return The @ref esp::physics::MotionType of the object or @ref
   * esp::physics::MotionType::ERROR_MOTIONTYPE if query failed.
   */
  esp::physics::MotionType getObjectMotionType(const int objectID,
                                               const int sceneID = 0);

  /**
   * @brief Set the @ref esp::physics::MotionType of an object.
   * See @ref esp::physics::PhysicsManager::getExistingObjectIDs.
   * @param motionType The desired motion type of the object
   * @param objectID The ID of the object identifying it in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene to
   * query.
   * @return whether or not the set was successful.
   */
  bool setObjectMotionType(const esp::physics::MotionType& motionType,
                           const int objectID,
                           const int sceneID = 0);

  /**
   * @brief Apply torque to an object. See @ref
   * esp::physics::PhysicsManager::applyTorque.
   * @param tau The desired torque to apply.
   * @param objectID The ID of the object identifying it in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   */
  void applyTorque(const Magnum::Vector3& tau,
                   const int objectID,
                   const int sceneID = 0);

  /**
   * @brief Apply force to an object. See @ref
   * esp::physics::PhysicsManager::applyForce.
   * @param force The desired linear force to apply.
   * @param relPos The desired location relative to the object origin at which
   * to apply the force.
   * @param objectID The ID of the object identifying it in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   */
  void applyForce(const Magnum::Vector3& force,
                  const Magnum::Vector3& relPos,
                  const int objectID,
                  const int sceneID = 0);

  /**
   * @brief Get the current 4x4 transformation matrix of an object.
   * See @ref esp::physics::PhysicsManager::getTransformation.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   * @return The 4x4 transform of the object.
   */
  Magnum::Matrix4 getTransformation(const int objectID, const int sceneID = 0);

  /**
   * @brief Set the 4x4 transformation matrix of an object kinematically.
   * See @ref esp::physics::PhysicsManager::setTransformation.
   * @param transform The desired 4x4 transform of the object.
   * @param  objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   */
  void setTransformation(const Magnum::Matrix4& transform,
                         const int objectID,
                         const int sceneID = 0);

  /**
   * @brief Set the 3D position of an object kinematically.
   * See @ref esp::physics::PhysicsManager::setTranslation.
   * @param translation The desired 3D position of the object.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   */
  void setTranslation(const Magnum::Vector3& translation,
                      const int objectID,
                      const int sceneID = 0);

  /**
   * @brief Get the current 3D position of an object.
   * See @ref esp::physics::PhysicsManager::getTranslation.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   * @return The 3D position of the object.
   */
  Magnum::Vector3 getTranslation(const int objectID, const int sceneID = 0);

  /**
   * @brief Set the orientation of an object kinematically.
   * See @ref esp::physics::PhysicsManager::setRotation.
   * @param rotation The desired orientation of the object.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   */
  void setRotation(const Magnum::Quaternion& rotation,
                   const int objectID,
                   const int sceneID = 0);

  /**
   * @brief Get the current orientation of an object.
   * See @ref esp::physics::PhysicsManager::getRotation.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   * @return A quaternion representation of the object's orientation.
   */
  Magnum::Quaternion getRotation(const int objectID, const int sceneID = 0);

  /**
   * @brief Discrete collision check for contact between an object and the
   * collision world.
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param sceneID !! Not used currently !! Specifies which physical scene of
   * the object.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest(const int objectID, const int sceneID = 0);

  /**
   * @brief the physical world has a notion of time which passes during
   * animation/simulation/action/etc... Step the physical world forward in time
   * by a desired duration. Note that the actual duration of time passed by this
   * step will depend on simulation time stepping mode (todo). See @ref
   * esp::physics::PhysicsManager::stepPhysics.
   * @todo timestepping options?
   * @param dt The desired amount of time to advance the physical world.
   * @return The new world time after stepping. See @ref
   * esp::physics::PhysicsManager::worldTime_.
   */
  double stepWorld(const double dt = 1.0 / 60.0);

  /**
   * @brief Get the current time in the simulated world. This is always 0 if no
   * @ref esp::physics::PhysicsManager is initialized. See @ref stepWorld. See
   * @ref esp::physics::PhysicsManager::getWorldTime.
   * @return The amount of time, @ref esp::physics::PhysicsManager::worldTime_,
   * by which the physical world has advanced.
   */
  double getWorldTime();

  /**
   * @brief Compute the navmesh for the simulator's current active scene and
   * assign it to the referenced @ref nav::PathFinder.
   * @param pathfinder The pathfinder object to which the recomputed navmesh
   * will be assigned.
   * @param navMeshSettings The @ref nav::NavMeshSettings instance to
   * parameterize the navmesh construction.
   * @return Whether or not the navmesh recomputation succeeded.
   */
  bool recomputeNavMesh(nav::PathFinder& pathfinder,
                        const nav::NavMeshSettings& navMeshSettings);

  /**
   * @brief Enable or disable frustum culling (enabled by default)
   * @param val, true = enable, false = disable
   */
  void setFrustumCullingEnabled(bool val) { frustumCulling_ = val; }

  /**
   * @brief Get status, whether frustum culling is enabled or not
   * @return true if enabled, otherwise false
   */
  bool isFrustumCullingEnabled() { return frustumCulling_; }

  /**
   * @brief Get a named @ref LightSetup
   */
  gfx::LightSetup getLightSetup(
      const std::string& key = assets::ResourceManager::DEFAULT_LIGHTING_KEY);

  /**
   * @brief Set a named @ref LightSetup
   *
   * If this name already exists, the @ref LightSetup is updated and all @ref
   * Drawables using this setup are updated.
   *
   * @param lightSetup Light setup this key will now reference
   * @param key Key to identify this @ref LightSetup
   */
  void setLightSetup(
      gfx::LightSetup lightSetup,
      const std::string& key = assets::ResourceManager::DEFAULT_LIGHTING_KEY);

  /**
   * @brief Set the light setup of an object
   *
   * @param objectID The object ID and key identifying the object in @ref
   * esp::physics::PhysicsManager::existingObjects_.
   * @param lightSetupKey @ref LightSetup key
   * @param sceneID !! Not used currently !! Specifies which physical scene
   * of the object.
   */
  void setObjectLightSetup(int objectID,
                           const std::string& lightSetupKey,
                           int sceneID = 0);

 protected:
  Simulator(){};

  bool isValidScene(int sceneID) const {
    return sceneID >= 0 && sceneID < sceneID_.size();
  }

  bool sceneHasPhysics(int sceneID) const {
    return isValidScene(sceneID) && physicsManager_ != nullptr;
  }

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;
  // CANNOT make the specification of resourceManager_ above the context_!
  // Because when deconstructing the resourceManager_, it needs
  // the GL::Context
  // If you switch the order, you will have the error:
  // GL::Context::current(): no current context from Magnum
  // during the deconstruction
  assets::ResourceManager resourceManager_;

  scene::SceneManager sceneManager_;
  int activeSceneID_ = ID_UNDEFINED;
  int activeSemanticSceneID_ = ID_UNDEFINED;
  std::vector<int> sceneID_;

  std::shared_ptr<scene::SemanticScene> semanticScene_ = nullptr;

  std::shared_ptr<physics::PhysicsManager> physicsManager_ = nullptr;

  core::Random random_;
  SimulatorConfiguration config_;

  // state indicating frustum culling is enabled or not
  //
  // TODO:
  // Such state, frustumCulling_ has also been defined in frontend (py)
  // See: examples/settings.py, habitat_sim/simulator.py for more information
  // ideally, to avoid inconsistency at any time, and reduce maintenance cost
  // this state should be defined in just one place.e.g., only in the frontend
  // Currently, we need it defined here, because sensor., e.g., PinholeCamera
  // rquires it when drawing the observation
  bool frustumCulling_ = true;

  ESP_SMART_POINTERS(Simulator)
};

}  // namespace sim
}  // namespace esp
