// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_SIMULATOR_H_
#define ESP_SIM_SIMULATOR_H_

#include <Corrade/Utility/Assert.h>

#include <utility>
#include "esp/assets/ResourceManager.h"
#include "esp/core/Esp.h"
#include "esp/core/Random.h"
#include "esp/gfx/DebugLineRender.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx/replay/Player.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/Sensor.h"

#include "SimulatorConfiguration.h"

namespace esp {
namespace nav {
class PathFinder;
struct NavMeshSettings;
class ActionSpacePathFinder;
}  // namespace nav
namespace scene {
class SemanticScene;
}  // namespace scene
namespace gfx {
class Renderer;
namespace replay {
class ReplayManager;
}  // namespace replay
}  // namespace gfx
}  // namespace esp

namespace esp {
namespace sim {
class Simulator {
 public:
  explicit Simulator(
      const SimulatorConfiguration& cfg,
      std::shared_ptr<metadata::MetadataMediator> _metadataMediator = nullptr);
  virtual ~Simulator();

  /**
   * @brief Closes the simulator and frees all loaded assets and GPU contexts.
   *
   * @warning Must reset the simulator to its "just after constructor" state for
   * python inheritance to function correctly.  Shared/unique pointers should be
   * set back to nullptr, any members set to their default values, etc.  If this
   * is not done correctly, the pattern for @ref `close` then @ref `reconfigure`
   * to create a "fresh" instance of the simulator may not work correctly
   *
   * @param destroy When set, destroy the background rendering thread and gl
   * context also. Otherwise these persist if the background rendering thread
   * was used. This is done because the OpenGL driver leaks memory on thread
   * destroy on some systems.
   */
  void close(bool destroy = true);

  void reconfigure(const SimulatorConfiguration& cfg);

  /**
   * @brief Allows side-loading semantic scene metadata from a semantic json
   * without loading the connected scene instance.
   *
   */
  void loadSemanticSceneDescriptor(const std::string& semSceneFilename);

  /**
   * @brief Reset the simulation state including the state of all physics
   * objects and the default light setup.
   * Sets the @ref worldTime_ to 0.0, changes the physical state of all objects back to their initial states.
   * Does not invalidate existing ManagedObject wrappers.
   * Does not add or remove object instances.
   * Only changes motion_type when scene_instance specified a motion type.
   * @param calledAfterSceneCreate Whether this reset is being called after a
   * new scene has been created in reconfigure. If so we con't want to
   * redundantly re-place the newly-placed object positions.
   */
  void reset(bool calledAfterSceneCreate = false);

  void seed(uint32_t newSeed);

  std::shared_ptr<gfx::Renderer> getRenderer() { return renderer_; }

  inline void getRenderGLContext() {
    // acquire GL context from background thread, if background rendering
    // enabled.
    if (renderer_) {
      renderer_->acquireGlContext();
    }
  }

  std::shared_ptr<gfx::replay::ReplayManager> getGfxReplayManager() {
    return gfxReplayMgr_;
  }

  /**
   * @brief The ID of the CUDA device of the OpenGL context owned by the
   * simulator. This will only be nonzero if the simulator is built in
   * --headless mode on linux
   */
  int gpuDevice() const {
    if (context_ == nullptr) {
      return 0;
    }
    return context_->gpuDevice();
  }

  // === Physics Simulator Functions ===
  /**
   * @brief Return manager for construction and access to articulated object
   * attributes for the current dataset.
   */
  const std::shared_ptr<metadata::managers::AOAttributesManager>&
  getAOAttributesManager() const;
  /**
   * @brief Return manager for construction and access to asset attributes for
   * the current dataset.
   */
  const std::shared_ptr<metadata::managers::AssetAttributesManager>&
  getAssetAttributesManager() const;
  /**
   * @brief Return manager for construction and access to light attributes and
   * layouts for the current dataset.
   */
  const std::shared_ptr<metadata::managers::LightLayoutAttributesManager>&
  getLightLayoutAttributesManager() const;

  /**
   * @brief Return manager for construction and access to object attributes and
   * layouts for the current dataset.
   */
  const std::shared_ptr<metadata::managers::ObjectAttributesManager>&
  getObjectAttributesManager() const;
  /**
   * @brief Return manager for construction and access to physics world
   * attributes.
   */
  const std::shared_ptr<metadata::managers::PhysicsAttributesManager>&
  getPhysicsAttributesManager() const;
  /**
   * @brief Return manager for construction and access to scene attributes for
   * the current dataset.
   */
  const std::shared_ptr<metadata::managers::StageAttributesManager>&
  getStageAttributesManager() const;

  /**
   * @brief Get current active dataset name from @ref metadataMediator_.
   */
  std::string getActiveSceneDatasetName();

  /**
   * @brief Set current active dataset name from @ref metadataMediator_.
   * @param _dsHandle The desired dataset to switch to. If has not been loaded,
   * an attempt will be made to load it.
   */
  void setActiveSceneDatasetName(const std::string& _dsHandle);

  /** @brief Return the library implementation type for the simulator currently
   * in use. Use to check for a particular implementation.
   * @return The implementation type of this simulator.
   */
  const esp::physics::PhysicsManager::PhysicsSimulationLibrary&
  getPhysicsSimulationLibrary() const {
    return physicsManager_->getPhysicsSimulationLibrary();
  }

  /** @brief Render any debugging visualizations provided by the underlying
   * physics simulator implementation. By default does nothing. See @ref
   * BulletPhysicsManager::debugDraw.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  void physicsDebugDraw(const Magnum::Matrix4& projTrans) const {
    physicsManager_->debugDraw(projTrans);
  }

  /**
   * @brief Get a copy of a stage's template when the stage was instanced.
   *
   * Use this to query the stage's properties when it was initialized.
   */
  metadata::attributes::StageAttributes::cptr getStageInitializationTemplate()
      const {
    if (sceneHasPhysics()) {
      return physicsManager_->getStageInitAttributes();
    }
    return nullptr;
  }

  /**
   * @brief get the current active scene graph
   */
  scene::SceneGraph& getActiveSceneGraph() {
    CORRADE_INTERNAL_ASSERT(std::size_t(activeSceneID_) < sceneID_.size());
    return sceneManager_->getSceneGraph(activeSceneID_);
  }

  ///////////////////////////
  // Semantic Scene and Data
  std::shared_ptr<scene::SemanticScene> getSemanticScene() {
    return resourceManager_->getSemanticScene();
  }

  /**
   * @brief Return a view of the currently set Semantic scene colormap.
   */
  const std::vector<Mn::Vector3ub>& getSemanticSceneColormap() const {
    return resourceManager_->getSemanticSceneColormap();
  }

  /** @brief check if the semantic scene exists.*/
  bool semanticSceneExists() const {
    return resourceManager_->semanticSceneExists();
  }

  /** @brief Check to see if there is a SemanticSceneGraph for rendering */
  bool semanticSceneGraphExists() const {
    return std::size_t(activeSemanticSceneID_) < sceneID_.size();
  }

  /** @brief get the semantic scene's SceneGraph for rendering */
  scene::SceneGraph& getActiveSemanticSceneGraph() {
    CORRADE_INTERNAL_ASSERT(semanticSceneGraphExists());
    return sceneManager_->getSceneGraph(activeSemanticSceneID_);
  }

  /**
   * @brief Build a map keyed by semantic color/id referencing a list of
   * connected component-based Semantic objects.
   */
  std::unordered_map<uint32_t, std::vector<scene::CCSemanticObject::ptr>>
  buildSemanticCCObjects() const {
    // build report with current stage attributes
    if (semanticSceneMeshLoaded_) {
      return resourceManager_->buildSemanticCCObjects(
          physicsManager_->getStageInitAttributes());
    }
    ESP_WARNING()
        << "Unable to build semantic CC objects since no semantic mesh exists.";
    return {};
  }

  /**
   * @brief Build a report on vertex color mappings to semantic scene descriptor
   * object colors, to show whether any verts have unknown colors and whether
   * any semantic object colors are not present in the mesh.
   */
  std::vector<std::string> buildVertexColorMapReport() const {
    if (semanticSceneMeshLoaded_) {
      return resourceManager_->buildVertexColorMapReport(
          physicsManager_->getStageInitAttributes());
    }
    ESP_WARNING() << "Unable to build vertex-to-color mapping report since no "
                     "semantic mesh exists.";
    return {};
  }

  ///////////////////////////
  // End Semantic Scene and Data

  /**
   * @brief Builds a @ref esp::metadata::attributes::SceneInstanceAttributes describing the
   * current scene configuration, and saves it to a JSON file, using @p
   * saveFileName .
   * @param saveFilename The name to use to save the current scene instance.
   * @return whether successful or not.
   */
  bool saveCurrentSceneInstance(const std::string& saveFilename) const;

  /**
   * @brief Builds a @ref esp::metadata::attributes::SceneInstanceAttributes describing
   * the current scene configuration, and saves it to a JSON file, using the
   * current scene attributes' filename, or an incremented version if @p
   * overwrite == false.
   * @param overwrite Whether to overwrite an existing file with the same name,
   * should one exist.
   * @return whether successful or not.
   */
  bool saveCurrentSceneInstance(bool overwrite = false) const;

  /**
   * @brief Get the IDs of the physics objects instanced in a physical scene.
   * See @ref esp::physics::PhysicsManager::getExistingObjectIDs.
   * @return A vector of ID keys into @ref
   * esp::physics::PhysicsManager::existingObjects_.
   */
  std::vector<int> getExistingObjectIDs() {
    if (sceneHasPhysics()) {
      return physicsManager_->getExistingObjectIDs();
    }
    return std::vector<int>();  // empty if no simulator exists
  }

  /**
   * @brief Get the axis-aligned bounding box (AABB) of the scene in global
   * space.
   */
  const Mn::Range3D& getSceneAabb() {
    return getActiveSceneGraph().getRootNode().getCumulativeBB();
  }

  /**
   * @brief Perform discrete collision detection for the scene.
   */
  void performDiscreteCollisionDetection() {
    physicsManager_->performDiscreteCollisionDetection();
  };

  /**
   * @brief Query physics simulation implementation for contact point data from
   * the most recent collision detection cache.
   *
   * @return a vector with each entry corresponding to a single contact point.
   */
  std::vector<esp::physics::ContactPointData> getPhysicsContactPoints() {
    return physicsManager_->getContactPoints();
  }

  /**
   * @brief Query the number of contact points that were active during the
   * collision detection check.
   *
   * @return the number of active contact points.
   */
  int getPhysicsNumActiveContactPoints() {
    return physicsManager_->getNumActiveContactPoints();
  }

  /**
   * @brief See BulletPhysicsManager.h getNumActiveOverlappingPairs
   */
  int getPhysicsNumActiveOverlappingPairs() {
    return physicsManager_->getNumActiveOverlappingPairs();
  }

  /**
   * @brief See BulletPhysicsManager.h getStepCollisionSummary
   */
  std::string getPhysicsStepCollisionSummary() {
    return physicsManager_->getStepCollisionSummary();
  }

  /**
   * @brief Set the stage to collidable or not.
   */
  void setStageIsCollidable(bool collidable) {
    if (sceneHasPhysics()) {
      physicsManager_->setStageIsCollidable(collidable);
    }
  }

  /**
   * @brief Get whether or not the stage is collision active.
   */
  bool getStageIsCollidable() {
    if (sceneHasPhysics()) {
      return physicsManager_->getStageIsCollidable();
    }
    return false;
  }

  /**
   * @brief returns the wrapper manager for the currently created rigid objects.
   * @return RigidObject wrapper manager.
   */
  std::shared_ptr<esp::physics::RigidObjectManager> getRigidObjectManager()
      const {
    if (sceneHasPhysics()) {
      return physicsManager_->getRigidObjectManager();
    }
    return nullptr;
  }  // getRigidObjectManager

  /**
   * @brief returns the wrapper manager for the currently created articulated
   * objects.
   * @return ArticulatedObject wrapper manager
   */
  std::shared_ptr<esp::physics::ArticulatedObjectManager>
  getArticulatedObjectManager() const {
    if (sceneHasPhysics()) {
      return physicsManager_->getArticulatedObjectManager();
    }
    return nullptr;
  }

  /**
   * @brief Raycast into the collision world of a scene.
   *
   * Note: A default @ref physics::PhysicsManager has no collision world, so
   * physics must be enabled for this feature.
   *
   * @param ray The ray to cast. Need not be unit length, but returned hit
   * distances will be in units of ray length.
   * @param maxDistance The maximum distance along the ray direction to search.
   * In units of ray length.
   * @param bufferDistance The casts the ray from this distance behind the
   * origin in the inverted ray direction to avoid errors related to casting
   * rays inside a collision shape's margin.
   * @return Raycast results sorted by distance.
   */
  esp::physics::RaycastResults castRay(const esp::geo::Ray& ray,
                                       double maxDistance = 100.0,
                                       double bufferDistance = 0.08) {
    if (sceneHasPhysics()) {
      return physicsManager_->castRay(ray, maxDistance, bufferDistance);
    }
    return esp::physics::RaycastResults();
  }

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
  double stepWorld(double dt = 1.0 / 60.0);

  /**
   * @brief Get the current time in the simulated world. This is always 0 if no
   * @ref esp::physics::PhysicsManager is initialized. See @ref stepWorld. See
   * @ref esp::physics::PhysicsManager::getWorldTime.
   * @return The amount of time, @ref esp::physics::PhysicsManager::worldTime_,
   * by which the physical world has advanced.
   */
  double getWorldTime();

  /**
   * @brief Get the last physics timestep in seconds
   *
   * @return The timestep.
   */
  double getPhysicsTimeStep();

  /**
   * @brief Get the simplified name of the @ref
   * esp::metadata::attributes::SceneInstanceAttributes used to create the scene
   * currently being simulated/displayed.
   */
  std::string getCurSceneInstanceName() const {
    if (curSceneInstanceAttributes_ == nullptr) {
      return "NONE";
    }
    return curSceneInstanceAttributes_->getSimplifiedHandle();
  }

  /**
   * @brief Return whether the @ref
   * esp::metadata::attributes::SceneInstanceAttributes used to create the scene
   * currently being simulated/displayed specifies a default COM handling for
   * rigid objects
   */

  bool getCurSceneDefaultCOMHandling() const {
    if (curSceneInstanceAttributes_ == nullptr) {
      return false;
    }
    return (curSceneInstanceAttributes_->getTranslationOrigin() ==
            metadata::attributes::SceneInstanceTranslationOrigin::AssetLocal);
  }

  /**
   * @brief Set the gravity in a physical scene.
   */
  void setGravity(const Magnum::Vector3& gravity) {
    if (sceneHasPhysics()) {
      physicsManager_->setGravity(gravity);
    }
  }

  /**
   * @brief Get the gravity in a physical scene.
   */
  Magnum::Vector3 getGravity() const {
    if (sceneHasPhysics()) {
      return physicsManager_->getGravity();
    }
    return Magnum::Vector3();
  }

  std::shared_ptr<esp::gfx::DebugLineRender> getDebugLineRender() {
    // We only create this if/when used (lazy creation)
    if (!debugLineRender_) {
      debugLineRender_ = std::make_shared<esp::gfx::DebugLineRender>();
    }
    return debugLineRender_;
  }

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
   * @brief Get the joined mesh data for all objects in the scene
   * @param includeStaticObjects flag to include static objects
   * @return A shared ptr assets::MeshData with required mesh
   */
  assets::MeshData::ptr getJoinedMesh(bool includeStaticObjects = false);

  /**
   * @brief Get the joined semantic mesh data for all objects in the scene
   * @param[out] objectIds will be populated with the object ids for the
   * semantic mesh
   * @return A shared ptr assets::MeshData with required mesh and populate the
   * objectIds
   */
  assets::MeshData::ptr getJoinedSemanticMesh(
      std::vector<std::uint16_t>& objectIds);

  /**
   * @brief Set visualization of the current NavMesh @ref pathfinder_ on or off.
   *
   * @param visualize Whether or not to visualize the navmesh.
   * @return Whether or not the NavMesh visualization is active.
   */
  bool setNavMeshVisualization(bool visualize);

  /**
   * @brief Query active state of the current NavMesh @ref pathfinder_
   * visualization.
   */
  bool isNavMeshVisualizationActive();

  /**
   * @brief Return a ref to a new drawables in the currently active scene, for
   * object creation.
   */
  inline esp::gfx::DrawableGroup& getDrawableGroup() {
    return sceneManager_->getSceneGraph(activeSceneID_).getDrawables();
  }

  /**
   * @brief Compute a trajectory visualization for the passed points.
   * @param trajVisName The name to use for the trajectory visualization
   * @param pts The points of a trajectory, in order
   * @param colorVec Array of colors for trajectory tube.
   * @param numSegments The number of the segments around the circumference of
   * the tube. Must be greater than or equal to 3.
   * @param radius The radius of the tube.
   * @param smooth Whether to smooth the points in the trajectory or not. Will
   * build a much bigger mesh
   * @param numInterp The number of interpolations between each trajectory
   * point, if smoothed
   * @return The ID of the object created for the visualization
   */
  int addTrajectoryObject(const std::string& trajVisName,
                          const std::vector<Mn::Vector3>& pts,
                          const std::vector<Mn::Color3>& colorVec,
                          int numSegments = 3,
                          float radius = .001,
                          bool smooth = false,
                          int numInterp = 10) {
    if (sceneHasPhysics()) {
      return physicsManager_->addTrajectoryObject(
          trajVisName, pts, colorVec, numSegments, radius, smooth, numInterp);
    }
    return ID_UNDEFINED;
  }
  /**
   * @brief Remove a trajectory visualization by name.
   * @param trajVisName The name of the trajectory visualization to remove.
   * @return whether successful or not.
   */
  bool removeTrajVisByName(const std::string& trajVisName) {
    if (sceneHasPhysics()) {
      return physicsManager_->removeTrajVisByName(trajVisName);
    }
    return false;
  }

  /**
   * @brief Remove a trajectory visualization by object ID.
   * @param trajVisObjID The object ID of the trajectory visualization to
   * remove.
   * @return whether successful or not.
   */
  bool removeTrajVisByID(int trajVisObjID) {
    if (sceneHasPhysics()) {
      return physicsManager_->removeTrajVisByID(trajVisObjID);
    }
    return false;
  }

  /**
   * @brief Initialize a sensor from its spec and attach it to the SceneNode of
   * a particular object or link.
   * @param objectId    Id of the object to which a sensor will be initialized
   * at its node
   * @param sensorSpec  SensorSpec of sensor to be initialized
   * @return            handle to sensor initialized
   *
   */
  esp::sensor::Sensor& addSensorToObject(
      int objectId,
      const esp::sensor::SensorSpec::ptr& sensorSpec);

  /**
   * @brief Displays observations on default frame buffer for a
   * particular sensor
   * @param sensorUuid   Unique Id of the sensor for which the observation is to
   *                   be returned
   */
  bool displayObservation(const std::string& sensorUuid);

  /**
   * @brief
   * get the render target of a particular sensor
   * @return pointer to the render target if it is a valid visual sensor,
   * otherwise nullptr
   * @param sensorUuid   Id of the sensor for which the observation is to
   *                   be returned
   */
  gfx::RenderTarget* getRenderTarget(const std::string& sensorUuid);

  /**
   * @brief draw observations to the frame buffer stored in that
   * particular sensor. Unlike the @displayObservation, it will
   * not display the observation on the default frame buffer

   * @param sensorUuid   Id of the sensor for which the observation is to
   *                   be returned
   */
  bool drawObservation(const std::string& sensorUuid);

  /**
   * @brief visualize the undisplayable observations such as depth, semantic, to
   * the frame buffer stored in the @ref SensorInfoVisualizer
   * Note: it will not display the observation on the default frame buffer
   * @param[in] sensorUuid   Id of the sensor for which the observation is to
   *                   be returned
   * @param[in] colorMapOffset the offset of the color map
   * @param[in] colorMapScale the scale of the color map
   * See details in @ref TextureVisualizerShader::setColorMapTransformation for
   * more info.
   *
   * NOTE: it assumes:
   * -) it is a non-rgb sensor (such as a depth or semantic sensor);
   * -) the drawObservation is called;
   * -) when the render target is bound to the sensor, "VisualizeTexture" is
   * enabled. See @ref Renderer::bindRenderTarget and @ref
   * Renderer::RenderTargetBindingFlag for more info
   * @return false if the sensor's observation cannot be visualized.
   */
  bool visualizeObservation(const std::string& sensorUuid,
                            float colorMapOffset,
                            float colorMapScale);

  bool visualizeObservation(const std::string& sensorUuid);

  bool getSensorObservation(const std::string& sensorUuid,
                            sensor::Observation& observation);

  bool getSensorObservationSpace(const std::string& sensorUuid,
                                 sensor::ObservationSpace& space);

  nav::PathFinder::ptr getPathFinder() { return pathfinder_; }
  void setPathFinder(nav::PathFinder::ptr pf);

  /**
   * @brief Enable or disable frustum culling (enabled by default)
   * @param val true = enable, false = disable
   */
  void setFrustumCullingEnabled(bool val) { frustumCulling_ = val; }

  /**
   * @brief Get status, whether frustum culling is enabled or not
   * @return true if enabled, otherwise false
   */
  bool isFrustumCullingEnabled() const { return frustumCulling_; }

  /**
   * @brief Get a copy of an existing @ref gfx::LightSetup by its key.
   *
   * @param key The string key of the @ref gfx::LightSetup.
   */
  gfx::LightSetup getLightSetup(const std::string& key = DEFAULT_LIGHTING_KEY) {
    return *resourceManager_->getLightSetup(key);
  }

  /**
   * @brief Get a copy of the currently set existing @ref gfx::LightSetup.
   *
   * @param key The string key of the @ref gfx::LightSetup.
   */
  gfx::LightSetup getCurrentLightSetup() {
    return *resourceManager_->getLightSetup(config_.sceneLightSetupKey);
  }

  /**
   * @brief retrieve Current lightsetup key
   */
  std::string getCurrentLightSetupKey() const {
    return config_.sceneLightSetupKey;
  }

  /**
   * @brief Register a @ref gfx::LightSetup with a key name.
   *
   * If this name already exists, the @ref gfx::LightSetup is updated and all
   * @ref esp::gfx::Drawable s using this setup are updated.
   *
   * @param lightSetup The @ref gfx::LightSetup this key will now reference.
   * @param key Key to identify this @ref gfx::LightSetup.
   */
  void setLightSetup(gfx::LightSetup lightSetup,
                     const std::string& key = DEFAULT_LIGHTING_KEY) {
    resourceManager_->setLightSetup(std::move(lightSetup), key);
  }

  //============= Object Rigid Constraint API =============

  /**
   * @brief Create a rigid constraint between two objects or an object and the
   * world.
   *
   * Note: requires Bullet physics to be enabled.
   *
   * @param settings The datastructure defining the constraint parameters.
   *
   * @return The id of the newly created constraint or ID_UNDEFINED if failed.
   */
  int createRigidConstraint(const physics::RigidConstraintSettings& settings) {
    return physicsManager_->createRigidConstraint(settings);
  }

  /**
   * @brief Update the settings of a rigid constraint.
   *
   * Note: requires Bullet physics to be enabled.
   *
   * @param constraintId The id of the constraint to update.
   * @param settings The new settings of the constraint.
   */
  void updateRigidConstraint(int constraintId,
                             const physics::RigidConstraintSettings& settings) {
    physicsManager_->updateRigidConstraint(constraintId, settings);
  }

  /**
   * @brief Remove a rigid constraint by id.
   *
   * Note: requires Bullet physics to be enabled.
   *
   * @param constraintId The id of the constraint to remove.
   */
  void removeRigidConstraint(int constraintId) {
    physicsManager_->removeRigidConstraint(constraintId);
  }

  /**
   * @brief Get a copy of the settings for an existing rigid constraint.
   *
   * Note: requires Bullet physics to be enabled.
   *
   * @param constraintId The id of the constraint.
   *
   * @return The settings of the constraint.
   */
  physics::RigidConstraintSettings getRigidConstraintSettings(
      int constraintId) const {
    return physicsManager_->getRigidConstraintSettings(constraintId);
  }

  //============= END - Object Rigid Constraint API =============

  /**
   * @brief Getter for PRNG.
   *
   * Use this where-ever possible so that habitat won't be effected by
   * python random or numpy.random modules.
   */
  core::Random::ptr random() { return random_; }

  /**
   * @brief Get this simulator's MetadataMediator
   */
  std::shared_ptr<metadata::MetadataMediator> getMetadataMediator() const {
    return metadataMediator_;
  }

  /**
   * @brief Set this simulator's MetadataMediator
   */
  void setMetadataMediator(
      std::shared_ptr<metadata::MetadataMediator> _metadataMediator);

  /**
   * @brief Load and add a render asset instance to the current scene graph(s).
   * @param assetInfo the asset to load
   * @param creation how to create the instance
   */
  scene::SceneNode* loadAndCreateRenderAssetInstance(
      const assets::AssetInfo& assetInfo,
      const assets::RenderAssetInstanceCreationInfo& creation);

  /**
   * @brief Runtime perf stats are various scalars helpful for troubleshooting
   * runtime perf.
   *
   * @return A vector of stat names; currently, this is constant so it can be
   * called once at startup. See also getRuntimePerfStatValues.
   */
  std::vector<std::string> getRuntimePerfStatNames();

  /**
   * @brief Runtime perf stats are various scalars helpful for troubleshooting
   * runtime perf.
   *
   * @return a vector of stat values. Stat values generally change after every
   * physics step. See also getRuntimePerfStatNames.
   */
  std::vector<float> getRuntimePerfStatValues();

 protected:
  Simulator() = default;

  /**
   * @brief if Navmesh visualization is active, reset the visualization.
   */
  void resetNavMeshVisIfActive() {
    if (isNavMeshVisualizationActive()) {
      // if updating pathfinder_ instance, refresh the visualization.
      setNavMeshVisualization(false);  // first clear the old instance
      setNavMeshVisualization(true);
    }
  }

  /**
   * @brief Builds a scene instance and populates it with initial object
   * layout, if appropriate, based on @ref
   * esp::metadata::attributes::SceneInstanceAttributes referenced by @p
   * activeSceneName .
   * @param activeSceneName The name of the desired SceneInstanceAttributes to
   * use to instantiate a scene.
   * @return Whether successful or not.
   */
  bool createSceneInstance(const std::string& activeSceneName);

  /**
   * @brief Instance the stage for the current scene based on
   * curSceneInstanceAttributes_, the currently active scene's @ref
   * esp::metadata::attributes::SceneInstanceAttributes
   * @param curSceneInstanceAttributes The attributes describing the current
   * scene instance.
   * @param curSemanticAttr The SemanticAttributes referenced by the current
   * scene instance, or nullptr if none.
   * @return whether stage creation is completed successfully
   */
  bool instanceStageForSceneAttributes(
      const metadata::attributes::SceneInstanceAttributes::cptr&
          curSceneInstanceAttributes,
      const std::shared_ptr<esp::metadata::attributes::SemanticAttributes>&
          semanticAttr);

  /**
   * @brief Instance all the objects in the scene based on
   * curSceneInstanceAttributes_, the currently active scene's @ref
   * esp::metadata::attributes::SceneInstanceAttributes.
   * @param curSceneInstanceAttributes The attributes describing the current
   * scene instance.
   * @return whether object creation and placement is completed successfully
   */
  bool instanceObjectsForSceneAttributes(
      const metadata::attributes::SceneInstanceAttributes::cptr&
          curSceneInstanceAttributes);

  /**
   * @brief Instance all the articulated objects in the scene based
   * on curSceneInstanceAttributes_, the currently active scene's @ref
   * esp::metadata::attributes::SceneInstanceAttributes.
   * @param curSceneInstanceAttributes The attributes describing the current
   * scene instance.
   * @return whether articulated object creation and placement is completed
   * successfully
   */
  bool instanceArticulatedObjectsForSceneAttributes(
      const metadata::attributes::SceneInstanceAttributes::cptr&
          curSceneInstanceAttributes);

  /**
   * @brief Build a @ref metadata::attributes::SceneInstanceAttributes
   * describing the current scene's present state.
   */
  metadata::attributes::SceneInstanceAttributes::ptr
  buildCurrentStateSceneAttributes() const;

  bool sceneHasPhysics() const { return physicsManager_ != nullptr; }

  void reconfigureReplayManager(bool enableGfxReplaySave);

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;
  // CANNOT make the specification of resourceManager_ above the context_!
  // Because when deconstructing the resourceManager_, it needs
  // the GL::Context
  // If you switch the order, you will have the error:
  // GL::Context::current(): no current context from Magnum
  // during the deconstruction
  std::unique_ptr<assets::ResourceManager> resourceManager_ = nullptr;

  /**
   * @brief Owns and manages the metadata/attributes managers
   */
  std::shared_ptr<metadata::MetadataMediator> metadataMediator_ = nullptr;

  /**
   * @brief Configuration describing currently active scene
   */
  metadata::attributes::SceneInstanceAttributes::cptr
      curSceneInstanceAttributes_ = nullptr;

  scene::SceneManager::uptr sceneManager_ = nullptr;

  int activeSceneID_ = ID_UNDEFINED;
  int activeSemanticSceneID_ = ID_UNDEFINED;

  /**
   * @brief Whether or not the loaded scene has a semantic scene mesh loaded.
   */
  bool semanticSceneMeshLoaded_ = false;

  std::vector<int> sceneID_;

  std::shared_ptr<physics::PhysicsManager> physicsManager_ = nullptr;

  std::shared_ptr<esp::gfx::replay::ReplayManager> gfxReplayMgr_;

  core::Random::ptr random_;
  SimulatorConfiguration config_;

  nav::PathFinder::ptr pathfinder_;
  // state indicating frustum culling is enabled or not
  //
  // TODO:
  // Such state, frustumCulling_ has also been defined in frontend (py)
  // See: examples/settings.py, src_python/habitat_sim/simulator.py for more
  // information ideally, to avoid inconsistency at any time, and reduce
  // maintenance cost this state should be defined in just one place.e.g., only
  // in the frontend Currently, we need it defined here, because sensor., e.g.,
  // PinholeCamera requires it when drawing the observation
  bool frustumCulling_ = true;

  //! NavMesh visualization variables
  int navMeshVisPrimID_ = esp::ID_UNDEFINED;
  esp::scene::SceneNode* navMeshVisNode_ = nullptr;

  /**
   * @brief Tracks whether or not the simulator was initialized
   * to load textures.  Because we cache mesh loading, this should
   * *not* be changed without calling close() first
   */
  Corrade::Containers::Optional<bool> requiresTextures_;

  std::shared_ptr<esp::gfx::DebugLineRender> debugLineRender_;

  std::vector<float> runtimePerfStatValues_;

  ESP_SMART_POINTERS(Simulator)
};

}  // namespace sim
}  // namespace esp

#endif  // ESP_SIM_SIMULATOR_H_
