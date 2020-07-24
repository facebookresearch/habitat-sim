// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/*global Module */

import { primitiveObjectHandles, fileBasedObjectHandles } from "./defaults";

/**
 * SimEnv class
 *
 * TODO(aps,msb) - Add support for multiple agents instead of
 * hardcoding 0th one.
 */
class SimEnv {
  // PUBLIC methods.

  /**
   * Create a simulator.
   * @param {Object} config - simulator config
   * @param {Object} episode - episode to run
   * @param {number} agentId - default agent id
   */
  constructor(config, episode = {}, agentId = 0) {
    this.sim = new Module.Simulator(config);
    this.episode = episode;
    this.pathfinder = this.sim.getPathFinder();
    this.initialAgentState = null;
    this.resolution = null;

    if (Object.keys(episode).length > 0) {
      this.initialAgentState = this.createAgentState(episode.startState);
    }
    this.selectedAgentId = agentId;
  }

  /**
   * Resets the simulation.
   */
  reset() {
    this.sim.reset();
    if (this.initialAgentState !== null) {
      const agent = this.sim.getAgent(this.selectedAgentId);
      agent.setState(this.initialAgentState, true);
    }
    this.updateCrossHairNode(this.resolution);
    this.syncObjects();
    this.addRandomObjects();
  }

  /**
   * Change selected agent in the simulation.
   * @param {number} agentId - agent id
   */
  changeAgent(agentId) {
    this.selectedAgentId = agentId;
  }

  /**
   * Update cross hair node position.
   */
  updateCrossHairNode(windowSize) {
    this.sim.updateCrossHairNode(windowSize);
  }

  /**
   * Sync objects grabbed by agent to agent body.
   */
  syncObjects() {
    this.sim.syncGrippedObject();
    this.sim.syncGrippedObjects();
  }

  /**
   * Adds n random objects at random navigable points in simulation.
   */
  addRandomObjects(numberOfObjects = 4) {
    let primitiveObjectIdx = 0;
    let fileBaseObjectIdx = 0;
    for (let object = 0; object < numberOfObjects; object++) {
      if (object % 2 == 0) {
        this.addTemplateObject(fileBasedObjectHandles[fileBaseObjectIdx]);
        fileBaseObjectIdx++;
      } else {
        this.addPrimitiveObject(primitiveObjectHandles[primitiveObjectIdx]);
        //primitiveObjectIdx++;
      }
    }
    this.recomputeNavMesh();
  }

  /**
   * Take one step in the simulation.
   * @param {string} action - action to take
   */
  step(action) {
    const agent = this.sim.getAgent(this.selectedAgentId);
    agent.act(action);
  }

  /**
   * Add an agent to the simulation.
   * @param {Object} config - agent config
   */
  addAgent(config) {
    let agentConfig = this.createAgentConfig(config);
    this.resolution = agentConfig.sensorSpecifications.get(0).resolution;
    return this.sim.addAgent(agentConfig);
  }

  /**
   * Adds an instance of the specified object mesh to the environment.
   * @param {number} objectLibIndex - index of the object's template
   * @param {number} sceneId - specifies which physical scene to add an object to
   * @returns {number} object ID or -1 if object was unable to be added
   */
  addObject(objectLibIndex) {
    // using default values for rest of the params
    return this.sim.addObject(objectLibIndex, null, "", 0);
  }

  /**
   * Remove an instanced object by ID
   * @param {number} objectID - index of the object's template
   * @param {boolean} deleteObjectNode - if true, deletes the object's scene node
   * @param {boolean} deleteVisualNode - if true, deletes the object's visual node
   * @param {number} sceneId - specifies which physical scene to remove an object from
   */
  removeObject(objectId) {
    // using default values for rest of the params
    return this.sim.removeObject(objectId, true, true, 0);
  }

  /**
   * Adds an instance of the specified object mesh to the environment.
   * @param {string} objectLibHandle - object's template config/origin handle
   * @returns {number} object ID or -1 if object was unable to be added
   */
  addObjectByHandle(objectLibHandle) {
    // using default values for rest of the params
    return this.sim.addObjectByHandle(objectLibHandle, null, "", 0);
  }

  /**
   * Add a random primitive object to the environment.
   * @param {number} objectLibHandle - object's template config/origin handle
   * @returns {number} object ID or -1 if object was unable to be added
   */
  addPrimitiveObject(objectLibHandle = "capsule3DSolid") {
    let objectId = this.addObjectByHandle(objectLibHandle);
    let newPosition = this.pathfinder.getRandomNavigablePoint();
    this.setTranslation(newPosition, objectId, 0);
    this.setObjectMotionType(Module.MotionType.STATIC, objectId, 0);
    return objectId;
  }

  /**
   * Add a random file based object to the environment.
   * @param {number} objectLibHandle - object's template config/origin handle
   * @returns {number} object ID or -1 if object was unable to be added
   */
  addTemplateObject(
    objectLibHandle = "/data/objects/sphere.phys_properties.json"
  ) {
    let objectId = this.addObjectByHandle(objectLibHandle);
    let newPosition = this.pathfinder.getRandomNavigablePoint();
    this.setTranslation(newPosition, objectId, 0);
    this.setObjectMotionType(Module.MotionType.STATIC, objectId, 0);
    return objectId;
  }

  /**
   * Add a random primitive object to the environment.
   * @returns {number} object ID or -1 if object was unable to be added
   */
  removeLastObject() {
    let exsitingObjectIds = this.getExistingObjectIDs();
    this.removeObject(exsitingObjectIds.get(exsitingObjectIds.size() - 1));
  }

  /**
   * Grab or release object under cross hair.
   * @returns {number} object ID or -1 if object was unable to be added
   */
  grabReleaseObject() {
    this.sim.grabReleaseObjectUsingCrossHair(this.resolution);
  }

  /**
   * Get the observation space for a given sensorId.
   * @param {number} sensorId - id of sensor
   * @returns {ObservationSpace} observation space of sensor
   */
  getObservationSpace(sensorId) {
    return this.sim.getAgentObservationSpace(this.selectedAgentId, sensorId);
  }

  /**
   * Get an observation from the given sensorId.
   * @param {number} sensorId - id of sensor
   * @param {Observation} obs - observation is read into this object
   */
  getObservation(sensorId, obs) {
    this.sim.getAgentObservation(this.selectedAgentId, sensorId, obs);
    return obs;
  }

  /**
   * Get the PathFinder for the scene.
   * @returns {PathFinder} pathFinder of the scene
   */
  getPathFinder() {
    return this.sim.getPathFinder();
  }

  /**
   * Get the motion type of an object.
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   * @returns {MotionType} object MotionType or ERROR_MOTIONTYPE if query failed
   */
  getObjectMotionType(objectID, sceneID) {
    return this.sim.getObjectMotionType(objectID, sceneID);
  }

  /**
   * Set the motion type of an object.
   * * @param {MotionType} motionType - desired motion type of the object
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   * @returns {bool} object MotionType or ERROR_MOTIONTYPE if query failed
   */
  setObjectMotionType(motionType, objectID, sceneID) {
    return this.sim.setObjectMotionType(motionType, objectID, sceneID);
  }

  /**
   * Get the IDs of the physics objects instanced in a physical scene.
   * @param {number} sceneID - scene id
   * @returns {Array} list of existing object Ids in the scene
   */
  getExistingObjectIDs(sceneId = 0) {
    return this.sim.getExistingObjectIDs(sceneId);
  }

  /**
   * Set the 4x4 transformation matrix of an object kinematically.
   * @param {Magnum::Matrix4} transform - desired 4x4 transform of the object.
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   */
  setTransformation(transform, objectID, sceneID) {
    this.sim.setTransformation(transform, objectID, sceneID);
  }

  /**
   * Get the current 4x4 transformation matrix of an object.
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   * @returns {Magnum::Matrix4} 4x4 transform of the object
   */
  getTransformation(objectID, sceneID) {
    return this.sim.getTransformation(objectID, sceneID);
  }

  /**
   * Get the current 3D position of an object.
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   * @returns {Magnum::Vector3} 3D position of the object
   */
  getTranslation(objectID, sceneID) {
    return this.sim.getTranslation(objectID, sceneID);
  }

  /**
   * Set the 3D position of an object kinematically.
   * @param {Magnum::Vector3} translation - 3D position of the object
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   */
  setTranslation(translation, objectID, sceneID) {
    this.sim.setTranslation(translation, objectID, sceneID);
  }

  /**
   * Get the current orientation of an object.
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   * @returns {Magnum::Vector3} quaternion representation of the object's orientation
   */
  getRotation(objectID, sceneID) {
    return this.sim.getRotation(objectID, sceneID);
  }

  /**
   * Set the orientation of an object kinematically.
   * @param {Magnum::Vector3} rotation - desired orientation of the object
   * @param {number} objectID - object id identifying the object in sim.existingObjects_
   * @param {number} sceneID - scene id
   */
  setRotation(rotation, objectID, sceneID) {
    this.sim.setRotation(rotation, objectID, sceneID);
  }

  /**
   * @param {double} dt - step the physical world forward in time by a desired duration.
   * @returns {double} world time after step
   */
  stepWorld(dt = 1.0 / 60.0) {
    return this.sim.stepWorld(dt);
  }

  /**
   * Display an observation from the given sensorId
   * to canvas selected as default frame buffer.
   * @param {number} sensorId - id of sensor
   */
  displayObservation(sensorId) {
    this.sim.displayObservation(0, sensorId);
  }

  /**
   * Get the semantic scene.
   * @returns {SemanticScene} semantic scene
   */
  getSemanticScene() {
    return this.sim.getSemanticScene();
  }

  getAgentState() {
    let state = new Module.AgentState();
    const agent = this.sim.getAgent(this.selectedAgentId);
    agent.getState(state);
    return state;
  }

  recomputeNavMesh() {
    let navMeshSettings = new Module.NavMeshSettings();
    navMeshSettings.setDefaults();
    navMeshSettings.agentRadius = 0.3;
    this.sim.recomputeNavMesh(this.getPathFinder(), navMeshSettings, true);
  }

  toggleNavMeshVisualization() {
    this.sim.toggleNavMeshVisualization();
  }

  /**
   * Get the distance to goal in polar coordinates.
   * @returns {Array} [magnitude, clockwise-angle (in radians)]
   */
  distanceToGoal() {
    if (Object.keys(this.episode).length === 0) {
      return [0, 0];
    }
    let dst = this.episode.goal.position;
    let state = this.getAgentState();
    let src = state.position;
    let dv = [dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]];
    dv = this.applyRotation(dv, state.rotation);
    return this.cartesian_to_polar(-dv[2], dv[0]);
  }

  // PRIVATE methods.

  // Rotate vector, v, by quaternion, q.
  // Result r = q' * v * q where q' is the quaternion conjugate.
  // http://www.chrobotics.com/library/understanding-quaternions
  applyRotation(v, q) {
    let x, y, z;
    [x, y, z] = v;
    let qx, qy, qz, qw;
    [qx, qy, qz, qw] = q;

    // i = q' * v
    let ix = qw * x - qy * z + qz * y;
    let iy = qw * y - qz * x + qx * z;
    let iz = qw * z - qx * y + qy * x;
    let iw = qx * x + qy * y + qz * z;

    // r = i * q
    let r = [];
    r[0] = ix * qw + iw * qx + iy * qz - iz * qy;
    r[1] = iy * qw + iw * qy + iz * qx - ix * qz;
    r[2] = iz * qw + iw * qz + ix * qy - iy * qx;

    return r;
  }

  cartesian_to_polar(x, y) {
    return [Math.sqrt(x * x + y * y), Math.atan2(y, x)];
  }

  createSensorSpec(config) {
    const converted = new Module.SensorSpec();
    for (let key in config) {
      let value = config[key];
      converted[key] = value;
    }
    return converted;
  }

  createAgentConfig(config) {
    const converted = new Module.AgentConfiguration();
    for (let key in config) {
      let value = config[key];
      if (key === "sensorSpecifications") {
        const sensorSpecs = new Module.VectorSensorSpec();
        for (let c of value) {
          sensorSpecs.push_back(this.createSensorSpec(c));
        }
        value = sensorSpecs;
      }
      converted[key] = value;
    }
    return converted;
  }

  createAgentState(state) {
    const converted = new Module.AgentState();
    for (let key in state) {
      let value = state[key];
      converted[key] = value;
    }
    return converted;
  }
}

export default SimEnv;
