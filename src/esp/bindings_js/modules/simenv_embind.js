// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/*global Module */

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
    this.initialAgentState = null;

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
  }

  changeAgent(agentId) {
    this.selectedAgentId = agentId;
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
    return this.sim.addAgent(this.createAgentConfig(config));
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
    //Check if VisualSensor
    const VisualSensorTypeSet = new Set([
      Module.SensorType.COLOR,
      Module.SensorType.DEPTH,
      Module.SensorType.SEMANTIC,
      Module.SensorType.NORMAL
    ]);
    const CameraSensorSubTypeSet = new Set([
      Module.SensorSubType.PINHOLE,
      Module.SensorSubType.ORTHOGRAPHIC
    ]);
    if (
      VisualSensorTypeSet.has(config["sensorType"]) &&
      CameraSensorSubTypeSet.has(config["sensorSubType"])
    ) {
      //TODO: Implement checks for different sensors
      const converted = new Module.CameraSensorSpec();
      for (let key in config) {
        let value = config[key];
        converted[key] = value;
      }
      return converted;
    }
    return null;
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
