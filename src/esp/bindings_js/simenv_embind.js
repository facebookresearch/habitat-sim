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
  constructor(config, episode, agentId) {
    this.sim = new Module.Simulator(config);
    this.episode = episode;
    this.initialAgentState = this.createAgentState(episode.startState);
    this.defaultAgentId = agentId;
  }

  /**
   * Resets the simulation.
   */
  reset() {
    this.sim.reset();
    const agent = this.sim.getAgent(this.defaultAgentId);
    agent.setState(this.initialAgentState, true);
  }

  /**
   * Take one step in the simulation.
   * @param {string} action - action to take
   */
  step(action) {
    const agent = this.sim.getAgent(this.defaultAgentId);
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
    return this.sim.getAgentObservationSpace(this.defaultAgentId, sensorId);
  }

  /**
   * Get an observation from the given sensorId.
   * @param {number} sensorId - id of sensor
   * @returns {Observation} observation from sensor
   */
  getObservation(sensorId) {
    const obs = new Module.Observation();
    this.sim.getAgentObservation(0, sensorId, obs);
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

  /**
   * Get the distance to goal in polar coordinates.
   * @returns {Array} [magnitude, clockwise-angle (in radians)]
   */
  distanceToGoal() {
    let dst = this.episode.goal.position;
    let state = new Module.AgentState();
    const agent = this.sim.getAgent(this.defaultAgentId);
    agent.getState(state);
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
