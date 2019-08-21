/**
 * SimEnv class
 */
class SimEnv {
  // PUBLIC methods.

  /**
   * Create a simulator.
   * @param {Object} config - simulator config
   * @param {number} agentId - default agent id
   */
  constructor(config, agentId) {
    this.sim = new Module.Simulator(config);;
    this.defaultAgentId = agentId;
  }

  /**
   * Resets the simulation.
   */
  reset() {
    this.sim.reset();
    const agent = this.sim.getAgent(this.defaultAgentId);
    if (this.initialAgentState) {
      agent.setState(this.initialAgentState, true);
    }
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
   * @param {Object} state - initial state of agent
   */
  addAgent(config, state) {
    if (state) {
      this.initialAgentState = this.createAgentState(state);
    }
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
  getObservation(sensorId, buffer) {
    const obs = new Module.Observation();
    this.sim.getAgentObservation(0, sensorId, obs);
    return obs;
  }

  // PRIVATE methods.

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
      if (key === 'sensorSpecifications') {
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
