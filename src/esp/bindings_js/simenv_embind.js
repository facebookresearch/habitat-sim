class SimEnv {
  // Uses embind simulator to act
  // (use promises so we can also use http to connect to simulator server)
  constructor(sim, agentId) {
    this.sim = sim;
    this.defaultAgentId = agentId;
  }

  reset() {
    return Promise.resolve(this.sim.reset());
  }

  step(action) {
    return this.act(this.defaultAgentId, action.name);
  }

  render(sensorId, buffer) {
    return this.getObservation(this.defaultAgentId, sensorId, buffer);
  }

  act(agentId, action) {
    const agent = this.sim.getAgent(agentId);
    if (agent) {
      return Promise.resolve(agent.act(action));
    } else {
      return Promise.reject("Invalid agent");
    }
  }

  getObservationSpace(agentId, sensorId) {
    return Promise.resolve(this.sim.getAgentObservationSpace(agentId, sensorId));
  }

  // TODO: Move and push this flip code into habitat-sim-js
  flipY(data, shape) {
    const width = shape.get(0);
    const height = shape.get(1);
    const channels = shape.get(2);
    const numElementsPerRow = channels * width;
    for (let row = 0; row < height / 2; row++) {
      const yOut = height - row - 1;
      const base = numElementsPerRow * row;
      const baseOut = numElementsPerRow * yOut;
      for (let col = 0; col < width; col++) {
        const step = col * channels;
        const idx = base + step;
        const idxOut = baseOut + step;
        for (let i = 0; i < channels; i++) {
          const t = data[idxOut+i];
          data[idxOut+i] = data[idx+i];
          data[idx+i] = t;
        }
      }
    }
  }

  getObservation(agentId, sensorId, buffer) {
    //const mobs = new Module.MapStringObservation();
    //const okay = this.sim.getAgentObservations(agentId, mobs);
    //console.log(sensorId);
    const obs = new Module.Observation();
    const okay = this.sim.getAgentObservation(0, sensorId, obs);
    if (okay) {
      if (buffer) {
        buffer.set(obs.getBytes());
      } else {
        buffer = obs.getBytes();
      }
      this.flipY(buffer, obs.getShape());
      return Promise.resolve(buffer);
    } else {
      return Promise.reject("Cannot get observation");
    }
  }

}