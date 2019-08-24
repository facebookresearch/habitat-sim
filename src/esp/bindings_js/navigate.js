/**
 * NavigateTask class
 */
class NavigateTask {
  // PUBLIC methods.

  /**
   * Create navigate task.
   * @param {SimEnv} sim - simulator
   * @param {Object} components - dictionary with status and canvas elements
   */
  constructor(sim, components) {
    this.sim = sim;
    this.sensorId = 'rgba_camera';
    this.components = components;
    this.ctx = components.canvas.getContext("2d");
    const shape = this.sim.getObservationSpace(this.sensorId).shape;
    this.imageData = this.ctx.createImageData(shape.get(1), shape.get(0));
    this.actions = [
      { name: 'moveForward', key: 'w' },
      { name: 'lookLeft', key: 'a', },
      { name: 'lookRight', key: 'd' },
      { name: 'done', key: ' ' }
    ];
  }

  /**
   * Initialize the task. Should be called once.
   */
  init() {
    this.bindKeys();
  }

  /**
   * Reset the task.
   */
  reset() {
    this.sim.reset();
    this.setStatus('Ready');
    this.render();
  }

  // PRIVATE methods.

  setStatus(text) {
    this.components.status.innerHTML = text;
  }

  applyGamma(data, gamma) {
    for (let i = 0; i < data.length; i++) {
      data[i] = Math.pow(data[i]/255.0, gamma) * 255;
    }
  }

  render() {
    const obs = this.sim.getObservation(this.sensorId, null);
    this.imageData.data.set(obs.getData());
    // convert from linear to sRGB gamma
    this.applyGamma(this.imageData.data, 2.2);
    this.ctx.putImageData(this.imageData, 0, 0);
  }

  handleAction(action) {
    this.sim.step(action);
    this.setStatus(action);
    this.render();
  }

  bindKeys() {
    document.addEventListener('keyup', (event) => {
      const key = String.fromCharCode(event.which).toLowerCase();
      for (let a of this.actions) {
        if (key === a.key) {
          this.handleAction(a.name);
          break;
        }
      }
    });
  }
}
