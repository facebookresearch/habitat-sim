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
    this.imageCtx = components.canvas.getContext("2d");
    const shape = this.sim.getObservationSpace(this.sensorId).shape;
    this.imageData = this.imageCtx.createImageData(shape.get(1), shape.get(0));
    this.radarCtx = components.radar.getContext("2d");
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

  renderImage() {
    const obs = this.sim.getObservation(this.sensorId, null);
    this.imageData.data.set(obs.getData());
    // convert from linear to sRGB gamma
    this.applyGamma(this.imageData.data, 2.2);
    this.imageCtx.putImageData(this.imageData, 0, 0);
    this.renderRadar();
  }

  renderRadar() {
    const width = 100, height = 100;
    let radius = width/2;
    let centerX = width/2;
    let centerY = height/2;
    let ctx = this.radarCtx;
    ctx.clearRect(0, 0, width, height);
    ctx.globalAlpha = 0.5;
    // draw circle
    ctx.fillStyle = "darkslategray";
    ctx.arc(centerX, centerY, radius, 0, 2*Math.PI);
    ctx.fill();
    // draw sector
    ctx.fillStyle = "darkgray";
    ctx.beginPath();
    // TODO(msb) Currently 90 degress but should really use fov.
    ctx.arc(centerX, centerY, radius, -Math.PI*3/4, -Math.PI/4);
    ctx.lineTo(centerX, centerY);
    ctx.closePath();
    ctx.fill();
    // draw target
    ctx.globalAlpha = 1.0;
    ctx.beginPath();
    let magnitude, angle;
    [magnitude, angle] = this.sim.distanceToGoal();
    let normalized = magnitude/(magnitude+1);
    let targetX = centerX + Math.sin(angle)*radius*normalized;
    let targetY = centerY - Math.cos(angle)*radius*normalized;
    ctx.fillStyle = "maroon";
    ctx.arc(targetX, targetY, 3, 0, 2*Math.PI);
    ctx.fill();
  }

  render() {
    this.renderImage();
    this.renderRadar();
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
