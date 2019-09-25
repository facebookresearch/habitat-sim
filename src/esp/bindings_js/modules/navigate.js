/**
 * NavigateTask class
 */
class NavigateTask {
  // PUBLIC methods.

  /**
   * Create navigate task.
   * @param {SimEnv} sim - simulator
   * @param {TopDownMap} topdown - TopDown Map
   * @param {Object} components - dictionary with status and canvas elements
   */
  constructor(sim, topdown, components) {
    this.sim = sim;
    this.topdown = topdown;
    this.components = components;
    let shape = this.sim.getObservationSpace("rgb").shape;
    this.semanticCtx = components.semantic.getContext("2d");
    shape = this.sim.getObservationSpace("semantic").shape;
    this.semanticImageData = this.semanticCtx.createImageData(
      shape.get(1),
      shape.get(0)
    );
    this.semanticObjects = this.sim.sim.getSemanticScene().objects;
    components.canvas.onmousedown = e => {
      this.handleMouseDown(e);
    };
    this.radarCtx = components.radar.getContext("2d");
    this.actions = [
      { name: "moveForward", key: "w" },
      { name: "lookLeft", key: "a" },
      { name: "lookRight", key: "d" },
      { name: "done", key: " " }
    ];
  }

  handleMouseDown(event) {
    let objectId = this.semantic_data[
      (640 * event.offsetY + event.offsetX) * 4
    ];
    this.setStatus(this.semanticObjects.get(objectId).category.getName(""));
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
    this.setStatus("Ready");
    this.render();
  }

  // PRIVATE methods.

  setStatus(text) {
    this.components.status.innerHTML = text;
  }

  renderImage() {
    this.sim.displayObservation("rgb");
    this.renderRadar();
  }

  renderSemanticImage() {
    if (this.semanticObjects.size() == 0) {
      return;
    }

    const obs = this.sim.getObservation("semantic", null);
    this.semantic_data = obs.getData();
    let data = this.semantic_data;

    // TOOD(msb) implement a better colorization scheme
    for (let i = 0; i < 640 * 480; i++) {
      if (data[i * 4] & 1) {
        this.semanticImageData.data[i * 4] = 255;
      } else {
        this.semanticImageData.data[i * 4] = 0;
      }
      if (data[i * 4] & 2) {
        this.semanticImageData.data[i * 4 + 1] = 255;
      } else {
        this.semanticImageData.data[i * 4 + 1] = 0;
      }
      if (data[i * 4] & 4) {
        this.semanticImageData.data[i * 4 + 2] = 255;
      } else {
        this.semanticImageData.data[i * 4 + 2] = 0;
      }
      this.semanticImageData.data[i * 4 + 3] = 255;
    }

    this.semanticCtx.putImageData(this.semanticImageData, 0, 0);
  }

  renderTopDown() {
    this.topdown.moveTo(this.sim.getAgentState().position);
  }

  renderRadar() {
    const width = 100,
      height = 100;
    let radius = width / 2;
    let centerX = width / 2;
    let centerY = height / 2;
    let ctx = this.radarCtx;
    ctx.clearRect(0, 0, width, height);
    ctx.globalAlpha = 0.5;
    // draw circle
    ctx.fillStyle = "darkslategray";
    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
    ctx.fill();
    // draw sector
    ctx.fillStyle = "darkgray";
    ctx.beginPath();
    // TODO(msb) Currently 90 degress but should really use fov.
    ctx.arc(centerX, centerY, radius, (-Math.PI * 3) / 4, -Math.PI / 4);
    ctx.lineTo(centerX, centerY);
    ctx.closePath();
    ctx.fill();
    // draw target
    ctx.globalAlpha = 1.0;
    ctx.beginPath();
    let magnitude, angle;
    [magnitude, angle] = this.sim.distanceToGoal();
    let normalized = magnitude / (magnitude + 1);
    let targetX = centerX + Math.sin(angle) * radius * normalized;
    let targetY = centerY - Math.cos(angle) * radius * normalized;
    ctx.fillStyle = "maroon";
    ctx.arc(targetX, targetY, 3, 0, 2 * Math.PI);
    ctx.fill();
  }

  render() {
    this.renderImage();
    this.renderSemanticImage();
    this.renderTopDown();
  }

  handleAction(action) {
    this.sim.step(action);
    this.setStatus(action);
    this.render();
  }

  bindKeys() {
    document.addEventListener("keyup", event => {
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

export default NavigateTask;
