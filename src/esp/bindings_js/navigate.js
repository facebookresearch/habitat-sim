class ActionQueue {
  constructor(sim, action_callback) {
    this.sim = sim;
    this.queue = async.queue((action, cb) => {
      sim.step(action)
        .then(data => 
        { 
          action_callback(action, data);
          cb(); 
        });
    }, 1);
  }

  push(action) {
    this.queue.push(action);
  }
}

class NavigateTask {
  constructor(sim, components) {
    this.sim = sim;
    this.sensorId = 'rgba_camera';
    this.components = components;
    this.actions = [
      { name: 'moveForward', key: 'w', action: 0 },
      { name: 'lookLeft', key: 'a', action: 1 },
      { name: 'lookRight', key: 'd', action: 2 },
      { name: 'done', key: ' ', action: 3 }
    ];
    this.actionQueue = new ActionQueue(sim, (action, data) => {
      this.setStatus(action.name);
      this.update();
    });
  }

  init() {
    this.bindKeys();
  }

  setStatus(text) {
    this.components.status.innerHTML = text;
  }

  render(mode, canvas) {
    const ctx = canvas.getContext("2d");
    const pspace = this.sim.getObservationSpace(this.sim.defaultAgentId, this.sensorId);
    pspace.then(space => {
      const shape = space.getShape();
      //console.log(shape.get(0), shape.get(1));
      const imageData = ctx.createImageData(shape.get(0), shape.get(1));
      this.sim.render(this.sensorId, imageData.data).then(
        data => ctx.putImageData(imageData, 0, 0) // Or at whatever offset you like
      );
    });
  }

  reset() {
    this.sim.reset().then(resp => {
      this.setStatus('Ready');
      this.render(this.sensorId, this.components.canvas);
    });
  }

  update() {
    this.render(this.sensorId, this.components.canvas);
  }

  bindKeys() {
    document.addEventListener('keyup', (event) => {
      const key = String.fromCharCode(event.which).toLowerCase();
      for (let a of this.actions) {
        if (key === a.key) {
          this.actionQueue.push(a);
          break;
        }
      }
    });
  }
}