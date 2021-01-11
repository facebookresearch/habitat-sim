// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/*global Module */
import SimEnv from "./simenv_embind";

/**
 * ReplayViewerTask class
 */
class ReplayViewerTask {
  // PUBLIC methods.

  /**
   * Create ReplayViewer task.
   * @param {SimEnv} sim - simulator
   * @param {Object} components - dictionary with status and canvas elements
   */
  constructor(sim, components) {
    this.sim = sim;
    this.components = components;

    this.actions = [
      { name: "moveForward", key: "w", keyCode: 87 },
      { name: "moveBackward", key: "s", keyCode: 83 },
      { name: "strafeLeft", key: "a", keyCode: 65 },
      { name: "strafeRight", key: "d", keyCode: 68 },
      { name: "turnLeft", key: "ArrowLeft", keyCode: 37 },
      { name: "turnRight", key: "ArrowRight", keyCode: 39 },
      { name: "lookUp", key: "ArrowUp", keyCode: 38 },
      { name: "lookDown", key: "ArrowDown", keyCode: 40 }
    ];
  }

  /**
   * Initialize the task. Should be called once.
   */
  init() {
    this.bindKeys();

    this.activeActions = [];
    this.isPaused = false;

    this.initAgentState();

    this.frameCounter = 0;
    this.replayStepFunction = setInterval(() => {
      if (this.player) {
        if (!this.isPaused) {
          var currIndex = this.player.getKeyframeIndex();
          var newIndex =
            currIndex == this.player.numKeyframes - 1 ? 0 : currIndex + 1;
          this.player.setKeyframeIndex(newIndex);
        }
      }

      for (let action of this.activeActions) {
        this.handleAction(action);
      }

      this.render();

      // Some reference code for recording a replay. This doesn't make sense for
      // a replay viewer, but it could be used to record a human demonstration
      // along with any physics simulation you're running.
      // let state = this.sim.getAgentState();
      // this.sim.sim
      //   .getGfxReplayManager()
      //   .addUserTransformToKeyframe("agent", state.position, state.rotation);
      // this.sim.sim.getGfxReplayManager().saveKeyframe();
      // if (isEndOfEpisode) {
      //   let replayStr = this.sim.sim
      //     .getGfxReplayManager()
      //     .writeSavedKeyframesToString();
      //   // send replayStr to psiturk or otherwise save it
      // }

      this.frameCounter += 1;
    }, 1000.0 / 60);

    this.numRendersSinceLastUpdate = 0;
    this.updateFpsFunction = setInterval(() => {
      var duration;
      var fps_desc;
      if (window.performance.now) {
        var timestamp = window.performance.now();
        if (this.lastTimestamp) {
          duration = (timestamp - this.lastTimestamp) / 1000.0;
        }
        this.lastTimestamp = timestamp;
        fps_desc = "FPS";
      } else {
        duration = 0.5;
        fps_desc = "APPROX FPS";
      }

      this.components.fps.innerHTML =
        fps_desc +
        " " +
        (this.numRendersSinceLastUpdate / duration).toFixed(1).toString();
      this.numRendersSinceLastUpdate = 0;
    }, 500.0);

    var filepath = "physics_demo.gfx_replay.json";
    this.player = this.sim.sim
      .getGfxReplayManager()
      .readKeyframesFromFile(filepath);
    if (!this.player) {
      this.setErrorStatus("failed to load replay");
    } else {
      this.setStatus("");
    }
    if (this.player) {
      this.player.setKeyframeIndex(0);
    }
  }

  // PRIVATE methods.

  setStatus(text) {
    this.components.status.style = "color:white";
    this.components.status.innerHTML = text;
  }

  setErrorStatus(text) {
    this.components.status.style = "color:red";
    this.components.status.innerHTML = text;
  }

  setWarningStatus(text) {
    this.components.status.style = "color:orange";
    this.components.status.innerHTML = text;
  }

  setSuccessStatus(text) {
    this.components.status.style = "color:green";
    this.components.status.innerHTML = text;
  }

  renderImage() {
    this.sim.displayObservation("rgb");
  }

  render() {
    this.renderImage();
    this.numRendersSinceLastUpdate += 1;
  }

  removeActiveAction(action) {
    const index = this.activeActions.indexOf(action);
    if (index > -1) {
      this.activeActions.splice(index, 1);
    }
  }

  syncAgentRotation() {
    let state = this.sim.getAgentState();

    state.rotation = Module.quaternionMultiply(
      Module.eulerToQuaternion([0, this.agentRotation, 0]),
      Module.eulerToQuaternion([this.agentLookPitch, 0, 0])
    );

    this.sim.setAgentState(state);
  }

  initAgentState() {
    let state = this.sim.getAgentState();
    state.position = [-3.01, 0, 5.92];
    this.sim.setAgentState(state);

    this.agentRotation = 3.5;
    this.agentLookPitch = -0.21;
    this.syncAgentRotation();
  }

  handleAction(action) {
    if (
      action == "moveForward" ||
      action == "moveBackward" ||
      action == "strafeLeft" ||
      action == "strafeRight"
    ) {
      let stepDist = 0.06;
      let stepSignedDist;
      let adjustedRotation;
      if (action == "moveForward" || action == "moveBackward") {
        stepSignedDist = action == "moveForward" ? stepDist : -stepDist;
        adjustedRotation = -this.agentRotation - Math.PI * 0.5;
      } else {
        stepSignedDist = action == "strafeLeft" ? stepDist : -stepDist;
        adjustedRotation = -this.agentRotation - Math.PI * 1.0;
      }
      let state = this.sim.getAgentState();
      state.position = [
        state.position[0] + Math.cos(adjustedRotation) * stepSignedDist,
        state.position[1],
        state.position[2] + Math.sin(adjustedRotation) * stepSignedDist
      ];
      this.sim.setAgentState(state);
    } else if (action == "turnLeft" || action == "turnRight") {
      let rotDist = 0.035;
      let rotSignedDist = action == "turnLeft" ? rotDist : -rotDist;
      this.agentRotation += rotSignedDist;
      this.syncAgentRotation();
    } else if (action == "lookUp" || action == "lookDown") {
      let rotDist = 0.015;
      let MIN_LOOK_PITCH = -Math.PI * 0.2;
      let MAX_LOOK_PITCH = Math.PI * 0.2;
      let rotSignedDist = action == "lookUp" ? rotDist : -rotDist;
      this.agentLookPitch += rotSignedDist;
      this.agentLookPitch = Math.min(
        Math.max(this.agentLookPitch, MIN_LOOK_PITCH),
        MAX_LOOK_PITCH
      );
      this.syncAgentRotation();
    }
  }

  handleKeypressUp(key) {
    for (let a of this.actions) {
      if (a.keyCode === key) {
        this.removeActiveAction(a.name);
        break;
      }
    }
  }

  handleKeypress(key, isRepeat) {
    for (let a of this.actions) {
      if (a.keyCode === key) {
        const index = this.activeActions.indexOf(a.name);
        if (index == -1) {
          this.activeActions.push(a.name);
        }
        return true;
      }
    }

    if (isRepeat) {
      return false;
    }

    if (key == " ".charCodeAt(0)) {
      this.isPaused = !this.isPaused;
    } else if (key == "R".charCodeAt(0)) {
      if (this.player) {
        var currIndex = this.player.getKeyframeIndex();
        var newIndex = Math.max(0, currIndex - 10);
        this.player.setKeyframeIndex(newIndex);
        this.isPaused = true; // pause after rewind
      }
    } else if (key == "L".charCodeAt(0)) {
      this.components.logWrapper.style.display =
        this.components.logWrapper.style.display == "block" ? "none" : "block";
    }
  }

  bindKeys() {
    var _self = this;
    _self.keyBindListener = function(event) {
      if (_self.handleKeypress(event.keyCode, event.repeat)) {
        event.preventDefault();
      }
    };
    document.addEventListener("keydown", _self.keyBindListener, true);

    _self.keyBindListener2 = function(event) {
      _self.handleKeypressUp(event.keyCode);
    };
    document.addEventListener("keyup", _self.keyBindListener2, true);
  }
}

class ReplayViewer {
  static preloadFiles(preloadFunc) {
    // For dataUrlBase, if you specify a relative path here like "data", it is relative
    // to the bindings_js folder where index.js is served. You can alternately specify a
    // full URL base such as https://www.mywebsite.com/data". For local testing, note
    // that we symlink your habitat-sim data directory (habitat-sim/data) to
    // /build_js/esp/bindings_js/data. (See "resources" in
    // src/esp/bindings_js/CMakeLists.txt.)
    let dataUrlBase = "data";
    preloadFunc(
      dataUrlBase + "/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
      false
    );
    preloadFunc(dataUrlBase + "/replays/physics_demo.gfx_replay.json", false);
    preloadFunc(dataUrlBase + "/objects/cheezit.glb", false);
    preloadFunc(dataUrlBase + "/objects/skillet.glb", false);
    preloadFunc(dataUrlBase + "/objects/chefcan.glb", false);
    preloadFunc(dataUrlBase + "/objects/banana.glb", false);
  }

  currentResolution = { height: 600, width: 800 };
  constructor(canvasId = "canvas") {
    this.canvasId = canvasId;
  }

  initializeModules() {
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = "NONE"; // see Asset.h EMPTY_SCENE
    this.simenv = new SimEnv(this.config);

    // A lot of these properties don't matter because we aren't moving the agent
    // through the agent API.
    let agentConfig = {
      height: 1.5,
      radius: 0.1,
      mass: 32.0,
      linearAcceleration: 20.0,
      angularAcceleration: 4 * Math.PI,
      linearFriction: 0.5,
      angularFriction: 1.0,
      coefficientOfRestitution: 0.0
    };

    agentConfig = this.updateAgentConfigWithSensors(agentConfig);

    this.simenv.addAgent(agentConfig);

    this.canvasElement = document.getElementById(this.canvasId);

    this.task = new ReplayViewerTask(this.simenv, {
      topdown: null,
      canvas: this.canvasElement,
      status: document.getElementById("status"),
      fps: document.getElementById("fps"),
      logWrapper: document.getElementById("log_wrapper")
    });
  }

  updateAgentConfigWithSensors(agentConfig) {
    const sensorConfigs = [
      {
        uuid: "rgb",
        sensorType: Module.SensorType.COLOR
      }
    ];

    agentConfig.sensorSpecifications = sensorConfigs;
    agentConfig = this.updateAgentConfigWithResolution(agentConfig);

    return agentConfig;
  }

  resetCanvas(resolution) {
    this.canvasElement.width = resolution.width;
    this.canvasElement.height = resolution.height;
  }

  updateAgentConfigWithResolution(agentConfig) {
    agentConfig.sensorSpecifications.forEach(sensorConfig => {
      if (sensorConfig.resolution === undefined) {
        sensorConfig.resolution = [
          this.currentResolution.height,
          this.currentResolution.width
        ];
      }
    });

    return agentConfig;
  }

  display() {
    this.initializeModules();
    this.task.init();
  }
}

export default ReplayViewer;
export { ReplayViewerTask };
