// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module, XRWebGLLayer */

import {
  getEyeSensorSpecs,
  updateHeadPose,
  VIEW_SENSORS,
  initGL,
  drawTextureData
} from "../lib/habitat-sim-js/vr_utils.js";
import { DataUtils } from "./data_utils.js";
import { Benchmark } from "./benchmark.js";

const objectSpawnOrder = [
  "frl_apartment_vase_02", // gray
  "frl_apartment_plate_02", // double-layer
  "frl_apartment_pan_01", // blue, with handle

  "frl_apartment_kitchen_utensil_05", // serving tray
  "banana_fixed",
  "banana_fixed",
  "banana_fixed",

  "frl_apartment_plate_01",
  "frl_apartment_plate_01",

  "frl_apartment_kitchen_utensil_06", // white handleless cup
  "frl_apartment_kitchen_utensil_06", // white handleless cup

  "frl_apartment_bowl_06", // small white
  "frl_apartment_bowl_06", // small white

  "frl_apartment_kitchen_utensil_02", // green spice shaker
  "frl_apartment_kitchen_utensil_03" // orange spice shaker
];

const benchmarkObjects = [
  "frl_apartment_vase_02", // gray
  "frl_apartment_plate_02", // double-layer
  "frl_apartment_pan_01", // blue, with handle
  "frl_apartment_kitchen_utensil_05", // serving tray
  "banana_fixed",
  "frl_apartment_plate_01",
  "frl_apartment_kitchen_utensil_06", // white handleless cup
  "frl_apartment_bowl_06", // small white
  "frl_apartment_kitchen_utensil_02", // green spice shaker
  "frl_apartment_kitchen_utensil_03" // orange spice shaker
];

let preloadedFiles = [];

export class VRDemo {
  fpsElement;
  lastPaintTime;

  webXRSession = null;
  xrReferenceSpace = null;
  gl = null;
  tex = null;
  inds = null;

  prevFwdHeld = false;
  prevLeftHeld = false;
  prevRightHeld = false;

  currentFramesSkipped = 0;

  objectCounter = 0;

  constructor() {
    this.fpsElement = document.getElementById("fps");
  }

  static preloadFiles(preloadFunc) {
    function doPreload(file) {
      preloadedFiles.push("../" + file);
      preloadFunc(file);
    }

    doPreload(DataUtils.getPhysicsConfigFilepath());

    doPreload(DataUtils.getStageFilepath(Module.stageName));
    doPreload(DataUtils.getStageConfigFilepath(Module.stageName));

    doPreload(DataUtils.getObjectFilepath("hand_r_open"));
    doPreload(DataUtils.getObjectConfigFilepath("hand_r_open"));
    doPreload(DataUtils.getObjectFilepath("hand_r_closed"));
    doPreload(DataUtils.getObjectConfigFilepath("hand_r_closed"));

    doPreload(DataUtils.getObjectFilepath("hand_l_open"));
    doPreload(DataUtils.getObjectConfigFilepath("hand_l_open"));
    doPreload(DataUtils.getObjectFilepath("hand_l_closed"));
    doPreload(DataUtils.getObjectConfigFilepath("hand_l_closed"));

    const replicaCadObjectNames = new Set();
    for (const object of objectSpawnOrder) {
      replicaCadObjectNames.add(object);
    }

    for (const name of replicaCadObjectNames) {
      doPreload(DataUtils.getObjectFilepath(name));
      doPreload(DataUtils.getObjectCollisionGlbFilepath(name));
      doPreload(DataUtils.getObjectConfigFilepath(name));
    }
  }

  printedBenchmarkResults = false;

  applyKeyframe(jsonKeyframe) {
    let keyframe = this.player.keyframeFromString(jsonKeyframe);
    this.player.applyKeyframe(keyframe);
    if (this.benchmarker) {
      if (this.benchmarker.active()) {
        this.benchmarker.stepBenchmark();
      } else if (!this.printedBenchmarkResults) {
        const res = this.benchmarker.getResults();
        console.log(
          `Frame time: ${res["meanFrameTime"].toFixed(2)} +/- ${res[
            "errorFrameTime"
          ].toFixed(2)} ms`
        );
        console.log(
          `Step time: ${res["meanStepTime"].toFixed(2)} +/- ${res[
            "errorStepTime"
          ].toFixed(2)} ms`
        );
        this.printedBenchmarkResults = true;
      }
    }
  }

  start() {
    let setup = this.setup.bind(this);
    let applyKeyframe = this.applyKeyframe.bind(this);
    this.workerThread = new Worker("js/physics_worker.js");
    let preloadInfo = {
      stageFilepath: DataUtils.getStageFilepath(Module.stageName),
      physicsConfigFilepath: DataUtils.getPhysicsConfigFilepath(),
      objectBaseFilepath: DataUtils.getObjectBaseFilepath(),
      preloadedFiles: preloadedFiles
    };
    this.workerThread.postMessage({ type: "preloadInfo", value: preloadInfo });
    this.workerThread.onmessage = function(e) {
      if (e.data.type == "ready") {
        setup();
      } else if (e.data.type == "keyframe") {
        applyKeyframe(e.data.value);
      }
    };
  }

  setup() {
    // init sim
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = "NONE";
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
    this.config.allowPbrShader = false; // Pbr shader isn't robust on WebGL yet
    this.sim = new Module.Simulator(this.config);

    // init agent
    const agentConfigOrig = new Module.AgentConfiguration();
    agentConfigOrig.sensorSpecifications = getEyeSensorSpecs(1024, 1024);
    this.sim.addAgent(agentConfigOrig);
    this.agentId = 0;

    // place agent
    const agent = this.sim.getAgent(this.agentId);
    let state = new Module.AgentState();
    agent.getState(state);
    state.position = [2.0, 0.1, 5.8]; // todo: specify start position/orientation via URL param (or react?)
    state.rotation = [0.0, 0.0, 0.0, 1.0];
    agent.setState(state, false);

    // load objects from data
    Module.loadAllObjectConfigsFromPath(
      this.sim,
      DataUtils.getObjectBaseFilepath()
    );

    // set up "Enter VR" button
    const elem = document.getElementById("enter-vr");
    elem.style.visibility = "visible";
    elem.addEventListener("click", this.enterVR.bind(this));

    // create player
    this.player = this.sim.getGfxReplayManager().createEmptyPlayer();
  }

  initBenchmark() {
    const spawnFn = (objectIdx, pos, vel) => {
      const handle = DataUtils.getObjectConfigFilepath(
        benchmarkObjects[objectIdx]
      );
      let arrpos = Module.toVec3f(pos);
      let arrvel = Module.toVec3f(vel);
      let spawnInfo = { handle: handle, pos: arrpos, vel: arrvel };
      this.workerThread.postMessage({ type: "spawn", value: spawnInfo });
    };
    const deleteFn = () => {
      this.workerThread.postMessage({ type: "delete", value: null });
    };
    const moveFn = (pos, rot) => {
      const agent = this.sim.getAgent(this.agentId);
      let state = new Module.AgentState();
      agent.getState(state);
      state.position = pos;
      state.rotation = rot;
      agent.setState(state, false);
    };
    this.benchmarker = new Benchmark(
      benchmarkObjects.length,
      spawnFn,
      deleteFn
    );
    this.benchmarker.spawnPos = new Module.Vector3(2, 2, 2);
    this.benchmarker.spawnPosJitter = 0.2;
    this.benchmarker.spawnVel = new Module.Vector3(0, -10, 0);

    this.benchmarker.numIterations = 5;
    this.benchmarker.objectsPerIteration = 5;

    this.benchmarker.stepsBetweenSpawn = 8;
    this.benchmarker.stepsBeforeDelete = 100;

    this.benchmarker.moveFn = moveFn;
    this.benchmarker.viewOffsetY = -2.5;
    this.benchmarker.moveRadius = 2.0;
    this.benchmarker.rotationRate = 0.05;

    this.benchmarker.start();
  }

  async enterVR() {
    if (this.gl === null) {
      this.gl = document.createElement("canvas").getContext("webgl", {
        xrCompatible: true
      });
      initGL(this.gl);
    }
    this.webXRSession = await navigator.xr.requestSession("immersive-vr", {
      requiredFeatures: ["local-floor"]
    });

    this.webXRSession.addEventListener("end", () => {
      this.webXRSession = null;
      this.exitVR();
    });

    const xrLayer = new XRWebGLLayer(this.webXRSession, this.gl);
    this.webXRSession.updateRenderState({ baseLayer: xrLayer });
    this.xrReferenceSpace = await this.webXRSession.requestReferenceSpace(
      "local-floor"
    );

    this.webXRSession.requestAnimationFrame(this.drawVRScene.bind(this));

    const handFilepathsByHandIndex = [
      [
        DataUtils.getObjectConfigFilepath("hand_l_open"),
        DataUtils.getObjectConfigFilepath("hand_l_closed")
      ],
      [
        DataUtils.getObjectConfigFilepath("hand_r_open"),
        DataUtils.getObjectConfigFilepath("hand_r_closed")
      ]
    ];
    const spawnOrder = [];
    for (const handle of objectSpawnOrder) {
      spawnOrder.push(DataUtils.getObjectConfigFilepath(handle));
    }
    const startData = {
      handFilepathsByHandIndex: handFilepathsByHandIndex,
      objectSpawnOrder: spawnOrder
    };
    this.workerThread.postMessage({ type: "start", value: startData });

    let lastFPSUpdateTime = performance.now();
    let overallFPS = 20;
    setInterval(() => {
      let curTime = performance.now();
      let timeElapsed = (curTime - lastFPSUpdateTime) / 1000.0;
      let curFPS = this.currentFramesSkipped / timeElapsed;
      overallFPS = 0.8 * overallFPS + 0.2 * curFPS;

      this.fpsElement.innerHTML = `FPS: ${overallFPS.toFixed(2)}`;

      lastFPSUpdateTime = curTime;
      this.currentFramesSkipped = 0;
    }, 100.0);

    this.fpsElement.style.visibility = "visible";

    if (Module.doBenchmarking) {
      this.initBenchmark();
    }
  }

  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
      this.fpsElement.style.visibility = "hidden";
    }
  }

  handleInput(frame) {
    let handInfo = [];
    for (let inputSource of frame.session.inputSources) {
      if (!inputSource.gripSpace) {
        continue;
      }
      let handIndex = inputSource.handedness == "left" ? 0 : 1;

      const agent = this.sim.getAgent(this.agentId);
      let state = new Module.AgentState();
      agent.getState(state);
      let agentPos = new Module.Vector3(...state.position);

      // 6DoF pose example
      const inputPose = frame.getPose(
        inputSource.gripSpace,
        this.xrReferenceSpace
      );

      let gp = inputSource.gamepad;
      let buttonStates = [false, false];
      for (let i = 0; i < gp.buttons.length; i++) {
        // Not sure what all these buttons are. Let's just use two.
        let remappedIndex = i == 0 ? 0 : 1;
        buttonStates[remappedIndex] ||=
          gp.buttons[i].value > 0 || gp.buttons[i].pressed == true;
      }

      const pointToArray = p => [p.x, p.y, p.z, p.w];

      // update hand obj pose
      let poseTransform = inputPose.transform;
      const handPos = Module.Vector3.add(
        new Module.Vector3(
          ...pointToArray(poseTransform.position).slice(0, -1)
        ),
        agentPos
      );

      let handRot = Module.toQuaternion(
        pointToArray(poseTransform.orientation)
      );

      handInfo.push({
        index: handIndex,
        pos: Module.toVec3f(handPos),
        rot: Module.toVec4f(handRot),
        gripButton: buttonStates[0],
        spawnButton: buttonStates[1]
      });
    }
    this.workerThread.postMessage({ type: "hands", value: handInfo });
  }

  drawVRScene(t, frame) {
    const session = frame.session;

    session.requestAnimationFrame(this.drawVRScene.bind(this));

    const pose = frame.getViewerPose(this.xrReferenceSpace);

    // Need the pose to render a frame
    if (!pose) {
      return;
    }

    const agent = this.sim.getAgent(this.agentId);

    this.handleInput(frame);
    if (!this.benchmarker || !this.benchmarker.active()) {
      updateHeadPose(pose, agent);
    }

    const layer = session.renderState.baseLayer;
    this.gl.bindFramebuffer(this.gl.FRAMEBUFFER, layer.framebuffer);

    for (var iView = 0; iView < pose.views.length; ++iView) {
      const view = pose.views[iView];
      const viewport = layer.getViewport(view);
      this.gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);

      const sensor = agent.getSubtreeSensors().get(VIEW_SENSORS[iView]);
      const texRes = sensor.specification().resolution;
      const texData = sensor.getObservation(this.sim).getData();
      drawTextureData(this.gl, texRes, texData);
    }

    this.currentFramesSkipped++;
    if (this.benchmarker && this.benchmarker.active()) {
      this.benchmarker.logFrame();
    }
  }
}
