// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
import { preload } from "../lib/habitat-sim-js/utils.js";

// Objects will be spawned in this order when user presses the spawn button. At
// the end it will loop back to the beginning.
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

  // Sets up the virtual file system so that these files can be loaded with
  // their respective file paths. This is called before start().
  preloadFiles(preloadFunc) {
    let preloadedFiles = [];
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

    this.preloadedFiles = preloadedFiles;
  }

  // Entry point for all the hand demo logic. Starts up the worker thread, then
  // calls setUpHabitat.
  start() {
    // Initiate worker thread.
    this.workerThread = new Worker("js/physics_worker_setup.js");
    // Tell worker what files it needs to preload.
    let preloadInfo = {
      physicsConfigFilepath: DataUtils.getPhysicsConfigFilepath(),
      stageFilepath: DataUtils.getStageFilepath(Module.stageName),
      objectBaseFilepath: DataUtils.getObjectBaseFilepath(),
      preloadedFiles: this.preloadedFiles,
      // Hack: we convert the 'preload' function into a string in order to pass
      // it to the worker.
      preloadFunc: preload.toString()
    };
    this.workerThread.postMessage({ type: "preloadInfo", value: preloadInfo });

    // Worker will tell us when it's "ready", which means it can start stepping
    // world as soon as we give it the "start" message. Then it will start
    // sending keyframes, which tell us, physics frame by physics frame, the
    // objects that were created, destroyed, and moved since the last step.
    let setUpHabitat = this.setUpHabitat.bind(this);
    let appendKeyframe = this.appendKeyframe.bind(this);
    this.workerThread.onmessage = function(e) {
      if (e.data.type == "ready") {
        // When the worker is ready, set up Habitat stuff and create the
        // "Enter VR" button.
        setUpHabitat();
      } else if (e.data.type == "keyframe") {
        // Whenever we receive a keyframe from the worker, we call
        // this.pushKeyframe() to give it to the player to process.
        appendKeyframe(e.data.value);
      } else {
        console.assert(false); // this should be unreachable
      }
    };
  }

  // Gives the player a keyframe to append.
  appendKeyframe(jsonKeyframe) {
    this.player.appendJSONKeyframe(jsonKeyframe);
  }

  // Initiate simulator, put an agent into the scene, and create "Enter VR"
  // button.
  setUpHabitat() {
    // init sim
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = "NONE";
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
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

    // create player that reads in keyframes and "plays" them back by
    // interacting with the Simulator
    this.player = this.sim.getGfxReplayManager().createEmptyPlayer();

    // set up "Enter VR" button
    const elem = document.getElementById("enter-vr");
    elem.style.visibility = "visible";
    elem.addEventListener("click", this.enterVR.bind(this));
  }

  // Code that runs when the user clicks "Enter VR".
  async enterVR() {
    // WebXR setup stuff.
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

    // Tell WebXR to call this.drawVRScene() to draw the next frame.
    this.webXRSession.requestAnimationFrame(this.drawVRScene.bind(this));

    // Prepare to tell the worker thread the hand filepaths and what objects
    // to spawn.
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
    // Tell the worker thread that we are ready to go. Then it will start
    // stepping physics.
    this.workerThread.postMessage({ type: "start", value: startData });

    // Code to compute FPS and update the indicator on the top right.
    // Only visible in Chrome WebXR emulator mode (not in headset).
    // todo: actually draw the FPS on the canvas.
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
  }

  // Code that runs when the user exits Immersive Mode.
  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
      this.fpsElement.style.visibility = "hidden";
    }
  }

  // Processes the hand pose and sends the relevant information to the worker.
  sendHandInfoToWorker(frame) {
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

  // WebXR calls this function, which renders frames to the canvas.
  drawVRScene(t, frame) {
    const session = frame.session;

    // Tells WebXR to call drawVRScene again when we are ready to draw another
    // frame.
    session.requestAnimationFrame(this.drawVRScene.bind(this));

    const pose = frame.getViewerPose(this.xrReferenceSpace);

    // Need the pose to render a frame
    if (!pose) {
      return;
    }

    const agent = this.sim.getAgent(this.agentId);

    // Set the player to the last (most recent) keyframe. This will iterate
    // through the new keyframes that were sent by the worker and apply them
    // one-by-one.
    this.player.setKeyframeIndex(this.player.getNumKeyframes() - 1);

    // Tell the worker thread the current hand positions and spawn/grip button
    // states (pressed or not pressed).
    this.sendHandInfoToWorker(frame);

    // Update the camera positions based on the user's head pose.
    updateHeadPose(pose, agent);

    // Draw stuff to the canvas.
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

    // Used to compute FPS.
    this.currentFramesSkipped++;
  }
}
