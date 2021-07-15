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

const pointToArray = p => [p.x, p.y, p.z, p.w];

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

class HandRecord {
  objIds = [];
  prevButtonStates = [false, false];
  heldObjId = -1;
}

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

  fps = 0;
  currentFramesSkipped = 0;

  constructor() {
    this.fpsElement = document.getElementById("fps");
  }

  static preloadFiles(preloadFunc) {
    preloadFunc(DataUtils.getPhysicsConfigFilepath());

    preloadFunc(DataUtils.getStageFilepath(Module.stageName));
    preloadFunc(DataUtils.getStageConfigFilepath(Module.stageName));

    preloadFunc(DataUtils.getObjectFilepath("hand_r_open"));
    preloadFunc(DataUtils.getObjectConfigFilepath("hand_r_open"));
    preloadFunc(DataUtils.getObjectFilepath("hand_r_closed"));
    preloadFunc(DataUtils.getObjectConfigFilepath("hand_r_closed"));

    preloadFunc(DataUtils.getObjectFilepath("hand_l_open"));
    preloadFunc(DataUtils.getObjectConfigFilepath("hand_l_open"));
    preloadFunc(DataUtils.getObjectFilepath("hand_l_closed"));
    preloadFunc(DataUtils.getObjectConfigFilepath("hand_l_closed"));

    const replicaCadObjectNames = new Set();
    for (const object of objectSpawnOrder) {
      replicaCadObjectNames.add(object);
    }

    for (const name of replicaCadObjectNames) {
      preloadFunc(DataUtils.getObjectFilepath(name));
      preloadFunc(DataUtils.getObjectCollisionGlbFilepath(name));
      preloadFunc(DataUtils.getObjectConfigFilepath(name));
    }
  }

  initSimAndSensors() {
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = DataUtils.getStageFilepath(Module.stageName);
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = DataUtils.getPhysicsConfigFilepath();
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
    this.config.allowPbrShader = false; // Pbr shader isn't robust on WebGL yet

    this.sim = new Module.Simulator(this.config);

    const agentConfigOrig = new Module.AgentConfiguration();
    agentConfigOrig.sensorSpecifications = getEyeSensorSpecs(1024, 1024);

    this.sim.addAgent(agentConfigOrig);
    this.agentId = 0;
  }

  // place agent and add objects
  initScene() {
    // Set agent to identity transform.
    const agent = this.sim.getAgent(this.agentId);
    let state = new Module.AgentState();
    agent.getState(state);
    // todo: specify start position/orientation via URL param (or react?)
    state.position = [2.0, 0.1, 5.8];
    state.rotation = [0.0, 0.0, 0.0, 1.0];
    agent.setState(state, false);

    Module.loadAllObjectConfigsFromPath(
      this.sim,
      DataUtils.getObjectBaseFilepath()
    );

    this.handRecords = [new HandRecord(), new HandRecord()];

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

    for (const handIndex of [0, 1]) {
      for (const filepath of handFilepathsByHandIndex[handIndex]) {
        let objId = this.sim.addObjectByHandle(filepath, null, "", 0);
        this.sim.setObjectMotionType(Module.MotionType.KINEMATIC, objId, 0);
        this.sim.setTranslation(new Module.Vector3(0.0, 0.0, 0.0), objId, 0);
        this.handRecords[handIndex].objIds.push(objId);
      }
    }

    this.objectCounter = 0;
  }

  setUpVR() {
    const elem = document.getElementById("enter-vr");
    elem.style.visibility = "visible";
    elem.addEventListener("click", this.enterVR.bind(this));
  }

  display() {
    this.initSimAndSensors();
    this.initScene();
    this.setUpVR();

    this.headPosesInputElement = document.getElementById("head_poses_input");
  }

  initBenchmark() {
    const spawnFn = (objectIdx, pos, vel) => {
      const handle = DataUtils.getObjectConfigFilepath(
        benchmarkObjects[objectIdx]
      );
      const objId = this.sim.addObjectByHandle(handle, null, "", 0);
      this.sim.setTranslation(pos, objId, 0);
      this.sim.setLinearVelocity(vel, objId, 0);
      return objId;
    };
    const deleteFn = objIds => {
      for (const objId of objIds) {
        this.sim.removeObject(objId, true, true, 0);
      }
    };
    this.benchmarker = new Benchmark(
      benchmarkObjects.length,
      spawnFn,
      deleteFn
    );
    this.benchmarker.spawnPos = new Module.Vector3(2, 2, 2);
    this.benchmarker.spawnPosJitter = 0.2;
    this.benchmarker.start();
  }

  async enterVR() {
    if (this.gl === null) {
      this.gl = document.createElement("canvas").getContext("webgl", {
        xrCompatible: true
      });
      initGL(this.gl);
      console.log("Initialized WebXR GL state");
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

    this.renderDisplay();

    let printedBenchmarkResults = false;
    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(1.0 / 60);
      if (this.benchmarker) {
        if (this.benchmarker.active()) {
          this.benchmarker.stepBenchmark();
        } else if (!printedBenchmarkResults) {
          const res = this.benchmarker.getResults();
          console.log("Average frame time:", res[0]);
          console.log("Average step time:", res[1]);
          printedBenchmarkResults = true;
        }
      }
    }, 1000.0 / 60);

    this.fpsUpdateFunction = setInterval(() => {
      this.updateFPS();
    }, 100.0);

    this.fpsElement.style.visibility = "visible";

    this.initBenchmark();
  }

  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
      this.fpsElement.style.visibility = "hidden";
    }
  }

  renderDisplay() {
    if (this.webXRSession !== null) {
      this.webXRSession.requestAnimationFrame(this.drawVRScene.bind(this));
    } else {
      window.setTimeout(this.renderDisplay.bind(this), 1000);
    }
  }

  handleInput(frame) {
    for (let inputSource of frame.session.inputSources) {
      if (!inputSource.gripSpace) {
        continue;
      }
      let handIndex = inputSource.handedness == "left" ? 0 : 1;
      let otherHandIndex = handIndex == 0 ? 1 : 0;
      let handRecord = this.handRecords[handIndex];
      let otherHandRecord = this.handRecords[otherHandIndex];

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
      let closed = buttonStates[0];
      let handObjId = closed ? handRecord.objIds[1] : handRecord.objIds[0];
      let hiddenHandObjId = closed
        ? handRecord.objIds[0]
        : handRecord.objIds[1];

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
      this.sim.setTranslation(handPos, handObjId, 0);
      this.sim.setRotation(handRot, handObjId, 0);

      // hack hide other hand by translating far away
      this.sim.setTranslation(
        new Module.Vector3(-1000.0, -1000.0, -1000.0),
        hiddenHandObjId,
        0
      );

      let palmFacingSign = handIndex == 0 ? 1.0 : -1.0;
      let palmFacingDir = handRot.transformVector(
        new Module.Vector3(palmFacingSign, 0.0, 0.0)
      );
      let grabRay = new Module.Ray(handPos, palmFacingDir);

      // try grab
      if (buttonStates[0] && !handRecord.prevButtonStates[0]) {
        let maxDistance = 0.15;

        let raycastResults = this.sim.castRay(grabRay, maxDistance, 0);
        let hitObjId = raycastResults.hasHits()
          ? raycastResults.hits.get(0).objectId
          : -1;
        console.log("Try grab", hitObjId);

        if (hitObjId != -1) {
          handRecord.heldObjId = hitObjId;

          if (otherHandRecord.heldObjId == hitObjId) {
            // release from other hand
            otherHandRecord.heldObjId = -1;
          }

          let currTrans = this.sim.getTranslation(handRecord.heldObjId, 0);
          let currRot = this.sim.getRotation(handRecord.heldObjId, 0);

          let handRotInverted = handRot.inverted();
          handRecord.heldRelRot = Module.Quaternion.mul(
            handRotInverted,
            currRot
          );
          handRecord.heldRelTrans = handRotInverted.transformVector(
            Module.Vector3.sub(currTrans, handPos)
          );

          // set held obj to kinematic
          this.sim.setObjectMotionType(
            Module.MotionType.KINEMATIC,
            handRecord.heldObjId,
            0
          );
        }
      }

      // update held object pose
      if (handRecord.heldObjId != -1) {
        let pad =
          Math.min(0.5, Math.max(0.3, palmFacingDir.y())) *
          0.05 *
          palmFacingSign;
        let adjustedRelTrans = Module.Vector3.add(
          handRecord.heldRelTrans,
          new Module.Vector3(pad, 0.0, 0.0)
        );

        this.sim.setTranslation(
          Module.Vector3.add(
            handPos,
            handRot.transformVector(adjustedRelTrans)
          ),
          handRecord.heldObjId,
          0
        );
        this.sim.setRotation(
          Module.Quaternion.mul(handRot, handRecord.heldRelRot),
          handRecord.heldObjId,
          0
        );
      }

      // handle release
      if (handRecord.heldObjId != -1 && !buttonStates[0]) {
        // set held object to dynamic
        this.sim.setObjectMotionType(
          Module.MotionType.DYNAMIC,
          handRecord.heldObjId,
          0
        );
        handRecord.heldObjId = -1;
      }

      if (buttonStates[1] && !handRecord.prevButtonStates[1]) {
        // cylinderSolid_rings_1_segments_12_halfLen_1_useTexCoords_false_useTangents_false_capEnds_true

        const offsetDist = 0.25;
        let spawnPos = Module.Vector3.add(
          handPos,
          new Module.Vector3(
            palmFacingDir.x() * offsetDist,
            palmFacingDir.y() * offsetDist,
            palmFacingDir.z() * offsetDist
          )
        );
        let filepath = "cubeSolid";
        if (this.objectCounter < objectSpawnOrder.length) {
          let nextIndex = this.objectCounter;
          this.objectCounter++;
          filepath = DataUtils.getObjectConfigFilepath(
            objectSpawnOrder[nextIndex]
          );
        }
        let objId = this.sim.addObjectByHandle(filepath, null, "", 0);
        if (objId != -1) {
          this.sim.setTranslation(spawnPos, objId, 0);
        }
      }

      handRecord.prevButtonStates = buttonStates;
    }
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
    updateHeadPose(pose, agent);

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
    const posAsArray = pointToArray(pose.transform.position).slice(0, -1);
    const orientationAsArray = pointToArray(pose.transform.orientation);
    this.tryLogHeadPose(posAsArray, orientationAsArray);
  }

  tryLogHeadPose(position, orientation) {
    if (!this.headPosesInputElement) {
      return;
    }

    const currDate = new Date();
    const currMs = currDate.getTime();

    const logHeadPosePeriodMs = 1000; // log every 1000 ms
    if (
      this.recentLogHeadPoseTimeMs &&
      currMs - this.recentLogHeadPoseTimeMs < logHeadPosePeriodMs
    ) {
      return;
    }

    if (!this.recentLogHeadPoseTimeMs) {
      this.recentLogHeadPoseTimeMs = currMs;
    } else {
      this.recentLogHeadPoseTimeMs += logHeadPosePeriodMs;
    }

    this.logHeadPose(position, orientation);
  }

  logHeadPose(positionFloatArray, orientationFloatArray) {
    const orientationPrecision = 7;
    const positionPrecision = 3;
    let s = "{{";
    for (let i = 0; i < positionFloatArray.length; i++) {
      const elem = positionFloatArray[i];
      s += elem.toFixed(positionPrecision);
      if (i < positionFloatArray.length - 1) {
        s += ",";
      }
    }
    s += "},{";
    for (let i = 0; i < orientationFloatArray.length; i++) {
      const elem = orientationFloatArray[i];
      s += elem.toFixed(orientationPrecision);
      if (i < orientationFloatArray.length - 1) {
        s += ",";
      }
    }
    s += "}},";
    this.headPosesInputElement.value += s;
  }

  updateFPS() {
    if (!this.fpsElement) {
      return;
    }

    if (!this.lastPaintTime) {
      this.lastPaintTime = performance.now();
    } else {
      const current = performance.now();
      const secondsElapsed = (current - this.lastPaintTime) / 1000;
      this.fps =
        0.7 * this.fps + (0.3 * this.currentFramesSkipped) / secondsElapsed;
      this.lastPaintTime = current;
      this.fpsElement.innerHTML = `FPS: ${this.fps.toFixed(2)}`;
    }

    this.currentFramesSkipped = 0;
  }
}
