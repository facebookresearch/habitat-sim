// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module, XRWebGLLayer */

import {
  getEyeSensorSpecs,
  updateHeadPose,
  VIEW_SENSORS
} from "../lib/habitat-sim-js/vr_utils.js";

const pointToArray = p => [p.x, p.y, p.z, p.w];

const dataDir = "data/";
const physicsConfigFilepath = dataDir + "default.physics_config.json";
const objectBaseFilepath = dataDir + "objects/";
const stageBaseFilepath = dataDir + "stages/";

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
  skipFrames = 60;
  currentFramesSkipped = 0;

  constructor() {
    this.fpsElement = document.getElementById("fps");
  }

  static getObjectFilepath(name) {
    return objectBaseFilepath + name + ".glb";
  }

  static getObjectConfigFilepath(name) {
    return objectBaseFilepath + name + ".object_config.json";
  }

  static getObjectCollisionGlbFilepath(name) {
    return objectBaseFilepath + name + "_cv_decomp.glb";
  }

  static getStageFilepath(name) {
    return stageBaseFilepath + name + ".glb";
  }

  static getStageConfigFilepath(name) {
    return stageBaseFilepath + name + ".stage_config.json";
  }

  static preloadFiles(preloadFunc) {
    preloadFunc(physicsConfigFilepath);

    preloadFunc(VRDemo.getStageFilepath(Module.stageName));
    preloadFunc(VRDemo.getStageConfigFilepath(Module.stageName));

    preloadFunc(VRDemo.getObjectFilepath("hand_r_open"));
    preloadFunc(VRDemo.getObjectConfigFilepath("hand_r_open"));
    preloadFunc(VRDemo.getObjectFilepath("hand_r_closed"));
    preloadFunc(VRDemo.getObjectConfigFilepath("hand_r_closed"));

    preloadFunc(VRDemo.getObjectFilepath("hand_l_open"));
    preloadFunc(VRDemo.getObjectConfigFilepath("hand_l_open"));
    preloadFunc(VRDemo.getObjectFilepath("hand_l_closed"));
    preloadFunc(VRDemo.getObjectConfigFilepath("hand_l_closed"));

    const replicaCadObjectNames = new Set();
    for (const object of objectSpawnOrder) {
      replicaCadObjectNames.add(object);
    }

    for (const name of replicaCadObjectNames) {
      preloadFunc(VRDemo.getObjectFilepath(name));
      preloadFunc(VRDemo.getObjectCollisionGlbFilepath(name));
      preloadFunc(VRDemo.getObjectConfigFilepath(name));
    }
  }

  display() {
    this.initSimAndSensors();
    this.initScene();
    this.setUpVR();

    this.headPosesInputElement = document.getElementById("head_poses_input");
  }

  setUpVR() {
    const elem = document.getElementById("enter-vr");
    elem.style.visibility = "visible";
    elem.addEventListener("click", this.enterVR.bind(this));
  }

  initSimAndSensors() {
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = VRDemo.getStageFilepath(Module.stageName);
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = physicsConfigFilepath;
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
    this.config.allowPbrShader = false; // Pbr shader isn't robust on WebGL yet

    this.sim = new Module.Simulator(this.config);

    const agentConfigOrig = new Module.AgentConfiguration();
    agentConfigOrig.sensorSpecifications = getEyeSensorSpecs();

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

    Module.loadAllObjectConfigsFromPath(this.sim, objectBaseFilepath);

    this.handRecords = [new HandRecord(), new HandRecord()];

    const handFilepathsByHandIndex = [
      [
        VRDemo.getObjectConfigFilepath("hand_l_open"),
        VRDemo.getObjectConfigFilepath("hand_l_closed")
      ],
      [
        VRDemo.getObjectConfigFilepath("hand_r_open"),
        VRDemo.getObjectConfigFilepath("hand_r_closed")
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

  // Compiles a GLSL shader. Returns null on failure
  compileShader(src, type) {
    const shader = this.gl.createShader(type);
    this.gl.shaderSource(shader, src);
    this.gl.compileShader(shader);

    if (this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS)) {
      return shader;
    } else {
      console.error("GLSL shader error: " + this.gl.getShaderInfoLog(shader));
      return null;
    }
  }

  initGL() {
    if (this.gl === null) {
      return null;
    }
    const vertSrc =
      "attribute vec3 p;attribute vec2 u;varying highp vec2 v;void main(){gl_Position=vec4(p,1);v=u;}";
    const fragSrc =
      "uniform sampler2D t;varying highp vec2 v;void main(){gl_FragColor=texture2D(t,v);}";
    const vert = this.compileShader(vertSrc, this.gl.VERTEX_SHADER);
    const frag = this.compileShader(fragSrc, this.gl.FRAGMENT_SHADER);
    if (vert === null || frag === null) {
      console.log("Failed to compile shaders");
      return null;
    }

    const program = this.gl.createProgram();

    this.gl.attachShader(program, vert);
    this.gl.attachShader(program, frag);

    this.gl.linkProgram(program);

    if (!this.gl.getProgramParameter(program, this.gl.LINK_STATUS)) {
      console.error("GLSL program error:" + this.gl.getProgramInfoLog(program));
      return null;
    }

    this.gl.useProgram(program);

    // Vertices
    const verts = [
      -1.0,
      1.0,
      0.0,
      -1.0,
      -1.0,
      0.0,
      1.0,
      -1.0,
      0.0,
      1.0,
      1.0,
      0.0
    ];
    const vertBuf = this.gl.createBuffer();
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, vertBuf);
    this.gl.bufferData(
      this.gl.ARRAY_BUFFER,
      new Float32Array(verts),
      this.gl.STATIC_DRAW
    );
    // Position attribute
    const posAttrib = this.gl.getAttribLocation(program, "p");
    this.gl.vertexAttribPointer(posAttrib, 3, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(posAttrib);

    // UVs
    const UVs = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0];
    const uvBuf = this.gl.createBuffer();
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, uvBuf);
    this.gl.bufferData(
      this.gl.ARRAY_BUFFER,
      new Float32Array(UVs),
      this.gl.STATIC_DRAW
    );
    // UV attribute
    const uvAttrib = this.gl.getAttribLocation(program, "u");
    this.gl.vertexAttribPointer(uvAttrib, 2, this.gl.FLOAT, false, 0, 0);
    this.gl.enableVertexAttribArray(uvAttrib);

    // Indices
    this.inds = [3, 2, 1, 3, 1, 0];
    const indBuf = this.gl.createBuffer();
    this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, indBuf);
    this.gl.bufferData(
      this.gl.ELEMENT_ARRAY_BUFFER,
      new Uint16Array(this.inds),
      this.gl.STATIC_DRAW
    );

    // Texture uniform
    const texUni = this.gl.getUniformLocation(program, "t");
    this.gl.uniform1i(texUni, 0);

    console.log("Initialized WebXR GL state");
  }

  async enterVR() {
    if (this.gl === null) {
      this.gl = document.createElement("canvas").getContext("webgl", {
        xrCompatible: true
      });
      this.initGL();
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

    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(1.0 / 60);
    }, 1000.0 / 60);

    this.fpsElement.style.visibility = "visible";
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

  drawTextureData(texRes, texData) {
    if (this.tex === null) {
      this.tex = this.gl.createTexture();
      this.gl.bindTexture(this.gl.TEXTURE_2D, this.tex);

      this.gl.texParameteri(
        this.gl.TEXTURE_2D,
        this.gl.TEXTURE_WRAP_S,
        this.gl.CLAMP_TO_EDGE
      );
      this.gl.texParameteri(
        this.gl.TEXTURE_2D,
        this.gl.TEXTURE_WRAP_T,
        this.gl.CLAMP_TO_EDGE
      );
      this.gl.texParameteri(
        this.gl.TEXTURE_2D,
        this.gl.TEXTURE_MIN_FILTER,
        this.gl.NEAREST
      );
      this.gl.texParameteri(
        this.gl.TEXTURE_2D,
        this.gl.TEXTURE_MAG_FILTER,
        this.gl.NEAREST
      );
    }

    this.gl.bindTexture(this.gl.TEXTURE_2D, this.tex);
    this.gl.texImage2D(
      this.gl.TEXTURE_2D,
      0, // level
      this.gl.RGBA, // internal format
      texRes[1], // width
      texRes[0], // height
      0, // border
      this.gl.RGBA, // format
      this.gl.UNSIGNED_BYTE, // type
      new Uint8Array(texData) // data
    );

    this.gl.activeTexture(this.gl.TEXTURE0);

    this.gl.drawElements(
      this.gl.TRIANGLES,
      this.inds.length,
      this.gl.UNSIGNED_SHORT,
      0
    );
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
          filepath = VRDemo.getObjectConfigFilepath(
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
      this.drawTextureData(texRes, texData);
    }

    this.updateFPS();

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

    if (this.currentFramesSkipped != this.skipFrames) {
      this.currentFramesSkipped++;
      return;
    }

    this.currentFramesSkipped = 0;

    if (!this.lastPaintTime) {
      this.lastPaintTime = performance.now();
    } else {
      const current = performance.now();
      const secondsElapsed = (current - this.lastPaintTime) / 1000;
      this.fps = this.skipFrames / secondsElapsed;
      this.lastPaintTime = current;
      this.fpsElement.innerHTML = `FPS: ${this.fps.toFixed(2)}`;
    }
  }
}
