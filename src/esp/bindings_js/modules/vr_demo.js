// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global XRWebGLLayer, Module */
import WebDemo from "./web_demo";
import { defaultAgentConfig } from "./defaults";

const BUTTON_ID = "vr_button";

const VIEW_SENSORS = ["left_eye", "right_eye"];
const pointToArray = p => [p.x, p.y, p.z, p.w];

const replicaCadObjectBaseFilepath = "data/replica/objects/";
const replicaCadObjectNames = [
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

  // "frl_apartment_bowl_01", // orange/white
  //"frl_apartment_bowl_02", // red
  //"frl_apartment_bowl_03", // silver
  // "frl_apartment_bowl_07", // large white with base
  // "frl_apartment_cup_01", // latte mug
  // "frl_apartment_cup_02", // coffee mug
  // "frl_apartment_cup_03", // wider latte mug
  // "frl_apartment_cup_05", // blacl latte mug
  // "frl_apartment_kitchen_utensil_01", // cooking pot with lid
  // "frl_apartment_kitchen_utensil_04", // three-tier drawer
  // "frl_apartment_kitchen_utensil_08", // cylindrical storage container
  // "frl_apartment_kitchen_utensil_09", // gray thermos
  // "frl_apartment_vase_01", // white
  // "frl_apartment_choppingboard_02",
];

const replicaCadObjectInstanceNames = [
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

///////////
// INPUT //
///////////
//const REPEAT_TIMEOUT = 500;
//const LEFT_RIGHT_AXIS = 2;
//const FWD_AXIS = 3;
//const VIEW_YAW_STEP = Math.PI * 0.1;

class HandRecord {
  objIds = [];
  prevButtonStates = [false, false];
  heldObjId = -1;
}

export class FpsHelper {
  // eslint-disable-next-line
  constructor(fpsComponent) {
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
      // fpsComponent.innerHTML =
      console.log(
        fps_desc +
          " " +
          (this.numRendersSinceLastUpdate / duration).toFixed(1).toString()
      );
      this.numRendersSinceLastUpdate = 0;
    }, 2000.0);
  }

  onFrame() {
    this.numRendersSinceLastUpdate += 1;
  }
}

class VRDemo extends WebDemo {
  fpsElement;
  lastPaintTime;

  webXRSession = null;
  xrReferenceSpace = null;
  gl = null;
  tex = null;
  inds = null;

  sensorSpecs = null;

  prevFwdHeld = false;
  prevLeftHeld = false;
  prevRightHeld = false;

  viewYawOffset = 0;
  FWD = Module.Vector3.zAxis(1);
  FWD_ANGLE = Math.atan2(this.FWD.z(), this.FWD.x());
  DOWN = Module.Vector3.yAxis(-1);

  fps = 0;
  skipFrames = 60;
  currentFramesSkipped = 0;

  constructor(canvasId = "canvas", fpsId = "fps") {
    super();
    this.canvasElement = document.getElementById(canvasId);
    this.fpsElement = document.getElementById(fpsId);
    this.fpsHelper = new FpsHelper(this.fpsElement);
  }

  static getReplicaCadObjectConfigFilepath(name) {
    let configFilepath =
      replicaCadObjectBaseFilepath +
      "configs_convex/" +
      name +
      ".object_config.json";
    return configFilepath;
  }

  static preloadFiles(preloadFunc) {
    // For dataUrlBase, if you specify a relative path here like "data", it is relative
    // to the bindings_js folder where index.js is served. You can alternately specify a
    // full URL base such as https://www.mywebsite.com/data". For local testing, note
    // that we symlink your habitat-sim data directory (habitat-sim/data) to
    // /build_js/esp/bindings_js/data. (See "resources" in
    // src/esp/bindings_js/CMakeLists.txt.)
    let dataUrlBase = "data";
    preloadFunc(dataUrlBase + "/objects/hand_r_open.glb", true);
    preloadFunc(dataUrlBase + "/objects/hand_r_open.object_config.json", true);
    preloadFunc(dataUrlBase + "/objects/hand_r_closed.glb", true);
    preloadFunc(
      dataUrlBase + "/objects/hand_r_closed.object_config.json",
      true
    );

    preloadFunc(dataUrlBase + "/objects/hand_l_open.glb", true);
    preloadFunc(dataUrlBase + "/objects/hand_l_open.object_config.json", true);
    preloadFunc(dataUrlBase + "/objects/hand_l_closed.glb", true);
    preloadFunc(
      dataUrlBase + "/objects/hand_l_closed.object_config.json",
      true
    );

    for (const name of replicaCadObjectNames) {
      let renderGlbFilepath = replicaCadObjectBaseFilepath + name + ".glb";
      let collisionGlbFilepath =
        replicaCadObjectBaseFilepath +
        "configs_convex/" +
        name +
        "_cv_decomp.glb";
      let configFilepath = VRDemo.getReplicaCadObjectConfigFilepath(name);
      preloadFunc(renderGlbFilepath);
      preloadFunc(collisionGlbFilepath);
      preloadFunc(configFilepath);
    }
  }

  display(agentConfig = defaultAgentConfig, episodePath = undefined) {
    super.display(agentConfig, episodePath);
    this.setUpVR();
  }

  setUpVR() {
    const button = document.createElement("button");
    button.id = BUTTON_ID;
    this.canvasElement.after(button);
    this.exitVR();
  }

  initSensors() {
    const agent = this.simenv.sim.getAgent(this.simenv.selectedAgentId);

    this.sensorSpecs = new Array(VIEW_SENSORS.length);
    for (var iView = 0; iView < VIEW_SENSORS.length; ++iView) {
      const sensor = agent.sensorSuite.get(VIEW_SENSORS[iView]);

      this.sensorSpecs[iView] = sensor.specification();
    }
  }

  initScene() {
    // Set agent to identity transform.
    const agent = this.simenv.sim.getAgent(this.simenv.selectedAgentId);
    let state = new Module.AgentState();
    agent.getState(state);
    // ReplicaCAD
    state.position = [2.0, 0.1, 5.8];
    // ConvoAI
    // state.position = [-1.43794, 0, -1.01545];
    state.rotation = [0.0, 0.0, 0.0, 1.0];
    agent.setState(state, false);

    Module.loadAllObjectConfigsFromPath(this.simenv.sim, "/data/objects");
    Module.loadAllObjectConfigsFromPath(
      this.simenv.sim,
      replicaCadObjectBaseFilepath + "configs_convex"
    );

    this.handRecords = [new HandRecord(), new HandRecord()];

    const handFilepathsByHandIndex = [
      [
        "/data/objects/hand_l_open.object_config.json",
        "/data/objects/hand_l_closed.object_config.json"
      ],
      [
        "/data/objects/hand_r_open.object_config.json",
        "/data/objects/hand_r_closed.object_config.json"
      ]
    ];

    for (const handIndex of [0, 1]) {
      for (const filepath of handFilepathsByHandIndex[handIndex]) {
        let objId = this.simenv.sim.addObjectByHandle(filepath, null, "", 0);
        this.simenv.sim.setObjectMotionType(
          Module.MotionType.KINEMATIC,
          objId,
          0
        );
        this.simenv.sim.setTranslation(
          new Module.Vector3(0.0, 0.0, 0.0),
          objId,
          0
        );
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
      this.initSensors();
      this.initScene();
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
      this.simenv.sim.stepWorld(1.0 / 60);
    }, 1000.0 / 60);
  }

  // remove the event listener
  resetButton() {
    const button = document.getElementById(BUTTON_ID);
    const newButton = button.cloneNode(true);
    button.parentNode.replaceChild(newButton, button);
    return newButton;
  }

  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
    }
    const button = this.resetButton();
    button.innerHTML = "Enter VR";
    button.addEventListener("click", this.enterVR.bind(this));
  }

  renderDisplay() {
    if (this.webXRSession !== null) {
      this.webXRSession.requestAnimationFrame(this.drawVRScene.bind(this));
      const button = this.resetButton();
      button.innerHTML = "Exit VR";
      button.addEventListener("click", this.exitVR.bind(this));
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
      let handIndex = inputSource.handedness == "left" ? 0 : 1;
      let otherHandIndex = handIndex == 0 ? 1 : 0;
      let handRecord = this.handRecords[handIndex];
      let otherHandRecord = this.handRecords[otherHandIndex];

      // if (inputSource.gamepad) {
      //   const demo = this;

      //   const fwdHeld = inputSource.gamepad.axes[FWD_AXIS] < -0.5;
      //   if (fwdHeld && !this.prevFwdHeld) {
      //     this.task.handleAction("moveForward");
      //     window.setTimeout(function() {
      //       demo.prevFwdHeld = false;
      //     }, REPEAT_TIMEOUT);
      //   }
      //   this.prevFwdHeld = fwdHeld;

      //   const leftHeld = inputSource.gamepad.axes[LEFT_RIGHT_AXIS] > 0.5;
      //   if (leftHeld && !this.prevleftHeld) {
      //     this.viewYawOffset += VIEW_YAW_STEP;
      //     window.setTimeout(function() {
      //       demo.prevLeftHeld = false;
      //     }, REPEAT_TIMEOUT);
      //   }
      //   this.prevLeftHeld = leftHeld;

      //   const rightHeld = inputSource.gamepad.axes[LEFT_RIGHT_AXIS] < -0.5;
      //   if (rightHeld && !this.prevRightHeld) {
      //     this.viewYawOffset -= VIEW_YAW_STEP;
      //     window.setTimeout(function() {
      //       demo.prevRightHeld = false;
      //     }, REPEAT_TIMEOUT);
      //   }
      //   this.prevRightHeld = rightHeld;
      // }

      const agent = this.simenv.sim.getAgent(this.simenv.selectedAgentId);
      let state = new Module.AgentState();
      agent.getState(state);
      let agentPos = new Module.Vector3(...state.position);

      // 6DoF pose example
      if (inputSource.gripSpace) {
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
        this.simenv.sim.setTranslation(handPos, handObjId, 0);
        this.simenv.sim.setRotation(handRot, handObjId, 0);

        let palmFacingSign = handIndex == 0 ? 1.0 : -1.0;
        let palmFacingDir = handRot.transformVector(
          new Module.Vector3(palmFacingSign, 0.0, 0.0)
        );

        // try grab
        if (buttonStates[0] && !handRecord.prevButtonStates[0]) {
          let maxDistance = 0.15;

          let hitObjId = Module.castRay(
            this.simenv.sim,
            handPos,
            palmFacingDir,
            maxDistance
          );

          if (hitObjId != -1) {
            handRecord.heldObjId = hitObjId;

            if (otherHandRecord.heldObjId == hitObjId) {
              // release from other hand
              otherHandRecord.heldObjId = -1;
            }

            let currTrans = this.simenv.sim.getTranslation(
              handRecord.heldObjId,
              0
            );
            let currRot = this.simenv.sim.getRotation(handRecord.heldObjId, 0);

            let handRotInverted = handRot.inverted();
            handRecord.heldRelRot = Module.Quaternion.mul(
              handRotInverted,
              currRot
            );
            handRecord.heldRelTrans = handRotInverted.transformVector(
              Module.Vector3.sub(currTrans, handPos)
            );

            // set held obj to kinematic
            this.simenv.sim.setObjectMotionType(
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

          this.simenv.sim.setTranslation(
            Module.Vector3.add(
              handPos,
              handRot.transformVector(adjustedRelTrans)
            ),
            handRecord.heldObjId,
            0
          );
          this.simenv.sim.setRotation(
            Module.Quaternion.mul(handRot, handRecord.heldRelRot),
            handRecord.heldObjId,
            0
          );
        }

        // handle release
        if (handRecord.heldObjId != -1 && !buttonStates[0]) {
          // set held object to dynamic
          this.simenv.sim.setObjectMotionType(
            Module.MotionType.DYNAMIC,
            handRecord.heldObjId,
            0
          );
          handRecord.heldObjId = -1;
        }

        if (buttonStates[1] && !handRecord.prevButtonStates[1]) {
          // cylinderSolid_rings_1_segments_12_halfLen_1_useTexCoords_false_useTangents_false_capEnds_true
          //let filepath = "cubeSolid";

          if (this.objectCounter < replicaCadObjectInstanceNames.length) {
            let nextIndex = this.objectCounter;
            this.objectCounter++;

            const offsetDist = 0.25;
            let spawnPos = Module.Vector3.add(
              handPos,
              new Module.Vector3(
                palmFacingDir.x() * offsetDist,
                palmFacingDir.y() * offsetDist,
                palmFacingDir.z() * offsetDist
              )
            );

            let filepath = VRDemo.getReplicaCadObjectConfigFilepath(
              replicaCadObjectInstanceNames[nextIndex]
            );
            let objId = this.simenv.sim.addObjectByHandle(
              filepath,
              null,
              "",
              0
            );
            if (objId != -1) {
              this.simenv.sim.setTranslation(spawnPos, objId, 0);
            }
          }
        }

        // hack hide other hand by translating far away
        this.simenv.sim.setTranslation(
          new Module.Vector3(-1000.0, -1000.0, -1000.0),
          hiddenHandObjId,
          0
        );

        handRecord.prevButtonStates = buttonStates;
      }
    }
  }

  updateHeadPose(pose, agent) {
    const headRotation = Module.toQuaternion(
      pointToArray(pose.transform.orientation)
    );
    const pointingVec = headRotation.transformVector(this.FWD);
    const pointingAngle =
      Math.atan2(pointingVec.z(), pointingVec.x()) - this.FWD_ANGLE;

    const agentQuat = Module.Quaternion.rotation(
      new Module.Rad(pointingAngle + this.viewYawOffset),
      this.DOWN
    );
    const inverseAgentRot = Module.Quaternion.rotation(
      new Module.Rad(-pointingAngle),
      this.DOWN
    );

    let state = new Module.AgentState();
    agent.getState(state);
    state.rotation = Module.toVec4f(agentQuat);
    agent.setState(state, false);

    for (var iView = 0; iView < pose.views.length; ++iView) {
      const view = pose.views[iView];

      const sensor = agent.sensorSuite.get(VIEW_SENSORS[iView]);

      const pos = pointToArray(view.transform.position).slice(0, -1); // don't need w for position
      sensor.setLocalTransform(
        Module.toVec3f(
          inverseAgentRot.transformVector(new Module.Vector3(...pos))
        ),
        Module.toVec4f(
          Module.Quaternion.mul(
            inverseAgentRot,
            Module.toQuaternion(pointToArray(view.transform.orientation))
          )
        )
      );
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

    const agent = this.simenv.sim.getAgent(this.simenv.selectedAgentId);

    this.handleInput(frame);
    this.updateHeadPose(pose, agent);

    const layer = session.renderState.baseLayer;
    this.gl.bindFramebuffer(this.gl.FRAMEBUFFER, layer.framebuffer);

    for (var iView = 0; iView < pose.views.length; ++iView) {
      const view = pose.views[iView];
      const viewport = layer.getViewport(view);
      this.gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);

      const sensor = agent.sensorSuite.get(VIEW_SENSORS[iView]);
      const texRes = this.sensorSpecs[iView].resolution;
      const texData = sensor.getObservation(this.simenv.sim).getData();
      this.drawTextureData(texRes, texData);
    }

    this.fpsHelper.onFrame();
  }
}

export default VRDemo;
