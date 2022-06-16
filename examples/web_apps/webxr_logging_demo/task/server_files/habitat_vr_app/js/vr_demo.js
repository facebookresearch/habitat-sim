// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

const VIEW_SENSORS = ["left_eye", "right_eye"];
const pointToArray = p => [p.x, p.y, p.z, p.w];

class HandRecord {
  objIds = [];
  prevButtonStates = [false, false];
  heldObjId = -1;
}

class VRDemo {
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

  viewYawOffset = 0;
  FWD = Module.Vector3.zAxis(1);
  FWD_ANGLE = Math.atan2(this.FWD.z(), this.FWD.x());
  DOWN = Module.Vector3.yAxis(-1);

  fps = 0;
  skipFrames = 60;
  currentFramesSkipped = 0;

  constructor() {
    this.fpsElement = document.getElementById("fps");
  }

  static preloadFiles(preloadFunc) {
    let dataUrlBase = "data/webxr_hand_demo_dataset";

    preloadFunc(dataUrlBase + "/stages/remake_v0_JustBigStuff_00.glb");
    preloadFunc(
      dataUrlBase + "/stages/remake_v0_JustBigStuff_00.stage_config.json"
    );

    preloadFunc(dataUrlBase + "/objects/hand_r_open.glb");
    preloadFunc(dataUrlBase + "/objects/hand_r_open.object_config.json");
    preloadFunc(dataUrlBase + "/objects/hand_r_closed.glb");
    preloadFunc(dataUrlBase + "/objects/hand_r_closed.object_config.json");

    preloadFunc(dataUrlBase + "/objects/hand_l_open.glb");
    preloadFunc(dataUrlBase + "/objects/hand_l_open.object_config.json");
    preloadFunc(dataUrlBase + "/objects/hand_l_closed.glb");
    preloadFunc(dataUrlBase + "/objects/hand_l_closed.object_config.json");
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
    this.config.scene_id =
      "data/webxr_hand_demo_dataset/stages/remake_v0_JustBigStuff_00.glb";
    this.config.enablePhysics = false;
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
    this.config.allowPbrShader = false; // Pbr shader isn't robust on WebGL yet

    this.sim = new Module.Simulator(this.config);

    const agentConfigOrig = new Module.AgentConfiguration();

    // Quest 2 is getting viewport of 1440 x 1584 for each eye (why? official specs say 1832 x 1920)
    // Let's do half-resolution (~55 fps vs ~20 fps for full-resolution)
    const width = 720;
    const height = 792;
    const hfov = 89;

    const specs = new Module.VectorSensorSpec();
    {
      const spec = new Module.CameraSensorSpec();
      spec.uuid = "left_eye";
      spec.sensorType = Module.SensorType.COLOR;
      spec.sensorSubType = Module.SensorSubType.PINHOLE;
      spec.resolution = [height, width];
      spec.hfov = hfov;
      specs.push_back(spec);
    }
    {
      let spec = new Module.CameraSensorSpec();
      spec.uuid = "right_eye";
      spec.sensorType = Module.SensorType.COLOR;
      spec.sensorSubType = Module.SensorSubType.PINHOLE;
      spec.resolution = [height, width];
      spec.hfov = hfov;
      specs.push_back(spec);
    }
    agentConfigOrig.sensorSpecifications = specs;

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
    state.position = [-1.43794, 0, -1.01545];
    state.rotation = [0.0, 0.0, 0.0, 1.0];
    agent.setState(state, false);

    Module.loadAllObjectConfigsFromPath(
      this.sim,
      "/data/webxr_hand_demo_dataset/objects"
    );

    this.handRecords = [new HandRecord(), new HandRecord()];

    const handFilepathsByHandIndex = [
      [
        "/data/webxr_hand_demo_dataset/objects/hand_l_open.object_config.json",
        "/data/webxr_hand_demo_dataset/objects/hand_l_closed.object_config.json"
      ],
      [
        "/data/webxr_hand_demo_dataset/objects/hand_r_open.object_config.json",
        "/data/webxr_hand_demo_dataset/objects/hand_r_closed.object_config.json"
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
  }

  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
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
      let handIndex = inputSource.handedness == "left" ? 0 : 1;
      let handRecord = this.handRecords[handIndex];

      const agent = this.sim.getAgent(this.agentId);
      let state = new Module.AgentState();
      agent.getState(state);
      let agentPos = new Module.Vector3(...state.position);

      // 6DoF pose example
      if (inputSource.gripSpace) {
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

        const inputPose = frame.getPose(
          inputSource.gripSpace,
          this.xrReferenceSpace
        );

        if (!inputPose) {
          // hack hide hand by translating far away
          this.sim.setTranslation(
            new Module.Vector3(-1000.0, -1000.0, -1000.0),
            hiddenHandObjId,
            0
          );
        } else {
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
        }

        // hack hide other hand by translating far away
        this.sim.setTranslation(
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

      const sensor = agent.getSubtreeSensors().get(VIEW_SENSORS[iView]);

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

    const agent = this.sim.getAgent(this.agentId);

    this.handleInput(frame);
    this.updateHeadPose(pose, agent);

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
      this.fpsElement.innerHTML += `${this.fps.toFixed(2)}<br>`;
    }
  }
}
