// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module, XRWebGLLayer */

import { initGL, drawTextureData } from "../lib/habitat-sim-js/xr_utils.js";
import {
  VIEW_SENSOR,
  getEyeSensorSpecs
} from "../lib/habitat-sim-js/ar_utils.js";
import { DataUtils } from "./data_utils.js";
import { SimHelpers } from "./sim_helpers.js";

// Each of these objects will be spawned at a random location chosen from
// possibleSpawnLocations.
const initialObjects = [
  "frl_apartment_plate_01", // plate
  "frl_apartment_plate_01",
  "frl_apartment_kitchen_utensil_06", // cup
  "frl_apartment_kitchen_utensil_06",
  "frl_apartment_bowl_06", // bowl
  "frl_apartment_bowl_06", // bowl
  "frl_apartment_kitchen_utensil_02", // green spice shaker
  "frl_apartment_kitchen_utensil_03" // orange spice shaker
];

// Possible locations on the counter to spawn objects at.
const possibleSpawnLocations = [
  [0.5, 1.0, 5.5],
  [0.8, 1.0, 5.75],
  [0.5, 1.0, 6.0],
  [0.8, 1.0, 6.25],
  [0.5, 1.0, 6.5],
  [0.8, 1.0, 6.75],
  [0.5, 1.0, 7.0],
  [0.8, 1.0, 7.25],
  [0.5, 1.0, 7.5]
];

export class VRDemo {
  webXRSession = null;
  xrReferenceSpace = null;
  gl = null;
  tex = null;
  headPos = null;
  lookDir = null;
  headRot = null;
  agentPos = null;
  heldObjectId = -1;
  objectIndex = 0;

  constructor() {}

  // Sets up the virtual file system so that these files can be loaded with
  // their respective file paths. This is called by habitat_main.js and is the
  // first function in this file to run.
  preloadFiles(preloadFunc) {
    preloadFunc(DataUtils.getPhysicsConfigFilepath());

    preloadFunc(DataUtils.getStageFilepath(Module.stageName));
    preloadFunc(DataUtils.getStageConfigFilepath(Module.stageName));

    // the "cursor" that indicates where the user is pointing
    preloadFunc(DataUtils.getObjectFilepath("greensphere"));
    preloadFunc(DataUtils.getObjectConfigFilepath("greensphere"));
    // the "dropMarker" that indicates where the held object would fall if it
    // were dropped
    preloadFunc(DataUtils.getObjectFilepath("redsphere"));
    preloadFunc(DataUtils.getObjectConfigFilepath("redsphere"));

    const replicaCadObjectNames = new Set();
    for (const object of initialObjects) {
      replicaCadObjectNames.add(object);
    }

    for (const name of replicaCadObjectNames) {
      preloadFunc(DataUtils.getObjectFilepath(name));
      preloadFunc(DataUtils.getObjectCollisionGlbFilepath(name));
      preloadFunc(DataUtils.getObjectConfigFilepath(name));
    }
  }

  // Entry point for all the hand demo logic.
  start() {
    // init sim
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = DataUtils.getStageFilepath(Module.stageName);
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = DataUtils.getPhysicsConfigFilepath();
    this.config.sceneLightSetup = ""; // this empty string means "use lighting"
    this.config.overrideSceneLightDefaults = true; // always set this to true
    this.sim = new Module.Simulator(this.config);

    // this class contains useful helper functions we will use
    this.simHelpers = new SimHelpers(this.sim);

    // init agent
    const agentConfigOrig = new Module.AgentConfiguration();
    agentConfigOrig.sensorSpecifications = getEyeSensorSpecs(1000, 450);
    this.sim.addAgent(agentConfigOrig);
    this.agentId = 0;

    // place agent
    const agent = this.sim.getAgent(this.agentId);
    let state = new Module.AgentState();
    agent.getState(state);
    state.position = [2.0, 1.2, 5.8]; // todo: specify start position/orientation via URL param (or react?)
    this.agentPos = new Module.Vector3(...state.position);
    state.rotation = [0.0, 0.0, 0.0, 1.0];
    agent.setState(state, false);

    // load objects from data
    Module.loadAllObjectConfigsFromPath(
      this.sim,
      DataUtils.getObjectBaseFilepath()
    );

    // used to draw red line from grabbed object down to the floor
    this.debugLineRender = this.sim.getDebugLineRender();

    // randomly place the objects on the counter
    this.spawnInitialObjects();

    // How far away held objects are.
    this.heldObjectDistance = 0.3;

    // Positions in the real world are multiplied by these values.
    this.moveSensitivity = new Module.Vector3(3.0, 1.5, 3.0);

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
    this.webXRSession = await navigator.xr.requestSession("immersive-ar");

    this.webXRSession.addEventListener("end", () => {
      this.webXRSession = null;
      this.exitVR();
    });

    const xrLayer = new XRWebGLLayer(this.webXRSession, this.gl);
    this.webXRSession.updateRenderState({ baseLayer: xrLayer });
    this.xrReferenceSpace = await this.webXRSession.requestReferenceSpace(
      "local"
    );

    // When the user touches the screen, we call onTouch. When they let go, we
    // call onRelease.
    this.webXRSession.addEventListener("selectstart", this.onTouch.bind(this));
    this.webXRSession.addEventListener("selectend", this.onRelease.bind(this));

    // Every 1/60s, step physics by 1/60s.
    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(1.0 / 60);
    }, 1000.0 / 60);

    // Initiate the cursor.
    this.cursor = this.sim.addObjectByHandle(
      DataUtils.getObjectConfigFilepath("greensphere"),
      null,
      "",
      0
    );
    this.sim.setObjectMotionType(Module.MotionType.KINEMATIC, this.cursor, 0);
    this.sim.setObjectIsCollidable(false, this.cursor);

    // Initiate the drop marker.
    this.dropMarker = this.sim.addObjectByHandle(
      DataUtils.getObjectConfigFilepath("redsphere"),
      null,
      "",
      0
    );
    this.sim.setObjectMotionType(
      Module.MotionType.KINEMATIC,
      this.dropMarker,
      0
    );
    this.sim.setObjectIsCollidable(false, this.dropMarker);
    this.simHelpers.hide(this.dropMarker);

    // Tell WebXR to call this.drawVRScene() to draw the next frame.
    this.webXRSession.requestAnimationFrame(this.drawVRScene.bind(this));
  }

  // Code that runs when the user exits Immersive Mode.
  exitVR() {
    if (this.webXRSession !== null) {
      this.webXRSession.end();
    }
  }

  // Randomly spawn the objects in initialObjects at locations in
  // possibleSpawnLocations.
  spawnInitialObjects() {
    // shuffle spawn locations
    for (let i = possibleSpawnLocations.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      const tmp = possibleSpawnLocations[j];
      possibleSpawnLocations[j] = possibleSpawnLocations[i];
      possibleSpawnLocations[i] = tmp;
    }
    // spawn objects at corresponding locations
    for (let i = 0; i < initialObjects.length; i++) {
      let handle = initialObjects[i];
      let objId = this.sim.addObjectByHandle(
        DataUtils.getObjectConfigFilepath(handle),
        null,
        "",
        0
      );
      this.sim.setTranslation(
        new Module.Vector3(...possibleSpawnLocations[i]),
        objId,
        0
      );
    }
  }

  // Casts a ray from head to middle of screen.
  raycastFromHead() {
    let grabRay = new Module.Ray(this.headPos, this.lookDir);
    return this.simHelpers.exclusiveRaycast(grabRay, 1000.0, [this.cursor]);
  }

  // When the user touches the screen, we see if they are pointing at an object
  // and if so, we pick it up.
  onTouch() {
    let rayHitInfo = this.raycastFromHead();
    const hitObjId = rayHitInfo == null ? -1 : rayHitInfo.objectId;
    if (hitObjId != -1) {
      this.heldObjectId = hitObjId;
      if (this.headRot != null) {
        let currRot = this.sim.getRotation(this.heldObjectId, 0);
        this.heldRelRot = Module.Quaternion.mul(
          this.headRot.inverted(),
          currRot
        );
      }
      this.sim.setObjectMotionType(
        Module.MotionType.KINEMATIC,
        this.heldObjectId,
        0
      );
    }
  }

  // When the user picks up their finger, we drop the held object if it exists.
  onRelease() {
    if (this.heldObjectId == -1) {
      return;
    }
    this.sim.setObjectMotionType(
      Module.MotionType.DYNAMIC,
      this.heldObjectId,
      0
    );
    this.simHelpers.hide(this.dropMarker);
    this.heldObjectId = -1;
  }

  // Given the pose (position/orientation of the phone), update the agent's eye
  // sensor location.
  updatePose(pose, agent) {
    const pointToArray = p => [p.x, p.y, p.z, p.w];
    const FWD = Module.Vector3.zAxis(1);
    const FWD_ANGLE = Math.atan2(FWD.z(), FWD.x());
    const DOWN = Module.Vector3.yAxis(-1);
    const viewYawOffset = 0;

    const headRotation = Module.toQuaternion(
      pointToArray(pose.transform.orientation)
    );
    const pointingVec = headRotation.transformVector(FWD);
    const pointingAngle =
      Math.atan2(pointingVec.z(), pointingVec.x()) - FWD_ANGLE;

    const agentQuat = Module.Quaternion.rotation(
      new Module.Rad(pointingAngle + viewYawOffset),
      DOWN
    );
    const inverseAgentRot = Module.Quaternion.rotation(
      new Module.Rad(-pointingAngle),
      DOWN
    );

    let state = new Module.AgentState();
    agent.getState(state);
    state.rotation = Module.toVec4f(agentQuat);
    agent.setState(state, false);

    const view = pose.views[0];

    const sensor = agent.getSubtreeSensors().get(VIEW_SENSOR);

    let pos = pointToArray(view.transform.position).slice(0, -1); // don't need w for position
    // adjust sensitivity
    pos[0] *= this.moveSensitivity.x();
    pos[1] *= this.moveSensitivity.y();
    pos[2] *= this.moveSensitivity.z();
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

    this.lookDir = new Module.Vector3(
      -pointingVec.x(),
      -pointingVec.y(),
      -pointingVec.z()
    );
    this.headPos = Module.Vector3.add(
      this.agentPos,
      new Module.Vector3(...pos)
    );
    this.headRot = headRotation;
  }

  // Adjust the held object to stay in front of the cursor with the correct
  // rotation.
  updateHeldObjectPose() {
    if (this.heldObjectId == -1) {
      return;
    }
    if (this.headPos != null) {
      this.sim.setTranslation(
        Module.Vector3.add(
          this.headPos,
          Module.Vector3.mul(this.lookDir, this.heldObjectDistance)
        ),
        this.heldObjectId,
        0
      );
    }
    if (this.headRot != null) {
      this.sim.setRotation(
        Module.Quaternion.mul(this.headRot, this.heldRelRot),
        this.heldObjectId,
        0
      );
    }
  }

  // Draw the green sphere that indicates where the user is looking.
  drawCursor() {
    let sceneHit = this.raycastFromHead();
    if (sceneHit != null) {
      let hitLoc = sceneHit.point;
      this.sim.setTranslation(hitLoc, this.cursor, 0);
    }
  }

  // Draw the red sphere that is vertically below the held object.
  drawDropMarker() {
    if (this.heldObjectId == -1) {
      return;
    }
    let objPos = this.sim.getTranslation(this.heldObjectId, 0);
    let downDir = new Module.Vector3(0, -1, 0);
    let rayDown = new Module.Ray(objPos, downDir);
    let hit = this.simHelpers.exclusiveRaycast(rayDown, 1000.0, [
      this.cursor,
      this.heldObjectId
    ]);
    // find where this ray hits, excluding the cursor and the held object
    let hitPos = hit == null ? -1 : hit.point;
    this.sim.setTranslation(hitPos, this.dropMarker, 0);
    let col = new Module.Color4(1.0, 0.0, 0.0, 1.0);
    this.debugLineRender.drawLine(objPos, hitPos, col);
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

    this.updatePose(pose, agent);
    this.updateHeldObjectPose();
    this.drawCursor();
    this.drawDropMarker();

    // Draw stuff to the canvas.
    const layer = session.renderState.baseLayer;
    this.gl.bindFramebuffer(this.gl.FRAMEBUFFER, layer.framebuffer);

    const view = pose.views[0];
    const viewport = layer.getViewport(view);
    this.gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);

    const sensor = agent.getSubtreeSensors().get(VIEW_SENSOR);
    const texRes = sensor.specification().resolution;
    const texData = sensor.getObservation(this.sim).getData();
    drawTextureData(this.gl, texRes, texData);
  }
}
