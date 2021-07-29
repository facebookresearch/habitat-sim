// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, importScripts */

function preloadFunc(url) {
  let file_parents_str = "/";
  const splits = url.split("/");
  let file = splits[splits.length - 1];
  if (url.indexOf("http") === -1) {
    let file_parents = splits.slice(0, splits.length - 1);
    for (let i = 0; i < splits.length - 1; i += 1) {
      file_parents_str += file_parents[i];
      if (!FS.analyzePath(file_parents_str).exists) {
        FS.mkdir(file_parents_str, 777);
      }
      file_parents_str += "/";
    }
  }
  FS.createPreloadedFile(file_parents_str, file, url, true, false);
  return file_parents_str + file;
}

function createMagnumModule(init) {
  const module = Object.assign({}, Module);
  Object.assign(module, {
    preRun: [],
    postRun: [],
    arguments: [],
    canvas: null,
    status: null,
    statusDescription: null,
    log: null,
    printErr: function() {},
    print: function() {},
    setStatus: function() {},
    setStatusDescription: function() {},
    totalDependencies: 0,
    monitorRunDependencies: function(left) {
      this.totalDependencies = Math.max(this.totalDependencies, left);
      if (left) {
        module.setStatus("Downloading...");
        module.setStatusDescription(
          this.totalDependencies - left + " / " + this.totalDependencies
        );
      } else {
        module.setStatus("Download complete");
        module.setStatusDescription("");
      }
    }
  });
  Object.assign(module, init);
  module.setStatus("Downloading...");
  if (module.log) {
    module.log.style.display = "none";
  }
  return module;
}

let stageFilepath, physicsConfigFilepath, objectBaseFilepath;
var Module;

onmessage = function(e) {
  console.assert(e.data.type == "preloadInfo");
  let preloadInfo = e.data.value;

  Module = createMagnumModule();

  Module.preRun.push(() => {
    stageFilepath = preloadInfo.stageFilepath;
    physicsConfigFilepath = preloadInfo.physicsConfigFilepath;
    objectBaseFilepath = preloadInfo.objectBaseFilepath;
    for (const file of preloadInfo.preloadedFiles) {
      preloadFunc(file);
    }
  });

  Module.onRuntimeInitialized = async function() {
    start();
  };

  importScripts("hsim_bindings.js");
};

// -------------------------------------------------------------------------------

function start() {
  postMessage({ type: "ready", value: null });
  onmessage = function(e) {
    // todo: get a bunch of variables here rather than hard code into this file
    if (e.data.type == "start") {
      let startData = e.data.value;
      let physicsWorker = new PhysicsWorker(
        startData.handFilepathsByHandIndex,
        startData.objectSpawnOrder
      );
      physicsWorker.start();
    }
  };
}

class HandRecord {
  objIds = [];
  prevGripState = false;
  prevSpawnState = false;
  heldObjId = -1;
}

class PhysicsWorker {
  objectCounter = 0;

  constructor(handFilepathsByHandIndex, objectSpawnOrder) {
    // initialize stuff
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = stageFilepath;
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = physicsConfigFilepath;
    this.config.createRenderer = false;
    this.config.enableGfxReplaySave = true;
    this.sim = new Module.Simulator(this.config);
    Module.loadAllObjectConfigsFromPath(this.sim, objectBaseFilepath);
    this.recorder = this.sim.getGfxReplayManager().getRecorder();

    this.objectSpawnOrder = objectSpawnOrder;

    this.handRecords = [new HandRecord(), new HandRecord()];

    // add hands
    for (const handIndex of [0, 1]) {
      for (const filepath of handFilepathsByHandIndex[handIndex]) {
        let objId = this.sim.addObjectByHandle(filepath, null, "", 0);
        this.sim.setObjectMotionType(Module.MotionType.KINEMATIC, objId, 0);
        this.sim.setTranslation(new Module.Vector3(0.0, 0.0, 0.0), objId, 0);
        this.handRecords[handIndex].objIds.push(objId);
      }
    }
  }

  start() {
    let spawn = this.spawn.bind(this);
    let deleteAllObjects = this.deleteAllObjects.bind(this);
    let operateHands = this.operateHands.bind(this);
    onmessage = function(e) {
      if (e.data.type == "spawn") {
        let spawnInfo = e.data.value;
        let handle = spawnInfo.handle;
        let pos = new Module.Vector3(...spawnInfo.pos);
        let vel = new Module.Vector3(...spawnInfo.vel);
        spawn(handle, pos, vel);
      } else if (e.data.type == "delete") {
        deleteAllObjects();
      } else if (e.data.type == "hands") {
        operateHands(e.data.value);
      }
    };

    // physics step
    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(0.003);

      this.recorder.saveKeyframe();
      let keyframe = this.recorder.getLatestKeyframe();
      let jsonKeyframe = this.recorder.keyframeToString(keyframe);
      postMessage({ type: "keyframe", value: jsonKeyframe });
    }, 1000.0 / 60);
  }

  curSpawned = [];

  spawn(handle, pos, vel) {
    const objId = this.sim.addObjectByHandle(handle, null, "", 0);
    this.sim.setTranslation(pos, objId, 0);
    this.sim.setLinearVelocity(vel, objId, 0);
    this.curSpawned.push(objId);
  }

  deleteAllObjects() {
    for (const objId of this.curSpawned) {
      this.sim.removeObject(objId, true, true, 0);
    }
    this.curSpawned = [];
  }

  operateHands(handInfo) {
    for (const hand of handInfo) {
      let handRecord = this.handRecords[hand.index];
      let otherHandRecord = this.handRecords[hand.index ^ 1];
      let handObjId = hand.gripButton
        ? handRecord.objIds[1]
        : handRecord.objIds[0];
      let hiddenHandObjId = hand.gripButton
        ? handRecord.objIds[0]
        : handRecord.objIds[1];
      let handPos = new Module.Vector3(...hand.pos);
      let handRot = Module.toQuaternion(hand.rot);

      this.sim.setTranslation(handPos, handObjId, 0);
      this.sim.setRotation(handRot, handObjId, 0);

      // hack hide other hand by translating far away
      this.sim.setTranslation(
        new Module.Vector3(-1000.0, -1000.0, -1000.0),
        hiddenHandObjId,
        0
      );

      let palmFacingSign = hand.index == 0 ? 1.0 : -1.0;
      let palmFacingDir = handRot.transformVector(
        new Module.Vector3(palmFacingSign, 0.0, 0.0)
      );

      // try grab
      if (hand.gripButton && !handRecord.prevGripState) {
        let grabRay = new Module.Ray(handPos, palmFacingDir);
        let maxDistance = 0.15;

        let raycastResults = this.sim.castRay(grabRay, maxDistance, 0);
        let hitObjId = raycastResults.hasHits()
          ? raycastResults.hits.get(0).objectId
          : -1;

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
      if (handRecord.heldObjId != -1 && !hand.gripButton) {
        // set held object to dynamic
        this.sim.setObjectMotionType(
          Module.MotionType.DYNAMIC,
          handRecord.heldObjId,
          0
        );
        handRecord.heldObjId = -1;
      }

      // spawn object
      if (hand.spawnButton && !handRecord.prevSpawnState) {
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
        if (this.objectCounter < this.objectSpawnOrder.length) {
          let nextIndex = this.objectCounter;
          this.objectCounter++;
          filepath = this.objectSpawnOrder[nextIndex];
        }
        let objId = this.sim.addObjectByHandle(filepath, null, "", 0);
        if (objId != -1) {
          this.sim.setTranslation(spawnPos, objId, 0);
        }
      }

      handRecord.prevGripState = hand.gripButton;
      handRecord.prevSpawnState = hand.spawnButton;
    }
  }
}
