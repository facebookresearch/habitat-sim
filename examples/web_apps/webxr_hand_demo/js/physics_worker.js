// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

class HandRecord {
  objIds = [];
  prevGripState = false;
  prevSpawnState = false;
  heldObjId = -1;
}

// eslint-disable-next-line no-unused-vars
class PhysicsWorker {
  objectCounter = 0;

  constructor(
    physicsConfigFilepath, // .physics_config.json file
    stageFilepath, // stage .glb file
    objectBaseFilepath, // folder with all the object_config.json files
    handFilepathsByHandIndex, // .object_config.json files for the hands, in a nested array like so: [[left closed, left open], [right closed, right open]]
    objectSpawnOrder // list of .object_config.json files that to be spawned in order when the user presses the spawn button
  ) {
    // initialize stuff
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = stageFilepath;
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = physicsConfigFilepath;
    this.config.createRenderer = false;
    this.config.enableGfxReplaySave = true;
    this.sim = new Module.Simulator(this.config);
    Module.loadAllObjectConfigsFromPath(this.sim, objectBaseFilepath);

    // This will automatically keep track of the physics we're doing, including
    // object creations, deletions, and movements. It stores this information
    // using an internal list of keyframes, which we will send one by one to
    // the main thread.
    this.recorder = this.sim.getGfxReplayManager().getRecorder();

    this.handRecords = [new HandRecord(), new HandRecord()];
    for (const handIndex of [0, 1]) {
      for (const filepath of handFilepathsByHandIndex[handIndex]) {
        let objId = this.sim.addObjectByHandle(filepath, null, "", 0);
        this.sim.setObjectMotionType(Module.MotionType.KINEMATIC, objId, 0);
        this.sim.setTranslation(new Module.Vector3(0.0, 0.0, 0.0), objId, 0);
        this.handRecords[handIndex].objIds.push(objId);
      }
    }

    this.objectSpawnOrder = objectSpawnOrder;
  }

  // Start stepping physics and sending keyframes to the main thread.
  start() {
    let spawn = this.spawn.bind(this);
    let deleteAllObjects = this.deleteAllObjects.bind(this);
    let updateHandInfo = this.updateHandInfo.bind(this);

    // Redefine onmessage again (it was already defined twice in
    // physics_worker_setup.js) This time, it will be used to handle spawning,
    // deleting, and hand updates. Currently, only hand updates are being
    // used by the webapp. Spawning and deleting are for debugging purposes.
    onmessage = function(e) {
      // "spawn" - means spawn an object with specified handle, position, and
      //   velocity.
      // "delete" - means delete all objects. Deleting an object with
      //   specified objectId is pretty complicated because you have to first
      //   communicate the objectId of a spawned object.
      // "hands" - contains hand input from user. The most recent hand info is
      //   processed at each physics step.
      if (e.data.type == "spawn") {
        let spawnInfo = e.data.value;
        let handle = spawnInfo.handle;
        let pos = new Module.Vector3(...spawnInfo.pos);
        let vel = new Module.Vector3(...spawnInfo.vel);
        spawn(handle, pos, vel);
      } else if (e.data.type == "delete") {
        deleteAllObjects();
      } else if (e.data.type == "hands") {
        updateHandInfo(e.data.value);
      } else {
        console.assert(false); // should be unreachable
      }
    };

    // Step physics every 1/60s.
    this.physicsStepFunction = setInterval(() => {
      if (this.handInfo) {
        this.operateHands();
      }

      this.sim.stepWorld(1.0 / 60);

      // Get the keyframe corresponding to this step from the recorder. Then
      // send it back to the main thread.
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

  updateHandInfo(handInfo) {
    this.handInfo = handInfo;
  }

  operateHands() {
    for (const hand of this.handInfo) {
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
        let nextIndex = this.objectCounter % this.objectSpawnOrder.length;
        this.objectCounter++;
        let filepath = this.objectSpawnOrder[nextIndex];
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
