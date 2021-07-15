// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

export class Benchmark {
  // PUBLIC VARIABLES
  numObjects = 0; // number of types of objects that can be spawned
  spawnFn = null; // callback function to spawn objects
  deleteFn = null; // callback function to delete objects

  spawnPos = new Module.Vector3(0, 0, 0); // where to spawn objects
  spawnPosJitter = 0.0; // x and z coordinates can be shifted by +/- this
  spawnVel = new Module.Vector3(0, -10, 0); // initial velocity

  numIterations = 10; // number of times to repeat the test
  objectsPerIteration = 50; // number of objects to spawn on each test

  stepsBetweenSpawn = 2; // number of stepWorld() calls between spawning
  stepsBeforeDelete = 20; // number of stepWorld() calls before deleting

  // PRIVATE VARIABLES
  #iterationIdx = 0;
  #objectIdx = 0;
  #stepIdx = 0;

  #currentlySpawned = [];

  #log = [];

  #active = false;

  /*
   * objectNames: The number of possible objects that can be spawned.
   * spawnFn: A function of type (objectIdx, pos, vel) -> objId.
   *   When called by stepBenchmark(), this function should spawn the object
   *   corresponding to objectIdx at position pos with velocity vel. It should
   *   then return the id of the spawned object.
   * deleteFn: A function of type objIds list -> (). When called by
   *   stepBenchmark(), this function should go through the object ids in the
   *   list and delete all of those objects.
   */
  constructor(numObjects, spawnFn, deleteFn) {
    this.spawnFn = spawnFn;
    this.deleteFn = deleteFn;
    this.numObjects = numObjects;
  }

  /* Starts the timer */
  start() {
    this.active = true;
    this.#log.push(["start", performance.now()]);
  }

  /* Returns true if the benchmark hasn't finished running. */
  active() {
    return this.active;
  }

  /*
   * Call this every time stepWorld() is called in the main program. This is
   * the code that spawns and deletes objects to execute the benchmark.
   */
  stepBenchmark() {
    console.assert(this.active());
    this.#log.push(["stepWorld", performance.now()]);

    this.#stepIdx++;
    if (this.#objectIdx == this.objectsPerIteration) {
      if (this.#stepIdx == this.stepsBeforeDelete) {
        this.#stepIdx = 0;
        this.#objectIdx = 0;
        this.#iterationIdx++;
        if (this.#iterationIdx == this.numIterations) {
          this.active = false;
        }

        this.deleteFn(this.#currentlySpawned);
        this.#currentlySpawned = [];
        this.#log.push(["delete", performance.now()]);
      }
    } else {
      if (this.#stepIdx == this.stepsBetweenSpawn) {
        this.#stepIdx = 0;
        this.#objectIdx++;

        const obj = Math.floor(Math.random() * this.numObjects);
        const jitterX =
          Math.random() * 2 * this.spawnPosJitter - this.spawnPosJitter;
        const jitterZ =
          Math.random() * 2 * this.spawnPosJitter - this.spawnPosJitter;
        const pos = new Module.Vector3(
          this.spawnPos.x() + jitterX,
          this.spawnPos.y(),
          this.spawnPos.z() + jitterZ
        );
        this.#currentlySpawned.push(this.spawnFn(obj, pos, this.spawnVel));
      }
    }
  }

  /* Call this every time a frame is rendered in the main program. */
  logFrame() {
    console.assert(this.active());
    this.#log.push(["renderFrame", performance.now()]);
  }

  /*
   * When the benchmark execution is finished, call this to compute the
   * results. Returns the average time between frames as well as the average
   * time between calls to stepWorld().
   */
  getResults() {
    console.assert(!this.active());
    console.assert(this.#log[0][0] == "start");

    let startTime = this.#log[0][1];

    let numFrames = 0;
    let numSteps = 0;

    let avgFrameTime = 0;
    let avgStepTime = 0;
    for (const entry of this.#log) {
      if (entry[0] == "start") {
        continue;
      } else if (entry[0] == "renderFrame") {
        numFrames++;
      } else if (entry[0] == "stepWorld") {
        numSteps++;
      } else if (entry[0] == "delete") {
        const totalTime = (entry[1] - startTime) / 1000.0;
        startTime = entry[1];
        const frameTime = totalTime / numFrames;
        const stepTime = totalTime / numSteps;
        numFrames = 0;
        numSteps = 0;
        avgFrameTime += frameTime;
        avgStepTime += stepTime;
      }
    }
    avgFrameTime /= this.numIterations;
    avgStepTime /= this.numIterations;
    return [avgFrameTime, avgStepTime];
  }
}
