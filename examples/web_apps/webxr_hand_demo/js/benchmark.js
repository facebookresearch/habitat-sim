// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

export class Benchmark {
  objectNames = null;
  spawnFn = null;
  deleteFn = null;

  spawnPos = new Module.Vector3(0, 0, 0);
  spawnPosJitter = 0.0;
  spawnVel = new Module.Vector3(0, -10, 0);

  numIterations = 10;
  objectsPerIteration = 50;

  stepsBetweenSpawn = 2;
  stepsBeforeDelete = 20;

  #iterationIdx = 0;
  #objectIdx = 0;
  #stepIdx = 0;

  #currentlySpawned = [];

  #log = [];

  constructor(objectNames, spawnFn, deleteFn) {
    this.spawnFn = spawnFn;
    this.deleteFn = deleteFn;
    this.objectNames = objectNames;

    this.#log.push(["start", performance.now()]);
  }

  active() {
    return this.#iterationIdx < this.numIterations;
  }

  stepBenchmark() {
    console.assert(this.active());
    this.#log.push(["stepWorld", performance.now()]);

    this.#stepIdx++;
    if (this.#objectIdx == this.objectsPerIteration) {
      if (this.#stepIdx == this.stepsBeforeDelete) {
        this.#stepIdx = 0;
        this.#objectIdx = 0;
        this.#iterationIdx++;

        this.deleteFn(this.#currentlySpawned);
        this.#currentlySpawned = [];
        this.#log.push(["delete", performance.now()]);
      }
    } else {
      if (this.#stepIdx == this.stepsBetweenSpawn) {
        this.#stepIdx = 0;
        this.#objectIdx++;

        const objectName = this.objectNames[
          Math.floor(Math.random() * this.objectNames.length)
        ];
        const jitterX =
          Math.random() * 2 * this.spawnPosJitter - this.spawnPosJitter;
        const jitterZ =
          Math.random() * 2 * this.spawnPosJitter - this.spawnPosJitter;
        const pos = new Module.Vector3(
          this.spawnPos.x() + jitterX,
          this.spawnPos.y(),
          this.spawnPos.z() + jitterZ
        );
        this.#currentlySpawned.push(
          this.spawnFn(objectName, pos, this.spawnVel)
        );
      }
    }
    return true;
  }

  logFrame() {
    console.assert(this.active());
    this.#log.push(["renderFrame", performance.now()]);
  }

  getResults() {
    console.assert(!this.active());
    console.assert(this.#log[0][0] == "start");

    let startTime = this.#log[0][1];

    let numFrames = 0;
    let numSteps = 0;

    let avgFPS = 0;
    let avgSPS = 0;
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
        const FPS = numFrames / totalTime;
        const SPS = numSteps / totalTime;
        console.log(FPS, SPS);
        numFrames = 0;
        numSteps = 0;
        avgFPS += FPS;
        avgSPS += SPS;
      }
    }
    avgFPS /= this.numIterations;
    avgSPS /= this.numIterations;
    return [avgFPS, avgSPS];
  }
}
