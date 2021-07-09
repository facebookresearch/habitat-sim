// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

const objectNames = [
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

function randomObject() {
  return objectNames[Math.floor(Math.random() * objectNames.length)];
}

let numIterations = 10;
let objectsPerIteration = 50;

/* Gets a list of objects to spawn/delete for physics benchmarking */
export function getBenchmarkTasks() {
  let res = [];
  for (let i = 0; i < numIterations; i++) {
    for (let j = 0; j < objectsPerIteration; j++) {
      // spawn a random object
      let objectName = randomObject();
      res.push(["spawn", objectName]);
    }
    res.push(["delete"]);
  }
  return res;
}

export function getResults(benchmarkLog) {
  console.assert(benchmarkLog[0][0] == "start");
  let startTime = benchmarkLog[0][1];

  let numFrames = 0;
  let numSteps = 0;

  let avgFPS = 0;
  let avgSPS = 0;
  for (const entry of benchmarkLog) {
    if (entry[0] == "start") {
      continue;
    } else if (entry[0] == "renderFrame") {
      numFrames++;
    } else if (entry[0] == "stepWorld") {
      numSteps++;
    } else if (entry[0] == "delete") {
      let totalTime = (entry[1] - startTime) / 1000.0;
      startTime = entry[1];
      let FPS = numFrames / totalTime;
      let SPS = numSteps / totalTime;
      console.log(FPS, SPS);
      numFrames = 0;
      numSteps = 0;
      avgFPS += FPS;
      avgSPS += SPS;
    }
  }
  avgFPS /= numIterations;
  avgSPS /= numIterations;
  return [avgFPS, avgSPS];
}
