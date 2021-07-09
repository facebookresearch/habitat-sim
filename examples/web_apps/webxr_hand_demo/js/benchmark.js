let framecnt = 0.0;
let delta = 100.0;

function logFrame() {
  framecnt += 1.0;
}

function spawnObject(objectName) {
  postMessage(["spawn", objectName]);
}

function deleteAllObjects() {
  postMessage(["delete"]);
}

function stepPhysics(dt) {
  postMessage(["step", dt]);
}

onmessage = function(s) {
  if (s.data == "frame") {
    logFrame();
  } else if (s.data == "run") {
    console.log("RUNNING BENCHMARK");
    runBenchmark();
  }
};

let curFPS = 0;

setInterval(function() {
  let thisFPS = framecnt / (delta / 1000.0);
  curFPS = 0.9 * curFPS + 0.1 * thisFPS;
  console.log("curFPS", curFPS.toFixed(2));
  framecnt = 0;
}, delta);

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

let n = 10;
let k = 30000;
function runBenchmark() {
  for (let i = 0; i < n; i++) {
    for (let t = 0; t < k; t++) {
      if (t % 2000 == 0) {
        // spawn a random object
        let objectName = randomObject();
        spawnObject(objectName);
      }
      stepPhysics(0.001);
    }
    deleteAllObjects();
  }
}
