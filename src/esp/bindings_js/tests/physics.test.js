// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/*global Module */

import {
  getServerAndURL,
  getBrowserAndPage,
  closeBrowserAndServer
} from "./test_utils.js";

const groundTruthStates = require("./physics_determinism.json");

test("simulator physics determinism", async () => {
  jest.setTimeout(120000);
  const { server, url } = await getServerAndURL(
    "build_js/esp/bindings_js/bindings.html?scene=skokloster-castle.glb&enablePhysics=true&defaultPhysConfig=default.phys_scene_config.json"
  );
  const { browser, page } = await getBrowserAndPage(url);

  page.setDefaultTimeout(120000);

  await page.waitForFunction(
    'document.querySelector("#status").style.color === "white"'
  );

  const executionContext = await page.mainFrame().executionContext();
  const results = await executionContext.evaluate(() => {
    let sim = window.demo.simenv;
    sim.reset();
    let timeline = [];

    let worldTime = function getWorldTime(sim) {
      return sim.getWorldTime();
    };

    let stepPhysics = function stepPhysics(sim) {
      sim.stepWorld();
      return;
    };

    let getAllObjectsState = function getAllObjectsTranslation(sim) {
      let existingObjectIds = sim.getExistingObjectIDs();
      let objectStates = [];
      for (let idx = 0; idx < existingObjectIds.size(); idx++) {
        let objectId = existingObjectIds.get(idx);
        let sceneId = 0;
        let objectTranslation = sim.getTranslation(objectId, sceneId);
        let objectTranslationArray = [
          objectTranslation.x(),
          objectTranslation.y(),
          objectTranslation.z()
        ];

        let motionType = sim.getObjectMotionType(objectId, sceneId);

        objectStates.push({
          objectId: objectId,
          translation: objectTranslationArray,
          motionType: motionType.value
        });
      }
      return objectStates;
    };

    let addObjects = function addSphereAndSoccerBallObjects(sim) {
      let sphereObjectId = sim.addObjectByHandle(
        "/data/objects/sphere.phys_properties.json"
      );
      let spherePosition = sim.convertVec3fToVector3([
        -0.9517786502838135,
        2.167676642537117,
        11.343990325927734
      ]);
      sim.setObjectMotionType(Module.MotionType.DYNAMIC, sphereObjectId, 0);
      sim.setTranslation(spherePosition, sphereObjectId, 0);

      let chairObjectId = sim.addObjectByHandle(
        "/data/objects/chair.phys_properties.json"
      );
      let chairPosition = sim.convertVec3fToVector3([
        -0.9517786502838135,
        1.57676642537117,
        11.343990325927734
      ]);
      sim.setObjectMotionType(Module.MotionType.DYNAMIC, chairObjectId, 0);
      sim.setTranslation(chairPosition, chairObjectId, 0);
      return [sphereObjectId, chairObjectId];
    };

    // initialize scene with objects
    addObjects(sim);

    // initial world time
    timeline.push({
      worldTime: worldTime(sim),
      stepCount: 0,
      objectStates: []
    });

    let testWorldTime = 3.0;
    let count = 1;
    let currentWorldTime = worldTime(sim);
    while (currentWorldTime < testWorldTime) {
      stepPhysics(sim);
      currentWorldTime = worldTime(sim);
      let state = {
        worldTime: currentWorldTime,
        stepCount: count,
        objectStates: getAllObjectsState(sim)
      };
      timeline.push(state);
      count++;
    }

    return Promise.resolve(timeline);
  });

  let testResultLength = results.length;
  let groundTruthFileLength = groundTruthStates.length;

  expect(testResultLength).toEqual(groundTruthFileLength);

  for (let i = 0; i < groundTruthFileLength; i++) {
    expect(results[i]["worldTime"]).toEqual(groundTruthStates[i]["world_time"]);
    expect(results[i]["stepCount"]).toEqual(groundTruthStates[i]["step_count"]);

    expect("objectStates" in results[i]).toEqual(true);

    let testRunObjectsStates = results[i]["objectStates"];
    let groundTruthObjectStates = groundTruthStates[i]["object_states"];
    for (let idx = 0; idx < testRunObjectsStates.length; i++) {
      expect(testRunObjectsStates[i]["objectId"]).toEqual(
        groundTruthObjectStates[i]["object_id"]
      );
      expect(testRunObjectsStates[i]["translation"]).toEqual(
        groundTruthObjectStates[i]["translation"]
      );
    }
  }

  closeBrowserAndServer(browser, server);
});
