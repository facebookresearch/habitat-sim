// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

import {
  throttle,
  getInfoSemanticUrl,
  buildConfigFromURLParameters
} from "../modules/utils";
import { infoSemanticFileName } from "../modules/defaults";

test("throttle should work properly", () => {
  let count = 0;
  let interval;
  const startTime = Date.now();
  function incrementCounter(resolve) {
    if (count === 5) {
      resolve(count);
    } else {
      count += 1;
    }
  }

  return new Promise(resolve => {
    interval = window.setInterval(
      throttle(() => incrementCounter(resolve), 500),
      50
    );
  }).then(count => {
    window.clearInterval(interval);
    expect(count).toEqual(5);
    expect(Date.now() - startTime).toBeGreaterThan(2500);
    return count;
  });
});

test("info semantic.json should have correct path", () => {
  const scenePaths = [
    "https://some_path.com/x/mesh.ply",
    "./some_path/mesh.ply",
    "mesh.ply",
    "/mesh.ply"
  ];
  const expectedInfoPaths = [
    "https://some_path.com/x/info_semantic.json",
    "./some_path/info_semantic.json",
    "info_semantic.json",
    "/info_semantic.json"
  ];

  scenePaths.forEach((item, index) => {
    expect(getInfoSemanticUrl(item, infoSemanticFileName)).toEqual(
      expectedInfoPaths[index]
    );
  });
});

test("configuration should be built from url parameters", () => {
  delete window.location;
  window.location = {};
  window.location.search = "?a=b&c&d=true&e=1";
  const config = buildConfigFromURLParameters();
  expect(config.a).toEqual("b");
  expect(config.c).toBeUndefined();
  expect(config.d).toEqual("true");
  expect(config.e).toEqual("1");
});
