// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

import {
  getServerAndURL,
  getBrowserAndPage,
  closeBrowserAndServer
} from "./test_utils.js";
import { toMatchImageSnapshot } from "jest-image-snapshot";

expect.extend({ toMatchImageSnapshot });

test("viewer rendering should match the snapshot", async () => {
  console.log("test 1");
  jest.setTimeout(30000);
  console.log("test 2");

  const { server, url } = await getServerAndURL(
    "build_js/esp/bindings_js/viewer.html?scene=skokloster-castle.glb&useDefaultEpisode=true"
  );
  console.log("test 3");

  const { browser, page } = await getBrowserAndPage(url);
  console.log("test 4");

  page.setDefaultTimeout(30000);

  page.on("console", msg => {
    for (let i = 0; i < msg.args().length; ++i) {
      console.log(`console event ${i}: ${msg.args()[i]}`);
    }
  });

  await page.waitForFunction(
    'document.querySelector("#status").style.color === "white"'
  );
  console.log("test 5");
  const screenshot = await page.screenshot();

  console.log("test 6");

  closeBrowserAndServer(browser, server);
  console.log("test 7");

  expect(screenshot).toMatchImageSnapshot({
    // Different GPUs and different driver version will produce slightly
    // different images; differences on aliased edges may also stem from how
    // a particular importer parses transforms
    failureThreshold: 0.15,
    failureThresholdType: "percent"
  });
  console.log("test 8");
});
