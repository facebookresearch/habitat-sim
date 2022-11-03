// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
  jest.setTimeout(12000);
  const { server, url } = await getServerAndURL(
    "build_js/esp/bindings_js/viewer.html?scene=skokloster-castle.glb&useDefaultEpisode=true"
  );
  const { browser, page } = await getBrowserAndPage(url);

  page.setDefaultTimeout(12000);

  await page.waitForFunction(
    'document.querySelector("#status").style.color === "white"'
  );
  const screenshot = await page.screenshot();

  closeBrowserAndServer(browser, server);
  expect(screenshot).toMatchImageSnapshot({
    // Different GPUs and different driver version will produce slightly
    // different images; differences on aliased edges may also stem from how
    // a particular importer parses transforms
    failureThreshold: 0.15,
    failureThresholdType: "percent"
  });
});
