// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

import {
  getServerAndURL,
  getBrowserAndPage,
  closeBrowserAndServer
} from "./test_utils.js";

// It's impractical to directly test Habitat JS bindings here because of
// the challenge of importing Module (emscripten-built webassembly). As
// a workaround, we load a test page in Puppeteer and log both console
// messages and page errors. See also test_page.js.
test("JS test page should load successfully", async () => {
  jest.setTimeout(20000);
  // avoid port conflict with viewer_demo.visual.test.js
  const { server, url } = await getServerAndURL(
    "build_js/esp/bindings_js/test_page.html",
    4005
  );
  const { browser, page } = await getBrowserAndPage(null);

  page.on("console", msg => {
    for (let i = 0; i < msg.args().length; ++i) {
      console.log(`console event ${i}: ${msg.args()[i]}`);
    }
  });

  page.on("pageerror", err => {
    console.log("pageerror event: " + err);
  });

  page.setDefaultTimeout(15000);

  let didTestPageLoad = true;

  await page.goto(url, { waitUntil: "load" }).catch(() => {
    didTestPageLoad = false;
  });

  await page.waitForFunction("window.didTestPageLoad").catch(() => {
    didTestPageLoad = false;
  });

  closeBrowserAndServer(browser, server);

  expect(didTestPageLoad).toBeTruthy();
});
