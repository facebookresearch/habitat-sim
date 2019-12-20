import {
  getURL,
  getBrowserAndPage,
} from "./test_utils.js";
import { toMatchImageSnapshot } from "jest-image-snapshot";

expect.extend({ toMatchImageSnapshot });

test("viewer rendering should match the snapshot", async () => {
  jest.setTimeout(120000);
  const url = getURL(
    "build_js/esp/bindings_js/viewer.html?scene=skokloster-castle.glb&useDefaultEpisode=true"
    );
  const { browser, page } = await getBrowserAndPage(url);

  page.setDefaultTimeout(120000);

  await page.waitForFunction(
    'document.querySelector("#status").style.color === "white"'
  );
  const screenshot = await page.screenshot();
  browser.close();
  expect(screenshot).toMatchImageSnapshot();
});
