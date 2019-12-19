import {
  getServerAndURL,
  getBrowserAndPage,
  closeBrowserAndServer
} from "./server.js";
import { toMatchImageSnapshot } from "jest-image-snapshot";

expect.extend({ toMatchImageSnapshot });

test("viewer rendering should match the snapshot", async () => {
  jest.setTimeout(120000);
  page.setDefaultTimeout(120000);
  const { server, url } = await getServerAndURL(
    "build_js/esp/bindings_js/viewer.html?useDefaultEpisode=true"
  );
  const { browser, page } = await getBrowserAndPage(url);
  await page.waitForFunction(
    'document.querySelector("#status").style.color === "white"'
  );
  const screenshot = await page.screenshot();

  closeBrowserAndServer(browser, server);
  expect(screenshot).toMatchImageSnapshot();
});
