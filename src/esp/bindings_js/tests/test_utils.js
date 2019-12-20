import puppeteer from "puppeteer";

export async function getBrowserAndPage(url) {
  const browser = await puppeteer.launch({
    args: ["--disable-lcd-text"],
    defaultViewport: { width: 1920, height: 1080 }
  });

  const page = await browser.newPage();

  await page.goto(url, { waitUntil: "load" });

  return { browser, page };
}

export function getURL(path) {
  return `http://localhost:4004/${path}`;
}
