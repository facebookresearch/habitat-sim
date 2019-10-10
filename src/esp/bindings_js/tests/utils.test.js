import { throttle, getInfoSemanticUrl } from "../modules/utils";

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
    expect(getInfoSemanticUrl(item)).toEqual(expectedInfoPaths[index]);
  });
});
