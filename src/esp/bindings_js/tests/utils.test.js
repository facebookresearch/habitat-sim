import { throttle } from "../modules/utils";

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
