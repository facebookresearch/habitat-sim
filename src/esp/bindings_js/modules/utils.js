// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/**
 *
 * @param {function} func Function to be throttled
 * @param {int} timeout Milliseconds for which throttle will be called
 * `func` will be called atleast one in timeout timeframe.
 */
export function throttle(func, timeout = 500) {
  let active = false;

  return function() {
    const args = arguments;
    const context = this;
    if (!active) {
      func.apply(context, args);
      active = true;
      window.setTimeout(() => {
        active = false;
      }, timeout);
    }
  };
}
