import { infoSemanticFileName } from "./defaults";

// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/**
 *
 * @param {function} func Function to be throttled
 * @param {int} timeout Milliseconds interval after which throttle will be called,
 * `func` will be called at least once in the timeout timeframe.
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

/**
 * Check whether web assembly is supported on the current browser or not.
 * Returns false if not otherwise true.
 */
export function checkWebAssemblySupport() {
  try {
    if (
      typeof WebAssembly === "object" &&
      typeof WebAssembly.instantiate === "function"
    ) {
      const module = new WebAssembly.Module(
        Uint8Array.of(0x0, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00)
      );
      if (module instanceof WebAssembly.Module) {
        return new WebAssembly.Instance(module) instanceof WebAssembly.Instance;
      }
    }
  } catch (e) {
    return false;
  }
  return false;
}

/**
 * Checks for WebGL2 support in current browser.
 * Returns 0 if there is no support, 1 if support is there but disabled otherwise 2.
 */
export function checkWebgl2Support() {
  let canvas;
  let ctx;
  let hasWebgl = false;

  try {
    canvas = document.createElement("canvas");
    ctx = canvas.getContext("webgl") || canvas.getContext("experimental-webgl");
  } catch (e) {
    return;
  }

  if (ctx !== undefined) {
    hasWebgl = true;
  }

  canvas = undefined;
  if (!hasWebgl) {
    if (typeof WebGL2RenderingContext !== "undefined") {
      return 1;
    } else {
      return 0;
    }
  } else {
    return 2;
  }
}

export function getInfoSemanticUrl(mainUrl) {
  const splits = mainUrl.split("/");
  const moreThanOne = splits.length > 1;
  splits.pop();
  let infoSemanticPath = infoSemanticFileName;
  if (moreThanOne) {
    infoSemanticPath = "/" + infoSemanticPath;
  }
  return splits.join("/") + infoSemanticPath;
}

export function buildConfigFromURLParameters(config = {}) {
  for (let arg of window.location.search.substr(1).split("&")) {
    let [key, value] = arg.split("=");
    if (key && value) {
      config[key] = decodeURIComponent(value);
    }
  }
  return config;
}
