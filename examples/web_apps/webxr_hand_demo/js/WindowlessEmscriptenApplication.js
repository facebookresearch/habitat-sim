/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021 Vladimír Vondruš <mosra@centrum.cz>

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

/* global define */

function createMagnumModule(init) {
  const offscreen = new OffscreenCanvas(256, 256);

  const module = Object.assign({}, Module);

  Object.assign(module, {
    preRun: [],
    postRun: [],

    arguments: [],

    printErr: function() {},

    print: function() {},

    /* onAbort not handled here, as the output is printed directly on the page */

    canvas: offscreen,
    status: document.getElementById("status"),
    statusDescription: document.getElementById("status-description"),

    setStatus: function(message) {
      if (module.status) {
        module.status.innerHTML = message;
      }
    },

    setStatusDescription: function(message) {
      if (module.statusDescription) {
        module.statusDescription.innerHTML = message;
      }
    },

    totalDependencies: 0,

    monitorRunDependencies: function(left) {
      this.totalDependencies = Math.max(this.totalDependencies, left);

      if (left) {
        module.setStatus("Downloading...");
        module.setStatusDescription(
          this.totalDependencies - left + " / " + this.totalDependencies
        );
      } else {
        module.setStatus("Download complete");
        module.setStatusDescription("");
      }
    }
  });

  Object.assign(module, init);

  module.setStatus("Downloading...");

  return module;
}

/* Default global Module object */
var Module = createMagnumModule();

/* UMD export */
if (typeof exports === "object" && typeof module === "object") {
  /* CommonJS/Node */
  module.exports = createMagnumModule;
} else if (typeof define === "function" && define["amd"]) {
  /* AMD */
  define([], function() {
    return createMagnumModule;
  });
} else if (typeof exports === "object") {
  /* CommonJS strict */
  exports["createMagnumModule"] = createMagnumModule;
}
