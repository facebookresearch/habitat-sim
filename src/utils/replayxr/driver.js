
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// TODO: These are hardcoded until we integrate a module to provide keyframes (from disk or network).
//       Must match definition in ReplayXr.cpp.
let jsonReplayFilePath = "data/short_replay_lights.json"
//let jsonReplayFilePath = "data/floorplanner.gfx_replay.json"
let filenameSet = new Set()

function preload(url) {
    const splits = url.split("/");
    var file = splits[splits.length - 1];
    if (url.indexOf("http") === -1) {
        var fileParents = splits.slice(0, splits.length - 1);
        var parentStack = []
        for (var i = 0; i < splits.length - 1; i += 1) {
            if (splits[i] == ".") {
            }
            else if (splits[i] == "..") {
                if (parentStack.length > 0) {
                    parentStack.pop();
                }
            }
            else {
                parentStack.push(splits[i]);
            }
        }

        var parentsStr = "/";
        for (var i = 0; i < parentStack.length; ++i) {
            parentsStr += parentStack[i];
            if (!FS.analyzePath(parentsStr).exists) {
                FS.mkdir(parentsStr);
            }
            parentsStr += "/";
        }
    }

    // TODO: The same file could be added at the same time simultaneously with relative paths (e.g. ./ and ../.)
    // TODO: Properly manage async pre-loading such as the app wait for resources to be processed before rendering.
    var fullAssetPath = parentsStr + file;
    if (!filenameSet.has(fullAssetPath)){
        filenameSet.add(fullAssetPath);
        if (!FS.analyzePath(fullAssetPath).exists) {
            FS.createPreloadedFile(parentsStr, file, url, true, false);
        } else {
            console.log("Unable to create virtual file for: " + url );
            console.log("> " + parentsStr + " ~ " + file);
        }
    }
    return fullAssetPath;
}

function isAssetValid(path) {
    return path !== "cubeSolid";
}

async function loadJsonFromFile(jsonFile) {
    const response = await fetch(jsonFile);
    const json = await response.json();
    return json;
}

// Quest Browser doesn't have a console. This prints messages directly in the page.
Module.printErr = function(_message) {
    console.error(Array.prototype.slice.call(arguments).join(' '));
    logElement = document.getElementById('log');
    logElement.innerHTML = _message + "<br/>" + logElement.innerHTML;
};

// Quest Browser doesn't have a console. This prints messages directly in the page.
Module.print = function(_message) {
    console.log(Array.prototype.slice.call(arguments).join(' '));
    logElement = document.getElementById('log');
    logElement.innerHTML = _message + "<br/>" + logElement.innerHTML;
};

Module.preRun.push(async () => {
    // TODO: For now, we pre-load before initialization.
    //       We may want to lazy-load upon receiving 'loads' in keyframes to avoid pre-loading entire datasets.
    loadJsonFromFile(jsonReplayFilePath).then(json => {
        var keyframe = json.keyframes[0];

        // Preload files in keyframe 'loads'
        if (keyframe.hasOwnProperty("loads")) {
            var loads = keyframe.loads;
            for (var i = 0; i < loads.length; ++i) {
                var filepath = loads[i].filepath;
                if (isAssetValid(filepath)) {
                    preload(filepath);
                }
            }
        }
        preload(jsonReplayFilePath);
    });
});
