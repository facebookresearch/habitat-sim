# Overview

This is a webapp that allows the user to look around a virtual scene on an AR device, like a mobile phone. They can pick up objects and release them by touching the screen.

# Known Issues

This demo may not work on iPhone because AR on the browser, at the time of this writing, has not been supported yet. An Android phone should work.

# Installation

1. `cd` to the main `habitat_sim` directory. Create a folder for the data, then download the webxr demo data:
```bash
$ mkdir examples/web_apps/webxr_mobile_demo/data
$ python -m habitat_sim.utils.datasets_download --uids webxr_demo --data-path examples/web_apps/webxr_mobile_demo/data
```
2. Follow [instructions](https://github.com/facebookresearch/habitat-sim#experimental-emscripten-webgl-and-web-apps) for installing and activating Emscripten, including `source path/to/emsdk_env.sh` or similar to configure env variables.
1. `cd` to `examples/web_apps/webxr_mobile_demo` (this directory). Then run a script to transpile Habitat into JS, copy the resulting files over, and also copy over the JS utils files.
```bash
$ chmod +x build_and_install_habitat_sim_js.sh
$ ./build_and_install_habitat_sim_js.sh
```
To update the Habitat-sim library for the mobile demo, you just need to rerun the second command.

# Testing
## Emulation in your desktop browser, standalone

1. `cd` to `examples/web_apps/webxr_mobile_demo` and http-serve the app folder:
```bash
$ python3 -m http.server
```
2. Install the [WebXR emulator](https://blog.mozvr.com/webxr-emulator-extension/) and then restart your browser.
1. Navigate to `http://0.0.0.0:8000`
    - You can also add a URL parameter to choose the stage to spawn in. For instance, appending `?stage=remake_v0_JustBigStuff_00` at the end of the URL means that stage will be loaded. By default, if this parameter is not given, `remake_v0_JustBigStuff_00` will be the scene that is loaded. See the Project folder structure section for instructions on how to add stages.
1. Recommended: watch the dev console as the page loads.
1. Once loading is complete, click "Enter VR". You should see a stereo 3D view of a kitchen scene.
1. In Chrome Developer Tools, find the WebXR tab. From the device list, select Samsung Galaxy S8+ (AR) or similar. You can now move around the virtual headset, as well as the virtual phone, to simulate the experience.
1. At the bottom of the WebXR tab, click "Exit immersive".

## Testing on mobile phone (or any device that supports AR)
1. Serve the content at a public URL. This has to be an HTTPS address because that is a requirement for WebXR (just serving it locally won't cut it either). You could upload this `webxr_mobile_demo` folder to Amazon S3 or any web host. You might experience some issues uploading the files in `webxr_mobile_demo/data` because they are symlinked. If that happens, upload the actual source files, then adjust the file structure to match `webxr_mobile_demo/data`. [todo: step through how to host on AWS]
1. On your device, open a browser that supports AR (Chrome is a safe bet) and navigate to the URL. Click "Enter VR".

# Troubleshooting

If you encounter Habitat-sim build errors, make sure you have Emscripten properly installed and activated. Delete the build output folders (`habitat-sim/build_corrade-rc` and `habitat-sim/build_js`) and rebuild.

If the app fails to load in your desktop browser, look for errors in the browser dev console. Make sure you have data files installed properly, e.g. `habitat-sim/examples/web_apps/webxr_mobile_demo/data/webxr_demo_dataset/stages/remake_v0_JustBigStuff_00.glb`. Make sure the WebXR emulator is installed and accessible (e.g. as a tab in Chrome Developer Tools).

# Project folder structure
- `data/webxr_demo_dataset` contains all the models that are used in the default setup, plus a few extra ReplicaCAD objects
    - `default.physics_config.json` contains the physics settings, e.g. gravitational acceleration.
    - `stages` contain the scene `.glb` files and their corresponding `.stage_config.json` files. In order to choose a scene other than `remake_v0_JustBigStuff_00`, you need to add its 2 files here.
    - `objects` contains a green sphere and a red sphere that are used as markers, as well as the ReplicaCAD objects that can be spawned. It also contains some extra objects you may use.
- `js` contains the JS source code of the mobile demo site.
- `lib` contains the JS transpiled version of Habitat-sim, as well as some general use utils JS files. This folder is what gets modified when you run `build_and_install_habitat_sim_js.sh`.
- `index.html` is the hosted site, and `style.css` is its stylesheet.

# Habitat-sim JS build

The Habitat-sim JS build is a webassembly build of the Habitat simulator. It's built by compiling Habitat-sim C++ code to webassembly using the Emscripten compiler. The build outputs are `hsim_bindings.js` and `hsim_bindings.wasm`. `hsim_bindings.js` can be included from Javascript and you can call into the simulator via the bindings defined in `habitat-sim/src/esp/bindings_js/bindings_js.cpp`.

We also utilize some JS utilities provided as .js files inside Habitat-sim. These are also copied from the Habitat-sim source folders to `examples/web_apps/webxr_mobile_demo/lib` as part of the build script.
