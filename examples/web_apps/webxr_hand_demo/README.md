# Overview

This is a webapp that allows the user to enter a scene in VR. The user can then spawn various objects and interact with them by picking them up or touching them. Use the index trigger buttons to grab and the secondary trigger buttons to spawn.

# Installation

1. Download the hand demo data:
```bash
python -m habitat_sim.utils.datasets_download --uids webxr_hand_demo_data --data-path examples/web_apps/webxr_hand_demo
```
2. Follow [instructions](https://github.com/facebookresearch/habitat-sim#experimental-emscripten-webgl-and-web-apps) for installing and activating Emscripten, including `source path/to/emsdk_env.sh` or similar to configure env variables.
1. Create the directory to store the JS transpiled version of Habitat:
```bash
mkdir -p examples/web_apps/webxr_hand_demo/lib/habitat-sim-js
```
4. Transpile Habitat into JS and copy the resulting files over. Also copy over the utils files.
```bash
chmod +x build_and_install_habitat_sim_js.sh
./build_and_install_habitat_sim_js.sh
```
To get an updated version of the Habitat library into the hand demo, you just need to rerun the second command.

# Testing
## VR emulation in your desktop browser, standalone

1. Http-serve the VR app folder:
```bash
cd examples/web_apps/webxr_hand_demo && python3 -m http.server
```
2. Install the [WebXR emulator](https://blog.mozvr.com/webxr-emulator-extension/) and then restart your browser.
1. Navigate to `http://0.0.0.0:8000/standalone.html`
    - You can also add a URL parameter to choose the stage to spawn in. For instance, appending `?stage=remake_v0_JustBigStuff_00` at the end of the URL means that stage will be loaded. By default, if this parameter is not given, `remake_v0_JustBigStuff_00` will be the scene that is loaded. See the Project folder structure section for instructions on how to add stages.
1. Recommended: watch the dev console as the page loads.
1. Once loading is complete, click "Enter VR". You should see a stereo 3D view of a kitchen scene.
1. In Chrome Developer Tools, find the WebXR tab. From the device list, select Oculus Quest or similar. In the WebXR panel's 3D view, move the headset or controllers to emulate the VR experience.
1. At the bottom of the WebXR tab, click "Exit immersive".
1. Observe the logged head poses in the html textarea.

## VR on Quest 2 (or other headset)
1. Using a modification of either method above, serve the content at a public URL. For example, you could upload the `task/server_files/habitat_vr_app` folder to Amazon S3 or any web host. [todo: explain how to do this for AWS]
1. On Quest 2, open the browser and navigate to the URL. Use the Quest controller to click "Enter VR".
1. Exit immersive VR with the Quest 2 controller's home button.

# Troubleshooting

If you encounter Habitat-sim build errors, make sure you have Emscripten properly installed and activated. Delete the build output folders (`habitat-sim/build_corrade-rc` and `habitat-sim/build_js`) and rebuild.

If the VR app fails to load in your desktop browser, look for errors in the browser dev console. Make sure you have data files installed properly, e.g. `habitat-sim/examples/web_apps/webxr_hand_demo/data/stages/remake_v0_JustBigStuff_00.glb`. Make sure the WebXR emulator is installed and accessible (e.g. as a tab in Chrome Developer Tools).

# Project folder structure
- `data` contains all the models that are used in the default setup, plus a few extra ReplicaCAD objects
    - `data/default.physics_config.json` contains the physics settings, e.g. gravitational acceleration
    - `data/stages` contain the scene `.glb` files and their corresponding `.stage_config.json` files. In order to choose a scene other than `remake_v0_JustBigStuff_00`, you need to add its 2 files here.
    - `data/objects` contains the hand models as well as the ReplicaCAD objects that can be spawned. It also contains some extra objects you may use.
- `js` contains the JS source code of the hand demo site.
- `lib` contains the JS transpiled version of Habitat-sim, as well as some general use utils JS files. This folder is what gets modified when you run `build_and_install_habitat_sim_js.sh`.
- `standalone.html` is the hosted site, and `style.css` is its stylesheet.

# Habitat-sim JS build

The Habitat-sim JS build is a webassembly build of the Habitat simulator. It's built by compiling Habitat-sim C++ code to webassembly using the Emscripten compiler. The build outputs are `hsim_bindings.js` and `hsim_bindings.wasm`. `hsim_bindings.js` can be included from Javascript and you can call into the simulator via the bindings defined in `habitat-sim/src/esp/bindings_js/bindings_js.cpp`.

The Habitat-sim JS build doesn't currently have an installer, so we can't get it from npm, conda, pip, or similar. We've added habitat-sim as a git submodule at `habitat-sim` and build it locally to produce `hsim_bindings.js` and `hsim_bindings.wasm`. We copy those files to `examples/web_apps/webxr_hand_demo/lib` as a post-build step in `build_and_install_habitat_sim_js.sh`.

We also utilize some JS utilities provided as .js files inside Habitat-sim. These are also copied from the Habitat-sim source folders to `examples/web_apps/webxr_hand_demo/lib` as part of the build script.
