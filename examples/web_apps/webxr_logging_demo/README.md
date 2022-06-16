# Overview

This example illustrates how to log states in Habitat VR. It is based on this [repository](https://github.com/eundersander/hab_vr_mephisto).

# Known Issues

- We may be doing something wrong in how we handle [IPD](https://support.oculus.com/articles/getting-started/getting-started-with-quest-2/ipd-quest-2/). @BlaiseRitchie reported that her Habitat VR app was unusable on Quest 2 due to vision/IPD problems. A possible source of bugs is that we are constructing our viewport/projection matrices inside Habitat based on eye transformed provided by WebXR, instead of directly using viewport/projection matrices provided by the WebXR API.

# Installation

1. `cd` to the main `habitat_sim` directory. Create a folder for the data, then download the hand demo data:

   ```bash
   $ mkdir examples/web_apps/webxr_logging_demo/task/server_files/habitat_vr_app/data
   $ python -m habitat_sim.utils.datasets_download --uids webxr_hand_demo --data-path examples/web_apps/webxr_logging_demo/task/server_files/habitat_vr_app/data
   ```

2. [Optional] Change the data symlink from absolute to relative. This will make your life easier if you want to run this demo on a VR headset.

   ```bash
   $ symlink_file=examples/web_apps/webxr_logging_demo/task/server_files/habitat_vr_app/data/webxr_hand_demo_dataset
   $ symlink_dir="$(dirname $symlink_file)"
   $ target="$(readlink $symlink_file)"
   $ rm $symlink_file
   $ ln -fs "$(realpath --relative-to=$symlink_dir $target)" $symlink_file
   ```

3. Follow the [instructions](https://github.com/facebookresearch/habitat-sim#experimental-emscripten-webgl-and-web-apps) for installing and activating Emscripten, including `source path/to/emsdk_env.sh` or similar to configure env variables.
4. `cd` to this directory and run

   ```bash
   $ chmod +x build_and_install_habitat_sim_js.sh
   $ sh build_and_install_habitat_sim_js.sh
   ```

# Testing

## VR emulation in your desktop browser, standalone (without Mephisto)

1. Http-serve the VR app folder:

   ```bash
   $ cd task/server_files/habitat_vr_app
   $ python3 -m http.server
   ```

2. Install the [WebXR emulator](https://blog.mozvr.com/webxr-emulator-extension/) and then restart your browser.
3. Browse to `http://0.0.0.0:8000/standalone.html`
4. Recommended: watch the dev console as the page loads.
5. Once loading is complete, click "Enter VR".
6. In Chrome Developer Tools, find the WebXR tab. From the device list, select Oculus Quest or similar. In the WebXR panel's 3D view, move the headset or controllers to emulate the VR experience.
7. At the bottom of the WebXR tab, click "Exit immersive".
8. Observe the logged head poses in the html textarea.

## As a Mephisto simple static task

1. Follow Mephisto [Getting Started](https://github.com/facebookresearch/mephisto/blob/master/docs/quickstart.md).
2. Make sure `npm` is in the `PATH` (one way to do so is to `source emsdk_env.sh`).
3. Run `python3 task/static_test_script.py`.
4. Watch the console output for something like:

   ```
   Mock task launched: localhost:3000 for preview, localhost:3000/?worker_id=x&assignment_id=0
   ```

5. Browse to the specified URL and follow the earlier instructions for VR emulation in your browser.
6. After exiting immersive VR, click to submit the task, including logged head poses. Mephisto should exit.

## VR on Quest 2 (or other headset)

1. Fix the data symlink as instructed in the [Installation](#installation) section.
2. Using a modification of either method above, serve the content at a public URL. For example, you could upload the `task/server_files/habitat_vr_app` folder to Amazon S3, GitHub pages (see below), or any web host.
3. On Quest 2, open the browser and navigate to the URL. Use the Quest controller to click "Enter VR".
4. Exit immersive VR with the Quest 2 controller's home button.
5. (Mephisto-only) Click to submit the task, including logged head poses.

## Using GitHub pages to host the VR application

This is useful if you want to test only the VR part of the application before deploying. To integrate with Mephisto, see the [instructions](#deploy-to-heroku) to deploy to Heroku.

1. Copy the `task/server_files/habitat_vr_app` to some location in your computer (outside of an existing git repository).
2. Create a new empty public repository on your GitHub account.
3. Push this folder into that repository.
4. Enable GitHub Pages by going to `Settings > Code and automation > Pages`, choosing a branch under `Source`, and clicking `Save`.
5. Navigate to the `standalone.html` page using the Oculus browser.

# Deploy to Heroku

Once you're done with testing, you can deploy the full application (VR + Mephisto) to Heroku. Make sure you initialized Heroku when setting up Mephisto, and then change the architect in `example.yaml` from `local` to `heroku`. Be sure to also specify a value for `heroku_app_name` (such as `vr-app`) when launching the task, otherwise you will have to type a randomly generated name that includes an hexadecimal string on the Oculus browser.

```bash
$ python task/static_test_script.py mephisto.architect.heroku_app_name=vr-app
```

# Troubleshooting

If you encounter Habitat-sim build errors, make sure you have Emscripten properly installed and activated. Delete the build output folders (`habitat-sim/build_corrade-rc` and `habitat-sim/build_js`) and rebuild.

If the VR app fails to load in your desktop browser, look for errors in the browser dev console. Make sure you have data files installed properly, e.g. `task/server_files/habitat_vr_app/data/webxr_hand_demo_dataset` points to the correct relative directory, and all the expected files are present inside. Make sure the WebXR emulator is installed and accessible (e.g. as a tab in Chrome Developer Tools).

When loading the Heroku app on the headset, if the page seems stuck at `Downloading: 0/1`, make sure that the browser is opening the page using `https` (the Quest browser seems to default to `http`). After that, wait a little bit (the assets can take some time to download). If you see the `Enter VR` link but clicking it seems have no effect, try the following

1. Using the handheld controller, point the cursor to the `Enter VR` link.
2. Barely press the trigger button, very gently.
3. A browser dialog asking you if you want to enter the immersive experience should pop up.
4. Press `Allow`.

# Project folder structure

`task` mirrors the structure of the Mephisto [simple_static_task example](https://github.com/facebookresearch/Mephisto/tree/master/examples/simple_static_task).

`task/server_files/habitat_vr_app` is a mostly-standalone Habitat web app.

`task/server_files/habitat_vr_app/data` contains the VR app's data files. These are 3D models and metadata files used by the simulator.

`task/server_files/habitat_vr_app/standalone.html` and `task/server_files/habitat_vr_task.html` are nearly identical. `standalone.html` is a full html page for hosting the VR app without Mephisto. `habitat_vr_task.html` is an html snippet which becomes part of the Mephisto task (see `task/conf/example.yaml`).

# Habitat-sim JS build

The Habitat-sim JS build is a webassembly build of the Habitat simulator. It's built by compiling Habitat-sim C++ code to webassembly using the Emscripten compiler. The build outputs are `hsim_bindings.js` and `hsim_bindings.wasm`. `hsim_bindings.js` can be included from Javascript and you can call into the simulator via the bindings defined in `habitat-sim/src/esp/bindings_js/bindings_js.cpp`. These files are automatically copied to the project folder as a post-build step when running `build_and_install_habitat_sim_js.sh`.

In the future, we may also utilize some JS utilities provided as .js files inside Habitat-sim; those can be symlinked or copied from the Habitat-sim source folders to `task/server_files/habitat_vr_app/lib/habitat-sim-js/`.
