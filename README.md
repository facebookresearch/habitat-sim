[![CircleCI](https://circleci.com/gh/facebookresearch/habitat-sim.svg?style=shield)](https://circleci.com/gh/facebookresearch/habitat-sim)
[![codecov](https://codecov.io/gh/facebookresearch/habitat-sim/branch/master/graph/badge.svg)](https://codecov.io/gh/facebookresearch/habitat-sim)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/habitat-sim/blob/master/LICENSE)

# Habitat-Sim

A flexible, high-performance 3D simulator with configurable agents, multiple sensors, and generic 3D dataset handling (with built-in support for [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), [Replica](https://github.com/facebookresearch/Replica-Dataset), and other datasets).
When rendering a scene from the Matterport3D dataset, Habitat-Sim achieves several thousand frames per second (FPS) running single-threaded, and reaches <a href="#fps_table"><b>over 10,000 FPS multi-process</b></a> on a single GPU!

[Try Habitat in your browser!](https://aihabitat.org/demo)

<p align="center">
  <img src="docs/images/habitat_compressed.gif" height="400">
</p>

---

## Table of contents
   1. [Motivation](#motivation)
   1. [Citing Habitat](#citing-habitat)
   1. [Details](#details)
   1. [Performance](#performance)
   1. [Installation](#installation)
   1. [Common build issues](#common-build-issues)
   1. [Testing](#testing)
   1. [Common testing issues](#common-testing-issues)
   1. [Documentation](#documentation)
   1. [Rendering to GPU Tensors](#rendering-to-gpu-tensors)
   1. [WebGL](#webgl)
   1. [Datasets](#datasets)
   1. [Examples](#examples)
   1. [Acknowledgments](#acknowledgments)
   1. [External Contributions](#external-contributions)
   1. [License](#license)
   1. [References](#references)

## Motivation ##
AI Habitat enables training of embodied AI agents (virtual robots) in a highly photorealistic & efficient 3D simulator, before transferring the learned skills to reality.
This empowers a paradigm shift from 'internet AI' based on static datasets (e.g. ImageNet, COCO, VQA) to embodied AI where agents act within realistic environments, bringing to the fore active perception, long-term planning, learning from interaction, and holding a dialog grounded in an environment.

## Citing Habitat
If you use the Habitat platform in your research, please cite the following [paper](https://arxiv.org/abs/1904.01201):

```
@inproceedings{habitat19iccv,
  title     =     {Habitat: {A} {P}latform for {E}mbodied {AI} {R}esearch},
  author    =     {Manolis Savva and Abhishek Kadian and Oleksandr Maksymets and Yili Zhao and Erik Wijmans and Bhavana Jain and Julian Straub and Jia Liu and Vladlen Koltun and Jitendra Malik and Devi Parikh and Dhruv Batra},
  booktitle =     {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
  year      =     {2019}
}
```

Habitat-Sim also builds on work contributed by others.  If you use contributed methods/models, please cite their works.  See the [External Contributions](#external-contributions) section
for a list of what was externally contributed and the corresponding work/citation.

## Details

The Habitat-Sim backend module is implemented in C++ and leverages the [magnum](https://github.com/mosra/magnum) graphics middleware library to support cross-platform deployment on a broad variety of hardware configurations. The architecture of the main abstraction classes is shown below. The design of this module ensures a few key properties:
* Memory-efficient management of 3D environment resources (triangle mesh geometry, textures, shaders) ensuring shared resources are cached and re-used
* Flexible, structured representation of 3D environments using SceneGraphs, allowing for programmatic manipulation of object state, and combination of objects from different environments
* High-efficiency rendering engine with multi-attachment render passes for reduced overhead when multiple sensors are active
* Arbitrary numbers of Agents and corresponding Sensors that can be linked to a 3D environment by attachment to a SceneGraph.

<p align="center">
 <img src='docs/images/habitat_architecture.png' width="800" />
 <p align="center"><i>Architecture of <code>Habitat-Sim</code> main classes</i></p>
</p>

The Simulator delegates management of all resources related to 3D environments to a ResourceManager that is responsible for loading and caching 3D environment data from a variety of on-disk formats. These resources are used within SceneGraphs at the level of individual SceneNodes that represent distinct objects or regions in a particular Scene. Agents and their Sensors are instantiated by being attached to SceneNodes in a particular SceneGraph.

<p align="center">
 <img src='docs/images/sensor-data.png' width="600" />
 <p align="center"><i>Example rendered sensor observations</i></p>
</p>


## Performance
The table below reports performance statistics for a test scene from the Matterport3D dataset (id `17DRP5sb8fy`) on a `Xeon E5-2690 v4 CPU` and `Nvidia Titan Xp`. Single-thread performance reaches several thousand frames per second, while multi-process operation with several independent simulation backends can reach more than 10,000 frames per second on a single GPU!
<table class="table" id="fps_table">
 <tr>
   <td></td>
   <th colspan="3"> 1 proc </th>
   <th colspan="3"> 3 procs </th>
   <th colspan="3"> 5 procs </th>
 </tr>
 <tr>
   <th>Sensors / Resolution</th>
   <th>128</th>
   <th>256</th>
   <th>512</th>
   <th>128</th>
   <th>256</th>
   <th>512</th>
   <th>128</th>
   <th>256</th>
   <th>512</th>
 </tr>
 <tr>
   <td>RGB</td>
   <td>4093</td>
   <td>1987</td>
   <td>848</td>
   <td>10638</td>
   <td>3428</td>
   <td>2068</td>
   <td>10592</td>
   <td>3574</td>
   <td>2629</td>
 </tr>
 <tr>
   <td>RGB + depth</td>
   <td>2050</td>
   <td>1042</td>
   <td>423</td>
   <td>5024</td>
   <td>1715</td>
   <td>1042</td>
   <td>5223</td>
   <td>1774</td>
   <td>1348</td>
 </tr>
 <tr>
   <td>RGB + depth + semantics*</td>
   <td>709</td>
   <td>596</td>
   <td>394</td>
   <td>1312</td>
   <td>1219</td>
   <td>979</td>
   <td>1521</td>
   <td>1429</td>
   <td>1291</td>
 </tr>
</table>

Previous simulation platforms that have operated on similar datasets typically produce on the order of a couple hundred frames per second. For example [Gibson](https://github.com/StanfordVL/GibsonEnv#gibson-framerate) reports up to about 150 fps with 8 processes, and [MINOS](https://github.com/minosworld/minos#benchmarking) reports up to about 167 fps with 4 threads.

*Note: The semantic sensor in MP3D houses currently requires the use of additional house 3D meshes with orders of magnitude more geometric complexity leading to reduced performance. We expect this to be addressed in future versions leading to speeds comparable to RGB + depth; stay tuned.

To run the above benchmarks on your machine, see instructions in the [examples](#examples) section.

## Installation

### Docker Image

We provide a pre-built docker container for habitat-api and habitat-sim, refer to [habitat-docker-setup](https://github.com/facebookresearch/habitat-api#docker-setup).

### From Source

We highly recommend installing a [miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/distribution/#download-section) environment (note: python>=3.6 is required). Once you have Anaconda installed, here are the instructions.


1. Clone this github repository.
   ```bash
   # Checkout the latest stable release
   git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
   cd habitat-sim
   ```

   List of stable releases is [available here](https://github.com/facebookresearch/habitat-sim/releases). Master branch contains 'bleeding edge' code and under active development.

1. Install Dependencies

    Common
   ```bash
   # We require python>=3.6 and cmake>=3.10
   conda create -n habitat python=3.6 cmake=3.14.0
   conda activate habitat
   pip install -r requirements.txt
   ```

    Linux (Tested with Ubuntu 18.04 with gcc 7.4.0)
   ```bash
   sudo apt-get update || true
   # These are fairly ubiquitous packages and your system likely has them already,
   # but if not, let's get the essentials for EGL support:
   sudo apt-get install -y --no-install-recommends \
        libjpeg-dev libglm-dev libgl1-mesa-glx libegl1-mesa-dev mesa-utils xorg-dev freeglut3-dev
   ```

   See this [configuration for a full list of dependencies](https://github.com/facebookresearch/habitat-sim/blob/master/.circleci/config.yml#L64) that our CI installs on a clean Ubuntu VM. If you run into build errors later, this is a good place to check if all dependencies are installed.

1. Build Habitat-Sim

    Default build (for machines with a display attached)
   ```bash
   # Assuming we're still within habitat conda environment
   python setup.py install
   ```

    For headless systems (i.e. without an attached display, e.g. in a cluster) and multiple GPU systems
   ```bash
   python setup.py install --headless
   ```

    For systems with CUDA (to build CUDA features)
   ```bash
   python setup.py install --with-cuda
   ```

   (Under development) With physics simulation via [Bullet Physics SDK](https://github.com/bulletphysics/bullet3/):
   First, install Bullet Physics using your system's package manager.

    Mac
   ```bash
   brew install bullet
   ```

    Linux
   ```bash
   sudo apt-get install libbullet-dev
   ```

    Next, enable bullet physics build via:
   ```bash
   python setup.py install --bullet    # build habitat with bullet physics
   ```

   Note1: Build flags stack, *e.g.* to build in headless mode, with CUDA, and bullet, one would use `--headless --with-cuda --bullet`.

   Note2: some Linux distributions might require an additional `--user` flag to deal with permission issues.

   Note3: for active development in Habitat, you might find `./build.sh` instead of `python setup.py install` more useful.


1. [Only if using `build.sh`] For use with [Habitat-API](https://github.com/facebookresearch/habitat-api) and your own python code, add habitat-sim to your `PYTHONPATH`. For example modify your `.bashrc` (or `.bash_profile` in Mac OS X) file by adding the line:
   ```bash
   export PYTHONPATH=$PYTHONPATH:/path/to/habitat-sim/
   ```

## Common build issues

- If your machine has a custom installation location for the nvidia OpenGL and EGL drivers, you may need to manually provide the `EGL_LIBRARY` path to cmake as follows.  Add `-DEGL_LIBRARY=/usr/lib/x86_64-linux-gnu/nvidia-opengl/libEGL.so` to the `build.sh` command line invoking cmake. When running any executable adjust the environment as follows: `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH} examples/example.py`.

- By default, the build process uses all cores available on the system to parallelize. On some virtual machines, this might result in running out of memory. You can serialize the build process via:
   ```bash
   python setup.py build_ext --parallel 1 install
   ```

- Build is tested on Tested with Ubuntu 18.04 with gcc 7.4.0 and MacOS 10.13.6 with Xcode 10 and clang-1000.10.25.5. If you experience compilation issues, please open an issue with the details of your OS and compiler versions.

  We also have a dev slack channel, please follow this [link](https://join.slack.com/t/ai-habitat/shared_invite/enQtNjY1MzM1NDE4MTk2LTZhMzdmYWMwODZlNjg5MjZiZjExOTBjOTg5MmRiZTVhOWQyNzk0OTMyN2E1ZTEzZTNjMWM0MjBkN2VhMjQxMDI) to get added to the channel.

## Testing

1. Download the test scenes from this [link](http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip) and extract locally.

1. **Interactive testing**: Use the interactive viewer included with Habitat-Sim
   ```bash
   ./build/viewer /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   You should be able to control an agent in this test scene.
   Use W/A/S/D keys to move forward/left/backward/right and arrow keys to control gaze direction (look up/down/left/right).
   Try to find the picture of a woman surrounded by a wreath.
   Have fun!

1. **Physical interactions**: If you would like to try out habitat with dynamical objects, first download our pre-processed object data-set from this [link](http://dl.fbaipublicfiles.com/habitat/objects_v0.1.zip) and extract as `habitat-sim/data/objects/`.

   To run an interactive C++ example GUI application with physics enabled run
   ```bash
   ./build/viewer --enable-physics /path/to/data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
   ```
   Use W/A/S/D keys to move forward/left/backward/right and arrow keys to control gaze direction (look up/down/left/right).
   Press 'o' key to add a random object, press 'p/f/t' to apply impulse/force/torque to the last added object or press 'u' to remove it.
   Press 'k' to kinematically nudge the last added object in a random direction.
   Press 'v' key to invert gravity.

1. **Non-interactive testing**: Run the example script:
   ```bash
   python examples/example.py --scene /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   The agent will traverse a particular path and you should see the performance stats at the very end, something like this:
  `640 x 480, total time: 3.208 sec. FPS: 311.7`.
  Note that the test scenes do not provide semantic meshes.
  If you would like to test the semantic sensors via `example.py`, please use the data from the Matterport3D dataset (see [Datasets](#Datasets)).
  We have also provided an [example demo](https://aihabitat.org/docs/habitat-api/habitat-api-demo.html) for reference.

    To run a physics example in python (after building with "Physics simulation via Bullet"):
    ```bash
    python examples/example.py --scene /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb --enable_physics
    ```
    Note that in this mode the agent will be frozen and oriented toward the spawned physical objects. Additionally, `--save_png` can be used to output agent visual observation frames of the physical scene to the current directory.


## Common testing issues

- If you are running on a remote machine and experience display errors when initializing the simulator, e.g.
   ```bash
    X11: The DISPLAY environment variable is missing
    Could not initialize GLFW
   ```

  ensure you do not have `DISPLAY` defined in your environment (run `unset DISPLAY` to undefine the variable)

- If you see libGL errors like:

   ```bash
    X11: The DISPLAY environment variable is missing
    Could not initialize GLFW
   ```

    chances are your libGL is located at a non-standard location. See e.g. [this issue](https://askubuntu.com/questions/541343/problems-with-libgl-fbconfigs-swrast-through-each-update).

## Documentation

Browse the online [Habitat-Sim documentation](https://aihabitat.org/docs/habitat-sim/index.html). 

To get you started, see the [Lighting Setup tutorial](https://aihabitat.org/docs/habitat-sim/lighting-setups.html) for adding new objects to existing scenes and relighting the scene & objects. The [Image Extractor tutorial](https://aihabitat.org/docs/habitat-sim/image-extractor.html) shows how to get images from scenes loaded in Habitat-Sim. 

## Rendering to GPU Tensors

We support transfering rendering results directly to a [PyTorch](https://pytorch.org/) tensor via CUDA-GL Interop.
This feature is built by when Habitat-Sim is compiled with CUDA, i.e. built with `--with-cuda`.  To enable it, set the
`gpu2gpu_transfer` flag of the sensor specification(s) to `True`

This is implemented in a way that is reasonably agnostic to the exact GPU-Tensor library being used, but we currently have only implemented support for PyTorch.


## WebGL

1. Download the [test scenes](http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip) and extract locally to habitat-sim creating habitat-sim/data.
1. Download and install [emscripten](https://emscripten.org/docs/getting_started/downloads.html) (version 1.38.38 is verified to work)
1. Set EMSCRIPTEN in your environment
   ```bash
   export EMSCRIPTEN=/pathto/emsdk/fastcomp/emscripten
1. Build using `./build_js.sh`
1. Run webserver
   ```bash
   python -m http.server 8000 --bind 127.0.0.1
   ```
1. Open <http://127.0.0.1:8000/build_js/esp/bindings_js/bindings.html>

## Datasets

- The full Matterport3D (MP3D) dataset for use with Habitat can be downloaded using the official [Matterport3D](https://niessner.github.io/Matterport/) download script as follows: `python download_mp.py --task habitat -o path/to/download/`. You only need the habitat zip archive and not the entire Matterport3D dataset. Note that this download script requires python 2.7 to run.
- The Gibson dataset for use with Habitat can be downloaded by agreeing to the terms of use in the [Gibson](https://github.com/StanfordVL/GibsonEnv#database) repository.
- Semantic information for Gibson is available from the [3DSceneGraph](https://3dscenegraph.stanford.edu/) dataset. The semantic data will need to be converted before it can be used within Habitat:
   ```bash
   tools/gen_gibson_semantics.sh /path/to/3DSceneGraph_medium/automated_graph /path/to/GibsonDataset /path/to/output
   ```
   To use semantics, you will need to enable the semantic sensor.

## Examples

Load a specific MP3D or Gibson house: `examples/example.py --scene path/to/mp3d/house_id.glb`.

Additional arguments to `example.py` are provided to change the sensor configuration, print statistics of the semantic annotations in a scene, compute action-space shortest path trajectories, and set other useful functionality. Refer to the `example.py` and `demo_runner.py` source files for an overview.

To reproduce the benchmark table from above run `examples/benchmark.py --scene /path/to/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb`.


## Code style

We use `clang-format-8` for linting and code style enforcement of c++ code.
Code style follows the [Google C++ guidelines](https://google.github.io/styleguide/cppguide.html).
Install `clang-format-8` through `brew install clang-format` on macOS.  For other systems, `clang-format-8` can be installed via `conda install clangdev -c conda-forge` or by downloading binaries or sources from [releases.llvm.org/download](http://releases.llvm.org/download.html).
For vim integration add to your .vimrc file `map <C-K> :%!clang-format<cr>` and use Ctrl+K to format entire file.
Integration plugin for [vscode](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format).

We use `black` and `isort` for linting and code style of python code.
Install `black` and `isort` through `pip install -U black isort`.
They can then be ran via `black .` and `isort`.

We use `eslint` with `prettier` plugin for linting, formatting and code style of JS code.
Install these dependencies through `npm install`. Then, for fixing linting/formatting errors run `npm run lint-fix`. Make sure you have a node version > 8 for this.

We also offer pre-commit hooks to help with automatically formatting code.
Install the pre-commit hooks with `pip install pre-commit && pre-commit install`.

## Development Tips

1. Install `ninja` (`sudo apt install ninja-build` on Linux, or `brew install ninja` on macOS) for significantly faster incremental builds
1. Install `ccache` (`sudo apt install ccache` on Linux, or `brew install ccache` on macOS) for significantly faster clean re-builds and builds with slightly different settings
1. You can skip reinstalling magnum every time by adding the argument of `--skip-install-magnum` to either `build.sh` or `setup.py`.  Note that you will still need to install magnum bindings once.
1. Arguments to `build.sh` and `setup.py` can be cached between subsequent invocations with the flag `--cache-args` on the _first_ invocation.

## Acknowledgments
The Habitat project would not have been possible without the support and contributions of many individuals. We would like to thank Xinlei Chen, Georgia Gkioxari, Daniel Gordon, Leonidas Guibas, Saurabh Gupta, Or Litany, Marcus Rohrbach, Amanpreet Singh, Devendra Singh Chaplot, Yuandong Tian, and Yuxin Wu for many helpful conversations and guidance on the design and development of the Habitat platform.


## External Contributions

* If you use the noise model from PyRobot, please cite the their [technical report](https://github.com/facebookresearch/pyrobot#citation).


    Specifically, the noise model used for the noisy control functions named `pyrobot_*` and defined in `habitat_sim/agent/controls/pyrobot_noisy_controls.py`


* If you use the Redwood Depth Noise Model, please cite their [paper](http://redwood-data.org/indoor/)

    Specifically, the noise model defined in `habitat_sim/sensors/noise_models/redwood_depth_noise_model.py` and `src/esp/sensor/RedwoodNoiseModel.*`


## License

Habitat-Sim is MIT licensed. See the LICENSE file for details.

## References

1. [Habitat: A Platform for Embodied AI Research](https://arxiv.org/abs/1904.01201). Manolis Savva, Abhishek Kadian, Oleksandr Maksymets, Yili Zhao, Erik Wijmans, Bhavana Jain, Julian Straub, Jia Liu, Vladlen Koltun, Jitendra Malik, Devi Parikh, Dhruv Batra. IEEE/CVF International Conference on Computer Vision (ICCV), 2019.
