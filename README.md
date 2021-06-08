[![CircleCI](https://circleci.com/gh/facebookresearch/habitat-sim.svg?style=shield)](https://circleci.com/gh/facebookresearch/habitat-sim)
[![codecov](https://codecov.io/gh/facebookresearch/habitat-sim/branch/master/graph/badge.svg)](https://codecov.io/gh/facebookresearch/habitat-sim)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/habitat-sim/blob/master/LICENSE)
[![Conda Version Badge](https://img.shields.io/conda/vn/aihabitat/habitat-sim?color=blue&label=conda%20version)](https://anaconda.org/aihabitat/habitat-sim)
[![Conda Platforms support Badge](https://img.shields.io/conda/pn/aihabitat/habitat-sim?color=orange&label=platforms)](https://anaconda.org/aihabitat/habitat-sim)
[![Documentation](https://img.shields.io/badge/docs-automated-green.svg)](https://aihabitat.org/docs/habitat-sim/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Python 3.6, 3.7, 3.8](https://img.shields.io/badge/python-3.6%20%7C%203.7%20%7C%203.8-blue.svg)](https://www.python.org/downloads/release/)
[![Supports Bullet](https://img.shields.io/static/v1?label=supports&message=Bullet%20Physics&color=informational&link=https://opensource.google/projects/bullet3)](https://opensource.google/projects/bullet3)
[![Slack Join](http://img.shields.io/static/v1?label=Join%20us%20on&message=%23habitat-dev&labelColor=%234A154B&logo=slack)](https://join.slack.com/t/ai-habitat/shared_invite/enQtNjY1MzM1NDE4MTk2LTZhMzdmYWMwODZlNjg5MjZiZjExOTBjOTg5MmRiZTVhOWQyNzk0OTMyN2E1ZTEzZTNjMWM0MjBkN2VhMjQxMDI)
[![Twitter Follow](https://img.shields.io/twitter/follow/ai_habitat?style=social)](https://twitter.com/ai_habitat)

# Habitat-Sim

A flexible, high-performance 3D simulator with configurable agents, multiple sensors, and generic 3D dataset handling (with built-in support for [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), [Replica](https://github.com/facebookresearch/Replica-Dataset), and other datasets).
When rendering a scene from the Matterport3D dataset, Habitat-Sim achieves several thousand frames per second (FPS) running single-threaded, and reaches <a href="#fps_table"><b>over 10,000 FPS multi-process</b></a> on a single GPU!


[Habitat-Lab](https://github.com/facebookresearch/habitat-lab) uses [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) as the core simulator and is a modular high-level library for end-to-end experiments in embodied AI -- defining embodied AI tasks (e.g. navigation, instruction following, question answering), training agents (via imitation or reinforcement learning, or no learning at all as in classical SLAM), and benchmarking their performance on the defined tasks using standard metrics.

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/habitat-sim/)

[![Habitat Demo](https://img.shields.io/static/v1?label=WebGL&message=Try%20AI%20Habitat%20In%20Your%20Browser%20&color=blue&logo=webgl&labelColor=%23990000&style=for-the-badge&link=https://aihabitat.org/demo)](https://aihabitat.org/demo)
<p align="center">
  <img src="docs/images/habitat_compressed.gif" height="400">
</p>

---

## Table of contents
   1. [URDF Prototype Details](#urdf-prototype-details)
   1. [Motivation](#motivation)
   1. [Citing Habitat](#citing-habitat)
   1. [Details](#details)
   1. [Performance](#performance)
   1. [Installation](#installation)
   1. [Testing](#testing)
   1. [Documentation](#documentation)
   1. [Rendering to GPU Tensors](#rendering-to-gpu-tensors)
   1. [Experimental: Emscripten, WebGL, and Web Apps](##experimental-emscripten-webgl-and-web-apps)
   1. [Datasets](#datasets)
   1. [Examples](#examples)
   1. [Code Style](#code-style)
   1. [Development Tips](#development-tips)
   1. [Acknowledgments](#acknowledgments)
   1. [External Contributions](#external-contributions)
   1. [License](#license)
   1. [References](#references)

## URDF Prototype Details ##
URDF support and articulated objects are coming soon to Habitat! This branch is an unstable prototype for this feature.

Want to play with robots? Try the demo in viewer.
- To get started:
  - Download and extract the [URDF demo assets](https://dl.fbaipublicfiles.com/habitat/URDF_demo_assets.zip)  into `path-to-habitat-sim/data/`.
  - Download and extract the [examples scenes](http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip) into `path-to-habitat-sim/data/`.
  - (optionally) Download and extract the [example rigid objects](http://dl.fbaipublicfiles.com/habitat/objects_v0.2.zip) into `path-to-habitat-sim/data/objects/` (structure should be e.g.: `path-to-habitat-sim/data/objects/banana.glb`).
  - Install the simulator for interactivity (with bullet) as described in [Installation](#installation).
- Now run the viewer with physics as described in [Testing](#testing).
   ```
   ./build/viewer --enable-physics -- "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
   ```
- Use keys to add robots from URDF (you can add multiple):
  - `0` - aliengo
  - `1` - kuka iiwa (fixed base)
  - `3` - locobot (no arm)
  - `4` - locobot (with arm)
- Other useful key commands:
  - `-` - remove last added robot
  - `m` - toggle kinematic/dynamic for last robot added
  - `v` - invert gravity
  - `o` - add random rigid object
  - `8` - add random rigid primitive
  - `u` - remove last rigid object added
  - `wasd` - move agent on navmesh
  - `n` - toggle navmesh visualization
  - `9` - place agent at random point from navmesh
  - `esc` - exit
- Useful mouse interactions:
  - `mouse_wheel` selects an interaction mode (current mode displayed in GUI)
    - `GRAB` - click to raycast on collision objects (box displays contact point).
      - `LEFT` - Create a ball joint constraint at the contact point until mouse released (also sets the objct to DYNAMIC). Drag to move the constraint point in the camera plane.
      - `RIGHT` - Sets objects to KINEMATIC (not robots). Drag to move the object or robot root in the camera plane.
    - `THROW` - Toss a sphere! (tip: clean up the last one with `u`).
    - `OPEN|CLOSE` - Experimental activation mode. On click activates scripted states. See `src/utils/viewer/viewer.cpp` for implementation details.
    - `DOF(#)` - Kinematically set dof position of last robot added.
      - `LEFT` - Click/drag horizontally to manually set the state of a dof.
      - `RIGHT` - Click/drag horizontally to select the controlled dof.




Also try the python API via the tutorial script (after download/install as above):
```
python path-to-habitat-sim/examples/tutorials/URDF_robotics_tutorial.py
```
**Note: this script will save and open a video file. On OSX you must close QuickTime Player between runs to see the updated video.**


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

Habitat-Sim can be installed in 3 ways:
1. Via Conda - Recommended method for most users. Stable release and nightly builds.
2. Via Docker - Updated approximately once per year for [Habitat Challenge](https://aihabitat.org/challenge/).
3. Via Source - For active development.

### [Recommended] Conda Packages

Habitat is under active development, and we advise users to restrict themselves to [stable releases](https://github.com/facebookresearch/habitat-sim/releases).
Starting with v0.1.4, we provide [conda packages for each release](https://anaconda.org/aihabitat). This is the recommended and easiest way to install Habitat-Sim.

Assuming you have [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/) installed, let's prepare a conda env:
```bash
# We require python>=3.6 and cmake>=3.10
conda create -n habitat python=3.6 cmake=3.14.0
conda activate habitat
pip install -r requirements.txt
```

Next, pick one of the options below depending on your system/needs:

- To install on machines with an attached display:
   ```bash
     conda install habitat-sim -c conda-forge -c aihabitat
   ```
- To install on headless machines (i.e. without an attached display, e.g. in a cluster) and machines with multiple GPUs:
   ```
   conda install habitat-sim headless -c conda-forge -c aihabitat
   ```
- To install habitat-sim with bullet physics
   ```
   conda install habitat-sim withbullet -c conda-forge -c aihabitat
   ```

- Note: Build parameters can be chained together. For instance, to install habitat-sim with physics on headless machines:
   ```
   conda install habitat-sim withbullet headless -c conda-forge -c aihabitat
   ```

Conda packages for older versions can installed by explicitly specifying the version, e.g. `conda install habitat-sim=0.1.6 -c conda-forge -c aihabitat`.

We also provide a [nightly conda build for the master branch](https://anaconda.org/aihabitat-nightly). However, this should only be used if you need a specific feature not yet in the latest release version. To get the nightly build of the latest master, simply swap `-c aihabitat` for `-c aihabitat-nightly`.


### Docker Image

We provide a pre-built docker container for [habitat-lab](https://github.com/facebookresearch/habitat-lab) and habitat-sim, refer to [habitat-docker-setup](https://github.com/facebookresearch/habitat-lab#docker-setup).

### From Source

Read [build instructions and common build issues](BUILD_FROM_SOURCE.md).

## Testing

1. Run our python data download utility to retrieve the test assets:
   ```bash
   python -m habitat_sim.utils.datasets_download --uids habitat_test_scenes --data-path /path/to/data/
   ```

1. **Interactive testing**: Use the interactive viewer included with Habitat-Sim
   ```bash
   # ./build/viewer if compiling locally
   habitat-viewer /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   You should be able to control an agent in this test scene.
   Use W/A/S/D keys to move forward/left/backward/right and arrow keys or mouse to control gaze direction (look up/down/left/right).
   Try to find the picture of a woman surrounded by a wreath.
   Have fun!

1. **Physical interactions**: If you would like to try out habitat with dynamical objects using the interactive viewer:
   First setup the test object assets by running the data download utility:
   ```bash
   python -m habitat_sim.utils.datasets_download --uids habitat_example_objects --data-path /path/to/data/
   ```

   To run an interactive C++ example GUI application with physics enabled run
   ```bash
   # ./build/viewer if compiling locally
   habitat-viewer --enable-physics --object-dir data/objects/example_objects -- data/scene_datasets/habitat-test-scenes/apartment_1.glb
   ```
   The viewer application will output user interface help to the console at runtime.

1. **Non-interactive testing**: Run the example script:
   ```bash
   python examples/example.py --scene /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   The agent will traverse a particular path and you should see the performance stats at the very end, something like this:
  `640 x 480, total time: 3.208 sec. FPS: 311.7`.
  Note that the test scenes do not provide semantic meshes.
  If you would like to test the semantic sensors via `example.py`, please use the data from the Matterport3D dataset (see [Datasets](#Datasets)).
  We have also provided an [example demo](https://aihabitat.org/docs/habitat-lab/habitat-lab-demo.html) for reference.

    To run a physics example in python (after building with "Physics simulation via Bullet"):
    ```bash
    python examples/example.py --scene /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb --enable_physics
    ```
    Note that in this mode the agent will be frozen and oriented toward the spawned physical objects. Additionally, `--save_png` can be used to output agent visual observation frames of the physical scene to the current directory.


### Common testing issues

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


## Experimental: Emscripten, WebGL, and Web Apps

Build `hsim_bindings.wasm`, our experimental Emscripten-compiled webassembly binary for use in WebGL html/Javascript apps. See the available Javascript bindings at `src/esp/bindings_js/bindings_js.cpp`. Check out our `bindings.html` demo app:

1. Download the [test scenes](http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip) and extract locally to habitat-sim creating habitat-sim/data.
1. Download and install [emscripten](https://emscripten.org/docs/getting_started/downloads.html) (you need at least version 1.38.48, newer versions such as 2.0.6 work too)
1. Activate your emsdk environment
1. Build using `./build_js.sh [--bullet]`
1. Run webserver
   ```bash
   python -m http.server 8000 --bind 127.0.0.1
   ```
1. Open <http://127.0.0.1:8000/build_js/esp/bindings_js/bindings.html>

You can build `hsim_bindings.wasm` without the demo web apps like so:
- `./build_js.sh --no-web-apps [--bullet]`

## Datasets

- The full Matterport3D (MP3D) dataset for use with Habitat can be downloaded using the official [Matterport3D](https://niessner.github.io/Matterport/) download script as follows: `python download_mp.py --task habitat -o path/to/download/`. You only need the habitat zip archive and not the entire Matterport3D dataset. Note that this download script requires python 2.7 to run.
- The Gibson dataset for use with Habitat can be downloaded by agreeing to the terms of use in the [Gibson](https://github.com/StanfordVL/GibsonEnv#database) repository.
- Semantic information for Gibson is available from the [3DSceneGraph](https://3dscenegraph.stanford.edu/) dataset. The semantic data will need to be converted before it can be used within Habitat:
   ```bash
   tools/gen_gibson_semantics.sh /path/to/3DSceneGraph_medium/automated_graph /path/to/GibsonDataset /path/to/output
   ```
   To use semantics, you will need to enable the semantic sensor.
- To work with the Replica dataset, you need a file called ```sorted_faces.bin``` for each model. Such files (1 file per model), along with a convenient setup script can be downloaded from here: [sorted_faces.zip](http://dl.fbaipublicfiles.com/habitat/sorted_faces.zip). You need:
```
  - Download the file from the above link;
  - Unzip it;
  - Use the script within to copy each data file to its corresponding folder (You will have to provide the path to the folder containing all replica models. For example, ~/models/replica/);
```
## Examples

Load a specific MP3D or Gibson house: `examples/example.py --scene path/to/mp3d/house_id.glb`.

Additional arguments to `example.py` are provided to change the sensor configuration, print statistics of the semantic annotations in a scene, compute action-space shortest path trajectories, and set other useful functionality. Refer to the `example.py` and `demo_runner.py` source files for an overview.

To reproduce the benchmark table from above run `examples/benchmark.py --scene /path/to/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb`.


## Code Style

We use `clang-format-12` for linting and code style enforcement of c++ code.
Code style follows the [Google C++ guidelines](https://google.github.io/styleguide/cppguide.html).
Install `clang-format-12` through `brew install clang-format` on macOS.  For other systems, `clang-format-12` can be installed via `conda install clangdev -c conda-forge` or by downloading binaries or sources from [releases.llvm.org/download](http://releases.llvm.org/download.html).
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

Habitat-Sim is MIT licensed. See the [LICENSE](LICENSE) for details.

## References

1. [Habitat: A Platform for Embodied AI Research](https://arxiv.org/abs/1904.01201). Manolis Savva, Abhishek Kadian, Oleksandr Maksymets, Yili Zhao, Erik Wijmans, Bhavana Jain, Julian Straub, Jia Liu, Vladlen Koltun, Jitendra Malik, Devi Parikh, Dhruv Batra. IEEE/CVF International Conference on Computer Vision (ICCV), 2019.
