<p align="center">
  <img width = "50%" src='docs/logos/habitat_logo_with_text_horizontal_blue.png' />
</p>

--------------------------------------------------------------------------------



# Habitat-Sim

A flexible, high-performance 3D simulator with configurable agents, multiple sensors, and generic 3D dataset handling (with built-in support for [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), [Replica](https://github.com/facebookresearch/Replica-Dataset), and other datasets).
When rendering a scene from the Matterport3D dataset, Habitat-Sim achieves several thousand frames per second (FPS) running single-threaded, and reaches <a href="#fps_table"><b>over 10,000 FPS multi-process</b></a> on a single GPU!

<p align="center">
  <img src="docs/images/habitat_compressed.gif" height="400">
</p>

---

## Table of contents
   0. [Updates](#updates)
   0. [Motivation](#motivation)
   0. [Citing Habitat](#citing-habitat)
   0. [Details](#details)
   0. [Performance](#performance)
   0. [Quick installation](#quick-installation)
   0. [Testing](#Testing)
   0. [Developer installation and getting started](#developer-installation-and-getting-started)
   0. [Datasets](#datasets)
   0. [Examples](#examples)
   0. [Common issues](#common-issues)
   0. [Acknowledgments](#acknowledgments)
   0. [License](#license)
   0. [References](#references)

## Updates ##

* **Urgent Update** (4/2/19) There was a bug in the code used to generate the semantic meshes habitat verion of MP3D.  If you do not have a README stating this was fixed in your download of this dataset, please redownload using the `download_mp.py` script.

## Motivation ##
AI Habitat enables training of embodied AI agents (virtual robots) in a highly photorealistic & efficient 3D simulator, before transferring the learned skills to reality.
This empowers a paradigm shift from 'internet AI' based on static datasets (e.g. ImageNet, COCO, VQA) to embodied AI where agents act within realistic environments, bringing to the fore active perception, long-term planning, learning from interaction, and holding a dialog grounded in an environment.

## Citing Habitat
If you use the Habitat platform in your research, please cite the following [technical report](https://arxiv.org/abs/1904.01201):
```
@article{habitat19arxiv,
  title =   {Habitat: A Platform for Embodied AI Research},
  author =  {Manolis Savva, Abhishek Kadian, Oleksandr Maksymets, Yili Zhao, Erik Wijmans, Bhavana Jain, Julian Straub, Jia Liu, Vladlen Koltun, Jitendra Malik, Devi Parikh and Dhruv Batra},
  journal = {arXiv preprint arXiv:1904.01201},
  year =    {2019}
}
```

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
   <td>439</td>
   <td>346</td>
   <td>185</td>
   <td>502</td>
   <td>385</td>
   <td>336</td>
   <td>500</td>
   <td>390</td>
   <td>367</td>
 </tr>
</table>

Previous simulation platforms that have operated on similar datasets typically produce on the order of a couple hundred frames per second. For example [Gibson](https://github.com/StanfordVL/GibsonEnv#gibson-framerate) reports up to about 150 fps with 8 processes, and [MINOS](https://github.com/minosworld/minos#benchmarking) reports up to about 167 fps with 4 threads.

*Note: The semantic sensor in MP3D houses currently requires the use of additional house 3D meshes with orders of magnitude more geometric complexity leading to reduced performance. We expect this to be addressed in future versions leading to speeds comparable to RGB + depth; stay tuned.

To run the above benchmarks on your machine, see instructions in the [examples](#examples) section.

## Quick installation

1. Clone the repo
1. Install numpy in your python env of choice (e.g., `pip install numpy` or `conda install numpy`)
1. Install Habitat-Sim via `python setup.py install` in your python env of choice (note: python 3 is required)

    Use `python setup.py install --headless` for headless systems (i.e. without an attached display) or if you need multi-gpu support.

    **Note**: the build requires `cmake` version 3.10 or newer. You can install cmake through `conda install cmake` or download directly from [https://cmake.org/download/](https://cmake.org/download/)

## Testing

1. Download the test scenes from this [link](http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip) and extract locally.
1. **Interactive testing**: Use the interactive viewer included with Habitat-Sim
   ```bash
   build/viewer /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   You should be able to control an agent in this test scene.
   Use W/A/S/D keys to move forward/left/backward/right and arrow keys to control gaze direction (look up/down/left/right).
   Try to find the picture of a woman surrounded by a wreath.
   Have fun!
1. **Non-interactive testing**: Run the example script:
   ```bash
   python examples/example.py --scene /path/to/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb
   ```
   The agent will traverse a particular path and you should see the performance stats at the very end, something like this:
`640 x 480, total time: 3.208 sec. FPS: 311.7`.
Note that the test scenes do not provide semantic meshes.
If you would like to test the semantic sensors via `example.py`, please use the data from the Matterport3D dataset (see [Datasets](#Datasets)).

We also provide a docker setup for habitat-stack, refer to [habitat-docker-setup](https://github.com/facebookresearch/habitat-api#docker-setup).


## Developer installation and getting started

1. Clone the repo.
1. Install numpy in your python env of choice (e.g., `pip install numpy` or `conda install numpy`)
1. Install dependencies in your python env of choice (e.g., `pip install -r requirements.txt`)
1. If you are using a virtual/conda environment, make sure to use the same environment throughout the rest of the build
1. Build using `./build.sh` or `./build.sh --headless` for headless systems (i.e. without an attached display) or if you need multi-gpu support
1. Test the build as described above (except that the interactive viewer is at `build/utils/viewer/viewer`).
1. For use with [Habitat-API](https://github.com/facebookresearch/habitat-api) and your own python code, add habitat-sim to your `PYTHONPATH` (not necessary if you used `python setup.py install` instead of `build.sh`). For example modify your `.bashrc` (or `.bash_profile` in Mac osx) file by adding the line:
   ```bash
   export PYTHONPATH=$PYTHONPATH:/path/to/habitat-sim/
   ```

## Datasets

- The full Matterport3D (MP3D) dataset for use with Habitat can be downloaded using the official [Matterport3D](https://niessner.github.io/Matterport/) download script as follows: `python download_mp.py --task habitat -o path/to/download/`. You only need the habitat zip archive and not the entire Matterport3D dataset. Note that this download script requires python 2.7 to run.
- The Gibson dataset for use with Habitat can be downloaded by agreeing to the terms of use in the [Gibson](https://github.com/StanfordVL/GibsonEnv#database) repository

## Examples

Load a specific MP3D or Gibson house: `examples/example.py --scene path/to/mp3d/house_id.glb`.

Additional arguments to `example.py` are provided to change the sensor configuration, print statistics of the semantic annotations in a scene, compute action-space shortest path trajectories, and set other useful functionality. Refer to the `example.py` and `demo_runner.py` source files for an overview.

To reproduce the benchmark table from above run `examples/benchmark.py --scene /path/to/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb`.

## Common issues

- Build is tested on Ubuntu 16.04 with gcc 7.1.0, Fedora 28 with gcc 7.3.1, and MacOS 10.13.6 with Xcode 10 and clang-1000.10.25.5. If you experience compilation issues, please open an issue with the details of your OS and compiler versions.
- If you are running on a remote machine and experience display errors when initializing the simulator, ensure you do not have `DISPLAY` defined in your environment (run `unset DISPLAY` to undefine the variable)
- If your machine has a custom installation location for the nvidia OpenGL and EGL drivers, you may need to manually provide the `EGL_LIBRARY` path to cmake as follows.  Add `-DEGL_LIBRARY=/usr/lib/x86_64-linux-gnu/nvidia-opengl/libEGL.so` to the `build.sh` command line invoking cmake. When running any executable adjust the environment as follows: `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH} examples/example.py`.

### Code style

We use `clang-format` for linting and code style enforcement of c++ code.
Code style follows the [Google C++ guidelines](https://google.github.io/styleguide/cppguide.html).
Install `clang-format` through `brew install clang-format` on MacOS, or by downloading binaries or sources from http://releases.llvm.org/download.html for Ubuntu etc.
For vim integration add to your .vimrc file `map <C-K> :%!clang-format<cr>` and use Ctrl+K to format entire file.
Integration plugin for [vscode](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format).

We use `black` for linting python code.
Install `black` through `pip install black`.
We also use pre-commit hooks to ensure linting and style enforcement.
Install the pre-commit hooks with `pip install pre-commit && pre-commit install`.

### Development Tips

1. Install `ninja` (`sudo apt install ninja-build` on Linux, or `brew install ninja` on MacOS) for significantly faster incremental builds
1. Install `ccache` (`sudo apt install ccache` on Linux, or `brew install ccache` on MacOS) for significantly faster clean re-builds and builds with slightly different settings

## Acknowledgments
The Habitat project would not have been possible without the support and contributions of many individuals. We would like to thank Xinlei Chen, Georgia Gkioxari, Daniel Gordon, Leonidas Guibas, Saurabh Gupta, Or Litany, Marcus Rohrbach, Amanpreet Singh, Devendra Singh Chaplot, Yuandong Tian, and Yuxin Wu for many helpful conversations and guidance on the design and development of the Habitat platform.

## License

Habitat-Sim is MIT licensed. See the LICENSE file for details.

## References

1. [Habitat: A Platform for Embodied AI Research](https://arxiv.org/abs/1904.01201). Manolis Savva, Abhishek Kadian, Oleksandr Maksymets, Yili Zhao, Erik Wijmans, Bhavana Jain, Julian Straub, Jia Liu, Vladlen Koltun, Jitendra Malik, Devi Parikh, Dhruv Batra. Tech report, arXiv:1904.01201, 2019.
