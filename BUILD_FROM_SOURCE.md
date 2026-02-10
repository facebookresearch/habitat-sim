## PIP install

```bash
git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim
pip install . --no-build-isolation
```

- You can also allow pip to compile a specific version of Habitat. First clone the repo, then `pip install . --no-build-isolation` in the current git root directory
  to start the compilation process. To quickly compile the latest main, run `pip install git+https://github.com/facebookresearch/habitat-sim`.

- Since pip builds out of tree by default, this process will copy quite a lot of data to your TMPDIR. You can change this location by modifying the TMPDIR env variable.
  For active development, use an editable install: `pip install -e . --no-build-isolation`.

- Build options are controlled via environment variables. For example:
  ```bash
  HABITAT_BUILD_GUI_VIEWERS=OFF pip install . --no-build-isolation   # headless build
  HABITAT_WITH_BULLET=ON pip install . --no-build-isolation          # enable Bullet physics
  HABITAT_WITH_CUDA=ON pip install . --no-build-isolation            # enable CUDA
  ```

- By default, we build a headless version with bullet enabled.


## Build from Source

We highly recommend installing a [miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/distribution/#download-section) environment (note: python>=3.9 is required). Once you have Anaconda installed, here are the instructions.


1. Clone this github repository.

   ```bash
   # Checkout the latest stable release
   git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
   cd habitat-sim
   ```

   List of stable releases is [available here](https://github.com/facebookresearch/habitat-sim/releases). Main branch contains 'bleeding edge' code and under active development.

1. Install Dependencies

    Common

   ```bash
   # We require python>=3.12 and cmake>=3.22
   conda create -n habitat python=3.12 cmake=3.27
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

   See the github actions and workflow [configs for a full list of dependencies](https://github.com/facebookresearch/habitat-sim/blob/main/.github) that our CI installs on a clean Ubuntu VM. If you run into build errors later, this is a good place to check if all dependencies are installed.

1. Build Habitat-Sim

    Default build (for machines with a display attached)

   ```bash
   # Assuming we're still within habitat conda environment
   pip install . --no-build-isolation
   ```

    For headless systems (i.e. without an attached display, e.g. in a cluster) and multiple GPU systems

   ```bash
   HABITAT_BUILD_GUI_VIEWERS=OFF pip install . --no-build-isolation
   ```

    For systems with CUDA (to build CUDA features)

   ```bash
   HABITAT_WITH_CUDA=ON pip install . --no-build-isolation
   ```

   With physics simulation via [Bullet Physics SDK](https://github.com/bulletphysics/bullet3/):
   To use Bullet, enable bullet physics build via:

   ```bash
   HABITAT_WITH_BULLET=ON pip install . --no-build-isolation
   ```

   With audio sensor via [rlr-audio-propagation](https://github.com/facebookresearch/rlr-audio-propagation/):
   To use Audio sensors (Linux only), enable the audio flag via:

   ```bash
   HABITAT_WITH_AUDIO=ON pip install . --no-build-isolation
   ```

   Note1: Build options stack via environment variables, *e.g.* to build in headless mode, with CUDA, and bullet:
   ```bash
   HABITAT_BUILD_GUI_VIEWERS=OFF HABITAT_WITH_CUDA=ON HABITAT_WITH_BULLET=ON pip install . --no-build-isolation
   ```

   Note2: some Linux distributions might require an additional `--user` flag to deal with permission issues.

   Note3: for active development in Habitat, use an editable install (`pip install -e . --no-build-isolation`) or `./build.sh` which wraps the editable install with convenience flags.

   Note4: Audio sensor is only available on Linux.

   ### Build Configuration Reference

   | Environment Variable | Default | Description |
   |---|---|---|
   | `HABITAT_BUILD_GUI_VIEWERS` | `ON` | Build GUI viewer applications (set `OFF` for headless) |
   | `HABITAT_WITH_BULLET` | `OFF` | Enable Bullet physics simulation |
   | `HABITAT_WITH_CUDA` | `OFF` | Enable CUDA support |
   | `HABITAT_WITH_AUDIO` | `OFF` | Enable audio sensor (Linux only) |
   | `HABITAT_LTO` | `OFF` | Enable link-time optimization |
   | `HABITAT_BUILD_TESTS` | `OFF` | Build C++ tests |

   You can also pass CMake arguments directly via pip's `--config-settings`:
   ```bash
   pip install . --no-build-isolation --config-settings=cmake.build-type=Release
   ```

   ### C++ Viewer and Replayer Applications

   When `HABITAT_BUILD_GUI_VIEWERS` is `ON` (the default for non-headless builds),
   the C++ `viewer` and `replayer` applications are compiled alongside the Python
   bindings.

   - **After `pip install .`**: the executables are installed on `PATH` as `viewer`
     and `replayer`.
   - **After `pip install -e .`** (editable/development builds): convenience symlinks
     are created at `build/viewer` and `build/replayer` in the repo root.
   - **Standalone CMake** (no Python): you can build the C++ apps directly:
     ```bash
     cmake -B build -S src \
       -DBUILD_GUI_VIEWERS=ON \
       -DBUILD_PYTHON_BINDINGS=OFF \
       -DCMAKE_BUILD_TYPE=RelWithDebInfo
     cmake --build build -j$(nproc)
     # Binaries at: build/utils/viewer/viewer and build/utils/replayer/replayer
     ```

## Common build issues

- If your machine has a custom installation location for the nvidia OpenGL and EGL drivers, you may need to manually provide the `EGL_LIBRARY` path to cmake as follows.  Add `-DEGL_LIBRARY=/usr/lib/x86_64-linux-gnu/nvidia-opengl/libEGL.so` via `--config-settings=cmake.define.EGL_LIBRARY=/usr/lib/x86_64-linux-gnu/nvidia-opengl/libEGL.so`. When running any executable adjust the environment as follows: `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl:${LD_LIBRARY_PATH} examples/example.py`.

- By default, the build process uses all cores available on the system to parallelize. On some virtual machines, this might result in running out of memory. You can limit parallelism via:
   ```bash
   pip install . --no-build-isolation --config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=1
   ```

- Build is tested on Tested with Ubuntu 18.04 with gcc 7.4.0 and MacOS 10.13.6 with Xcode 10 and clang-1000.10.25.5. If you experience compilation issues, please open an issue with the details of your OS and compiler versions.

  We also have a dev slack channel, please follow this [link](https://join.slack.com/t/ai-habitat/shared_invite/enQtNjY1MzM1NDE4MTk2LTZhMzdmYWMwODZlNjg5MjZiZjExOTBjOTg5MmRiZTVhOWQyNzk0OTMyN2E1ZTEzZTNjMWM0MjBkN2VhMjQxMDI) to get added to the channel.
