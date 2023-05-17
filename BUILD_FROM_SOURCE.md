## [Experimental] PIP install

- This is an automated way for building the necessary habitat binaries. For better support please skip to the Build from Source section.
- The build files are not cached and therefore this build method is slow and not recommended for active development.

```bash
   git clone --branch stable https://github.com/facebookresearch/habitat-sim.git
   cd habitat-sim
   pip install . -v
```

- You can also allow pip to compile a specific version of Habitat. First clone the repo, then `pip install .` in the current git root directory
  to start the compilation process. To quickly compile the latest main, run `pip install git+https://github.com/facebookresearch/habitat-sim`.

- Since pip out of tree by default, this process will copy quite a lot of data to your TMPDIR. You can change this location by modifying the TMPDIR env variable.
  It will also not cache previous builds effectively and therefore will be slow. For active development, building using `python setup.py install...` is recommended.

- Most compilation options can be accessed by either modifying the relevant ENV\_VARS (WITH\_BULLET, WITH\_CUDA, HEADLESS) etc or by passing the args through pip's `--global-option` and `--build-option` arguments.

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
   # We require python>=3.9 and cmake>=3.10
   conda create -n habitat python=3.9 cmake=3.14.0
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

   See this [configuration for a full list of dependencies](https://github.com/facebookresearch/habitat-sim/blob/main/.circleci/config.yml#L64) that our CI installs on a clean Ubuntu VM. If you run into build errors later, this is a good place to check if all dependencies are installed.

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

   With physics simulation via [Bullet Physics SDK](https://github.com/bulletphysics/bullet3/):
   To use Bullet, enable bullet physics build via:

   ```bash
   python setup.py install --bullet    # build habitat with bullet physics
   ```

   With audio sensor via [rlr-audio-propagation](https://github.com/facebookresearch/rlr-audio-propagation/):
   To use Audio sensors (Linux only), enable the audio flag via:

   ```bash
   python setup.py install --audio    # build habitat with audio sensor
   ```

   Note1: Build flags stack, *e.g.* to build in headless mode, with CUDA, and bullet, one would use `--headless --with-cuda --bullet`.

   Note2: some Linux distributions might require an additional `--user` flag to deal with permission issues.

   Note3: for active development in Habitat, you might find `./build.sh` instead of `python setup.py install` more useful.

   Note4: Audio sensor is only available on Linux.

1. [Only if using `build.sh`] For use with [Habitat Lab](https://github.com/facebookresearch/habitat-lab) and your own python code, add habitat-sim to your `PYTHONPATH`. For example modify your `.bashrc` (or `.bash_profile` in Mac OS X) file by adding the line:
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
