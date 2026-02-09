### Building for macOS

Install conda packages as described here `conda-build/common/install_conda.sh` and run `conda create --name py312 python=3.12 -y; conda activate py312`.
Running `python matrix_builder.py` with Python >= 3.9 will start the build process by setting all the environment variables and making a call to conda build. Make sure the `meta.yaml` file in `conda-build/habitat-sim/` is configured correctly according to https://docs.conda.io/projects/conda-build/en/latest/resources/define-metadata.html

The build system uses **scikit-build-core** (configured in `pyproject.toml`). The conda build script (`build.sh`) passes build options via environment variables:

| Environment Variable          | Effect                         |
|-------------------------------|--------------------------------|
| `HEADLESS=1`                  | Disable GUI viewers (EGL only) |
| `WITH_BULLET=1`              | Enable Bullet physics          |
| `WITH_CUDA=1`                | Enable CUDA support            |
| `LTO=1`                      | Enable Link Time Optimization  |

These are mapped to `HABITAT_*` env vars internally by `build.sh`, which are then read by scikit-build-core's `[tool.scikit-build.cmake.define]` section in `pyproject.toml`.

Once the package is built, make sure you're logged in to anaconda cloud and then run `anaconda upload <path to the tarball file that conda build created>`. For example `anaconda upload hsim-macos/osx-64/habitat-sim-0.3.3-py3.12_osx.tar.bz2`. This will upload the package to anaconda cloud for everyone to download.

To then download the package, run `conda install -c aihabitat -c conda-forge habitat-sim`.


### Building for Linux

The process is almost the same for linux; there is a corresponding python script for starting things off, however we use a docker container to do the builds. There is a dockerfile in this directory that you can use to create a container.

```
docker build -t hsim_condabuild_dcontainer -f Dockerfile .
```

That will create your docker container. Now run

```
docker run -it --ipc=host --rm -v $(pwd)/../:/remote hsim_condabuild_dcontainer bash
```

From there you will have a shell within your linux container. Now, navigate to `cd /remote/conda-build` where habitat-sim has been mounted. Create a conda environment within the linux container with Python >= 3.9: `conda create --name py312 python=3.12; conda activate py312`. And then run `python matrix_builder.py`, which will kick off the build process. After this has finished, upload it to anaconda cloud in the same way described in the macOS section.

To download the package, run `conda install -c aihabitat -c conda-forge habitat-sim headless`.

Our linux conda builds currently support `{head / headless} x {with bullet / without bullet}` binaries. In the command above, we are telling conda to use a feature called `headless`.


### How the build works

1. `matrix_builder.py` reads the version from `pyproject.toml` and iterates over the build matrix (Python versions × bullet × headless × CUDA).
2. For each combination, it sets environment variables and invokes `conda build`.
3. `conda build` runs `build.sh`, which maps the conda variant variables (`HEADLESS`, `WITH_BULLET`, etc.) to `HABITAT_*` env vars and runs `pip install . --no-build-isolation`.
4. **scikit-build-core** handles the CMake configuration, build, and install automatically.
5. Magnum/Corrade Python bindings are installed via CMake `install()` targets (no separate pip install step needed).
6. `build.sh` then applies RPATH fixups for conda relocatability.


### Notes

* If building from your normal development clone of the repo, make sure to remove your build folder, i.e. `rm -r ../build`.  The builder will copy that folder and cmake will error out otherwise.

* You don't need to create a fresh conda env for building.  Just installing `conda-build>=3.18.9` in any existing conda env (with Python >= 3.9) is sufficient.

* You can upload all the binaries with one command by running `find . -name "*.tar.bz2" | xargs -I {} anaconda upload {}` in this folder.
