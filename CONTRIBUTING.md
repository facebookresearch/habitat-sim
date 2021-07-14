# Contributing to Habitat-Sim
We want to make contributing to this project as easy and transparent as
possible.

## Developer Workflow
- Active development should happen on your fork of the repositories.
- Name your PR in a way that unambiguously identifies the feature or fix.
- Follow the contribution guide to ensure your code is conformant to the conventions and style.
- Try to make small, logically independent, self-documenting commits (and reflect this in the commit messages by providing brief rationale/change summary).
- We encourage creating draft PRs to gather early feedback.
- Request reviews from at least one Habitat core team member (if the scope of changes necessitates, request from two or more reviewers).
- We have adopted squash-and-merge as the policy for incorporating PRs into the master branch.  We encourage more smaller/focused PRs rather than big PRs with many independent changes.  This also enables faster development by merging PRs into master quickly and reducing the need to rebase due to changes on master.
- While working on a PR, try to religiously keep your fork up-to-date with master by rebasing as necessary.  Note that the above recommendation for smaller and more frequent PRs reduces the burden of rebasing.
- We expect PR ready for final review only if Continuous Integration tests are passing.
- Recommended: after getting a PR through reviews/feedback and is merged into master, delete the branch to de-clutter noise.
- Reach out to us with questions or suggestions on our Slack channel.

## Contributor License Agreement ("CLA")
In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Facebook's open source projects. Complete your CLA [here](https://code.facebook.com/cla).

By contributing to habitat-sim, you agree that your contributions will be licensed
under [the LICENSE file](https://github.com/facebookresearch/habitat-sim/blob/master/LICENSE).

## Versioning / release workflow
We use [semantic versioning](https://semver.org/). To prepare a release:
1. Update version numbers.
2. Update the change log.
3. Make sure all tests are passing.
4. Create a release tag with change log summary using the github release interface (release tag should follow semantic versioning as described above)

Stable versions are regularly assigned by Habitat core team after rigorous testing.

## Issues
We use [GitHub issues](https://github.com/facebookresearch/habitat-sim/issues) to track public bugs. Please ensure your description is
clear and has sufficient instructions to be able to reproduce the issue.


## Coding Style

- C++
  - In general, we follow [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) and [Google C++ guidelines](https://google.github.io/styleguide/cppguide.html)
  - Use `clang-format-12` for style enforcement and linting. 
  Install `clang-format-12` through `brew install clang-format` on macOS. For other systems, `clang-format-12` can be installed via `conda install clangdev -c conda-forge` or by downloading binaries or sources from [releases.llvm.org/download](http://releases.llvm.org/download.html). 
  For vim integration add to your .vimrc file `map <C-K> :%!clang-format<cr>` and use Ctrl+K to format entire file. Integration plugin for [vscode](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)..
- Python
  - We follow PEP8 and use [typing](https://docs.python.org/3/library/typing.html).
  - We use `black` and `isort` for linting and code style of python code. 
  Install `black` and `isort` through `pip install -U black isort`. They can then be ran via `black .` and `isort`.
- JS
  - We use `eslint` with `prettier` plugin for linting, formatting and code style of JS code.
  Install these dependencies through `npm install`. Then, for fixing linting/formatting errors run `npm run lint-fix`. Make sure you have a node version > 8 for this.

We also use pre-commit hooks to ensure linting and style enforcement. Install the pre-commit hooks with `pip install pre-commit && pre-commit install`.

## Documentation
- Our documentation style is based on Magnum / Corrade and uses [a similar build system](https://mcss.mosra.cz/documentation/doxygen/).
- A good example of the documentation style is in esp::gfx::DepthUnprojection (DepthUnprojection.h).
- Documentation of PRs is highly encouraged!

## Developer tips
- Install **ninja** (`sudo apt install ninja-build` on Linux, or `brew install ninja` on MacOS) for significantly faster incremental builds.
- Install **ccache** (`sudo apt install ccache` on Linux, or `brew install ccache` on MacOS) for significantly faster clean re-builds and builds with slightly different settings

## Development Tips

1. Install `ninja` (`sudo apt install ninja-build` on Linux, or `brew install ninja` on macOS) for significantly faster incremental builds
1. Install `ccache` (`sudo apt install ccache` on Linux, or `brew install ccache` on macOS) for significantly faster clean re-builds and builds with slightly different settings
1. You can skip reinstalling magnum every time by adding the argument of `--skip-install-magnum` to either `build.sh` or `setup.py`.  Note that you will still need to install magnum bindings once.
1. Arguments to `build.sh` and `setup.py` can be cached between subsequent invocations with the flag `--cache-args` on the _first_ invocation.

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
