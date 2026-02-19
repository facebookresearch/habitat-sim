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
- We have adopted squash-and-merge as the policy for incorporating PRs into the main branch.  We encourage more smaller/focused PRs rather than big PRs with many independent changes.  This also enables faster development by merging PRs into main quickly and reducing the need to rebase due to changes on main.
- While working on a PR, try to religiously keep your fork up-to-date with main by rebasing as necessary.  Note that the above recommendation for smaller and more frequent PRs reduces the burden of rebasing.
- We expect PR ready for final review only if Continuous Integration tests are passing.
- Recommended: after getting a PR through reviews/feedback and is merged into main, delete the branch to de-clutter noise.
- Reach out to us with questions or suggestions on our [Discussions forum](https://github.com/facebookresearch/habitat-lab/discussions).

## Contributor License Agreement ("CLA")
In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Meta's open source projects. Complete your CLA [here](https://code.facebook.com/cla).

By contributing to habitat-sim, you agree that your contributions will be licensed
under [the LICENSE file](https://github.com/facebookresearch/habitat-sim/blob/main/LICENSE).

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
  - Use `clang-format-13` for style enforcement and linting.
  Install `clang-format` through `brew install clang-format` on macOS. For other systems, `clang-format` can be installed via `conda install clangdev -c conda-forge` or by downloading binaries or sources from [releases.llvm.org/download](http://releases.llvm.org/download.html).
  For vim integration add to your .vimrc file `map <C-K> :%!clang-format<cr>` and use Ctrl+K to format entire file. Integration plugin for [vscode](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format).
- Python
  - We follow PEP8 and use [typing](https://docs.python.org/3/library/typing.html).
  - We use `black`, `isort`, and `ruff` for linting and code style of python code.
  Install them through `pip install -U black isort ruff`. They can then be run via `black .`, `isort .`, and `ruff check .`.

We also use pre-commit hooks to ensure linting and style enforcement. Install the pre-commit hooks with `pip install pre-commit && pre-commit install`.

## Documentation
- Our documentation style is based on Magnum / Corrade and uses [a similar build system](https://mcss.mosra.cz/documentation/doxygen/).
- The gfx_batch library is a good example of the documentation style.
- Documentation of PRs is highly encouraged!

## Development Tips

1. Install `ninja` (`sudo apt install ninja-build` on Linux, or `brew install ninja` on macOS) for significantly faster incremental builds
1. Install `ccache` (`sudo apt install ccache` on Linux, or `brew install ccache` on macOS) for significantly faster clean re-builds and builds with slightly different settings
1. Use editable installs (`pip install -e . --no-build-isolation`) for active development — scikit-build-core will automatically rebuild when you re-import after C++ changes
1. Build options are set via environment variables (e.g. `HABITAT_BUILD_GUI_VIEWERS=OFF` for headless, `HABITAT_WITH_CUDA=ON` for CUDA). Both GUI viewers and Bullet physics are enabled by default. See [BUILD_FROM_SOURCE.md](BUILD_FROM_SOURCE.md) for the full reference table.
