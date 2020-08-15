# Contributing to habitat-sim
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
- If you haven't already, complete [the Contributor License Agreement ("CLA")](https://code.facebook.com/cla).

## Contributor License Agreement ("CLA")
In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Facebook's open source projects. Complete your CLA [here](https://code.facebook.com/cla).

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
  - Use `clang-format`(clang-format-8) for style enforcement and linting. Install clang-format through `brew install clang-format` on MacOS, or using `apt-get install -y clang-format-8` as we do for [the testing environment setup](https://github.com/facebookresearch/habitat-sim/blob/master/.circleci/config.yml) or by downloading [binaries or sources](http://releases.llvm.org/download.html) for Ubuntu etc.
- Python
  - We follow PEP8 and use [typing](https://docs.python.org/3/library/typing.html).
  - Use `black` for style enforcement and linting. Install black through `pip install black`.

We also use pre-commit hooks to ensure linting and style enforcement. Install the pre-commit hooks with `pip install pre-commit && pre-commit install`.

## Documentation
- Our documentation style is based on Magnum / Corrade and uses [a similar build system](https://mcss.mosra.cz/documentation/doxygen/).
- A good example of the documentation style is in esp::gfx::DepthUnprojection (DepthUnprojection.h).
- Documentation of PRs is highly encouraged!

## Developer tips
- Install **ninja** (`sudo apt install ninja-build` on Linux, or `brew install ninja` on MacOS) for significantly faster incremental builds.
- Install **ccache** (`sudo apt install ccache` on Linux, or `brew install ccache` on MacOS) for significantly faster clean re-builds and builds with slightly different settings


## License
By contributing to habitat-sim, you agree that your contributions will be licensed
under [the LICENSE file](https://github.com/facebookresearch/habitat-sim/blob/master/LICENSE).
