# Contributing to habitat-sim
We want to make contributing to this project as easy and transparent as
possible.

## Pull Requests
We actively welcome your pull requests.

1. Fork the repo and create your branch from `master`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. If you haven't already, complete the Contributor License Agreement ("CLA").

## Contributor License Agreement ("CLA")
In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Facebook's open source projects.

Complete your CLA here: <https://code.facebook.com/cla>

## Issues
We use GitHub issues to track public bugs. Please ensure your description is
clear and has sufficient instructions to be able to reproduce the issue.

## Test
Please make sure that C++ unit tests are passing and run the example script:
```
python ./example.py
```

## Check typing
We use mypy to check Python typing and guard API consistency, please make sure next command doesn't complain prior to submission:
```
mypy . --ignore-missing-imports
```

## Coding Style
We use `clang-format` for linting and code style enforcement of c++ code.
Code style follows the [Google C++ guidelines](https://google.github.io/styleguide/cppguide.html).
Install `clang-format` through `brew install clang-format` on MacOS, or by downloading binaries or sources from http://releases.llvm.org/download.html for Ubuntu etc.
For vim integration add to your .vimrc file `map <C-K> :%!clang-format<cr>` and use Ctrl+K to format entire file.
Integration plugin for [vscode](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format).

We use `black` for linting and code of python code.
Install `black` through `pip install black`

We also use pre-commit hooks to ensure linting and style enforcement.  Install the pre-commit hooks
with `pip install pre-commit && pre-commit install`.

## License
By contributing to habitat-sim, you agree that your contributions will be licensed
under the LICENSE file in the root directory of this source tree.
