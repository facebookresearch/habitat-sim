# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import builtins
import itertools
import os
import os.path as osp
import platform
import shlex
import subprocess
import sys
import time

import git

builtins.__HSIM_SETUP__ = True
sys.path.insert(0, osp.join(osp.dirname(__file__), "..", "src_python"))

import habitat_sim

build_cmd_template = """
conda build \
  --python {PY_VER} \
  --channel conda-forge \
  --no-test \
  {ANACONDA_UPLOAD_MODE} \
  --output-folder {OUTPUT_FOLDER} \
  habitat-sim
"""


def call(cmd, env=None):
    cmd = shlex.split(cmd)
    subprocess.check_call(cmd, env=env)


def get_default_modes_and_vers():
    py_vers = ["3.9"]
    bullet_modes = [False, True]
    if platform.system() == "Darwin":
        return py_vers, bullet_modes, [False], [None]
    elif platform.system() == "Linux":  # noqa: SIM106
        return py_vers, bullet_modes, [True, False], [None]
    else:
        raise RuntimeError(f"Unknown system: {platform.system()}")


def get_platform_string() -> str:
    if platform.system() == "Darwin":
        return "macos"
    elif platform.system() == "Linux":  # noqa: SIM106
        return "linux"
    else:
        raise RuntimeError(f"Unknown system: {platform.system()}")


def get_headless_mode_for_test() -> bool:
    if platform.system() == "Darwin":
        return False
    elif platform.system() == "Linux":  # noqa: SIM106
        return True
    else:
        raise RuntimeError(f"Unknown system: {platform.system()}")


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ci_test",
        help="Test package conda build during continues integration test.",
        action="store_true",
    )
    parser.add_argument(
        "--nightly", help="Make conda nightly build.", action="store_true"
    )
    parser.add_argument(
        "--conda_upload",
        help="Upload conda binaries as package to authenticated Anaconda cloud account.",
        action="store_true",
    )

    return parser


def main():
    args = build_parser().parse_args()
    py_vers, bullet_modes, headless_modes, cuda_vers = get_default_modes_and_vers()

    # For CI test only one package build for test speed interest
    if args.ci_test:
        bullet_modes = [True]
        headless_modes = [get_headless_mode_for_test()]
        py_vers = ["3.9"]

    for py_ver, use_bullet, headless, cuda_ver in itertools.product(
        py_vers, bullet_modes, headless_modes, cuda_vers
    ):
        env = os.environ.copy()

        env["VERSION"] = habitat_sim.__version__
        # including a timestamp in anticipation of nightly builds
        if args.nightly:
            env["VERSION"] = env["VERSION"] + time.strftime(".%Y.%m.%d")
        env["WITH_BULLET"] = "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["LTO"] = "0" if args.ci_test else "1"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        build_string = f"py{py_ver}_"
        if headless:
            build_string += "headless_"
            env["HEADLESS"] = "1"
            env["HABITAT_HEADLESS_VARIANT"] = "headless"

        if use_bullet:
            build_string += "bullet_"
            env["WITH_BULLET"] = "1"
            env["HABITAT_BULLET_VARIANT"] = "bullet"
        else:
            env["CONDA_BULLET"] = ""

        if cuda_ver is not None:
            build_string += f"cuda{cuda_ver}_"
            env["WITH_CUDA"] = "1"
            env["CUDA_VER"] = cuda_ver
            if cuda_ver == "10.0":
                env[
                    "CONDA_CUDATOOLKIT_CONSTRAINT"
                ] = "- cudatoolkit >=10.0,<10.1 # [not osx]"
            elif cuda_ver == "9.2":
                env[
                    "CONDA_CUDATOOLKIT_CONSTRAINT"
                ] = "- cudatoolkit >=9.2,<9.3 # [not osx]"

        build_string += f"{get_platform_string()}"

        # including the commit hash in conda build string
        repo = git.Repo(search_parent_directories=True)
        sha = repo.head.object.hexsha
        build_string += "_" + sha

        env["HSIM_BUILD_STRING"] = build_string

        call(
            build_cmd_template.format(
                PY_VER=py_ver,
                OUTPUT_FOLDER=f"hsim-{get_platform_string()}",
                ANACONDA_UPLOAD_MODE=""
                if args.conda_upload
                else "--no-anaconda-upload",
            ),
            env=env,
        )


if __name__ == "__main__":
    main()
