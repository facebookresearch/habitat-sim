import argparse
import builtins
import itertools
import os
import os.path as osp
import shlex
import subprocess
import sys
import time

import git

builtins.__HSIM_SETUP__ = True
sys.path.append("../")

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

    if args.ci_test:
        bullet_modes = [True]
        py_vers = ["3.6"]
    else:

        py_vers = ["3.6", "3.7", "3.8"]
        bullet_modes = [False, True]

    for py_ver, use_bullet in itertools.product(py_vers, bullet_modes):
        env = os.environ.copy()

        # including a timestamp in anticipation of nightly builds
        env["VERSION"] = habitat_sim.__version__
        if args.nightly:
            env["VERSION"] = env["VERSION"] + time.strftime(".%Y.%m.%d")
        env["WITH_BULLET"] = "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["LTO"] = "0" if args.ci_test else "1"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        build_string = f"py{py_ver}_"
        if use_bullet:
            build_string += "bullet_"
            env["WITH_BULLET"] = "1"
            env["CONDA_BULLET_FEATURE"] = "- withbullet"

        build_string += "osx"

        # including the commit hash in conda build string
        repo = git.Repo(search_parent_directories=True)
        sha = repo.head.object.hexsha
        build_string += "_" + sha

        env["HSIM_BUILD_STRING"] = build_string

        call(
            build_cmd_template.format(
                PY_VER=py_ver,
                OUTPUT_FOLDER="hsim-macos",
                ANACONDA_UPLOAD_MODE=""
                if args.conda_upload
                else "--no-anaconda-upload",
            ),
            env=env,
        )


if __name__ == "__main__":
    main()
