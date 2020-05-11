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
  --no-anaconda-upload \
  --output-folder {OUTPUT_FOLDER} \
  habitat-sim
"""


def call(cmd, env=None):
    cmd = shlex.split(cmd)
    subprocess.check_call(cmd, env=env)


def build_parser():
    parser = argparse.ArgumentParser()

    return parser


def main():
    args = build_parser().parse_args()
    py_vers = ["3.6"]
    bullet_modes = [True, False]

    for py_ver, use_bullet in itertools.product(py_vers, bullet_modes):
        env = os.environ.copy()
        # including the commit hash
        repo = git.Repo(search_parent_directories=True)
        sha = repo.head.object.hexsha
        env["VERSION"] = habitat_sim.__version__ + "." + sha
        env["WITH_BULLET"] = "1" if use_bullet else "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        build_string = f"py{py_ver}_"
        if use_bullet:
            build_string += "bullet_"
            env["CONDA_BULLET"] = "- bullet"
            env["CONDA_BULLET_FEATURE"] = "- withbullet"
        else:
            env["CONDA_BULLET"] = ""
            env["CONDA_BULLET_FEATURE"] = ""

        build_string += "osx"
        env["HSIM_BUILD_STRING"] = build_string

        call(
            build_cmd_template.format(PY_VER=py_ver, OUTPUT_FOLDER="hsim-macos"),
            env=env,
        )


if __name__ == "__main__":
    main()
