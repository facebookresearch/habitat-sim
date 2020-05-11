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
    bullet_modes = [True, False][1:]
    headless_modes = [True, False][0:1]
    cuda_vers = [None, "9.2", "10.0"][0:1]

    for py_ver, use_bullet, headless, cuda_ver in itertools.product(
        py_vers, bullet_modes, headless_modes, cuda_vers
    ):
        env = os.environ.copy()

        # including the commit hash
        repo = git.Repo(search_parent_directories=True)
        sha = repo.head.object.hexsha
        env["VERSION"] = habitat_sim.__version__ + "." + sha
        env["WITH_BULLET"] = "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        build_string = f"py{py_ver}_"
        if headless:
            build_string += "headless_"
            env["HEADLESS"] = "1"
            env["CONDA_HEADLESS_FEATURE"] = "- headless"

        if use_bullet:
            build_string += "bullet_"
            env["CONDA_BULLET"] = "- bullet"
            env["WITH_BULLET"] = "1"
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

        build_string += "linux"
        env["HSIM_BUILD_STRING"] = build_string

        call(
            build_cmd_template.format(PY_VER=py_ver, OUTPUT_FOLDER="hsim-linux"),
            env=env,
        )


if __name__ == "__main__":
    main()
