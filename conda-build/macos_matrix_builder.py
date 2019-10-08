import subprocess
import shlex
import argparse
import os.path as osp
import itertools
import os

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
    bullet_modes = [True, False][0:1]

    for py_ver, use_bullet in itertools.product(py_vers, bullet_modes):
        env = os.environ.copy()
        env["WITH_BULLET"] = "1" if use_bullet else "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        output_folder = f"py{py_ver}_"
        if use_bullet:
            output_folder += "bullet_"
            env["CONDA_BULLET"] = "- bullet"
            env["CONDA_BULLET_FEATURE"] = "- withbullet"
        else:
            env["CONDA_BULLET"] = ""
            env["CONDA_BULLET_FEATURE"] = ""

        output_folder += "osx"
        env["HSIM_BUILD_STRING"] = output_folder

        call(
            build_cmd_template.format(PY_VER=py_ver, OUTPUT_FOLDER=output_folder),
            env=env,
        )


if __name__ == "__main__":
    main()
