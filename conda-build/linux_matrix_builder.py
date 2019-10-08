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
    bullet_modes = [True, False][1:]
    headless_modes = [True, False][1:]

    for py_ver, use_bullet, headless in itertools.product(
        py_vers, bullet_modes, headless_modes
    ):
        env = os.environ.copy()
        env["WITH_BULLET"] = "0"
        env["WITH_CUDA"] = "0"
        env["HEADLESS"] = "0"
        env["HSIM_SOURCE_PATH"] = osp.abspath(osp.join(osp.dirname(__file__), ".."))

        output_folder = f"py{py_ver}_"
        if headless:
            output_folder += "headless_"
            env["HEADLESS"] = "1"
            env["CONDA_HEADLESS_FEATURE"] = "- headless"

        if use_bullet:
            output_folder += "bullet_"
            env["CONDA_BULLET"] = "- bullet"
            env["WITH_BULLET"] = "1"

        output_folder += "linux"

        call(
            build_cmd_template.format(PY_VER=py_ver, OUTPUT_FOLDER=output_folder),
            env=env,
        )


if __name__ == "__main__":
    main()
