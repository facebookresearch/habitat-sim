#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

#
# A script to capture the running environment information for debugging
# purposes.
# Please, run this script and provide it's output when reporting a bug.
#

from __future__ import absolute_import, division, print_function, unicode_literals

import locale
import os
import platform
import subprocess
import sys


def run_command(command: str) -> str:
    p = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
    )
    output, error = p.communicate()

    encoded = locale.getpreferredencoding()
    out = str(output.decode(encoded).encode("ascii"))
    return out.strip()


def get_gcc_version() -> str:
    return run_command("gcc --version | head -n 1")


def get_cmake_version() -> str:
    return run_command("cmake --version | head -n 1")


def get_nvidia_smi() -> str:
    return run_command("nvidia-smi | sed '/^ *$/,$ d'")


def get_pip_packages() -> str:
    return run_command(
        'bash -c "join -t= <(pip3 list --format=freeze | '
        "sort) <(awk -F== '{ print $1 }' requirements.txt | "
        "tr '>' ' ' | tr '=' ' ' | sort)\""
    )


def get_conda_packages() -> str:
    conda = os.environ.get("CONDA_EXE", "conda")
    return run_command(
        # f"join -t' ' <({conda} list | "
        # "sort) <(awk -F== '{ print $1 }' requirements.txt | "
        # "tr '>' ' ' | tr '=' ' ' | sort)"
        f"bash -c \"join -t' ' <({conda} list | "
        "sort) <(awk -F== '{ print $1 }' requirements.txt | "
        "tr '>' ' ' | tr '=' ' ' | sort)\""
    )


def main() -> None:
    print(
        f"ENVIRONMENT INFO:\n"
        f"Platform: {platform.platform()}\n"
        f"Machine: {platform.machine()}\n"
        f"Processor: {platform.processor()}\n"
        f"Libc version: {' '.join(platform.libc_ver())}\n"
        f"Mac version: {platform.mac_ver()[0]}\n"
        f"Python version: "
        f"{'.'.join(platform.python_version_tuple())}\n"
        f"Architecture: {' '.join(platform.architecture())}\n"
        f"Win version: {' '.join(platform.win32_ver())}\n"
        f"System OS: {platform.system()}\n"
        f"Release: {platform.release()}\n"
        f"Version: {platform.version()}\n"
        f"Operational System: {sys.platform}\n"
        f"GCC version: {get_gcc_version()}\n"
        f"CMAKE version: {get_cmake_version()}\n"
        f"NVIDIA-SMI: {get_nvidia_smi()}\n"
        f"Pip packages versions:\n{get_pip_packages()}\n"
        f"Conda packages versions:\n{get_conda_packages()}\n"
    )


if __name__ == "__main__":
    main()
