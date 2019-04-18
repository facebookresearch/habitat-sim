#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Adapted from: http://www.benjack.io/2017/06/12/python-cpp-tests.html
"""

import os
import os.path as osp
import sys
import platform
import subprocess
import builtins

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from setuptools.command.install import install


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


HEADLESS = False


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        root = osp.dirname(extdir)
        mode_file = osp.join(root, "bindings/mode.py")
        with open(mode_file, "r") as f:
            contents = [l.strip() for l in f.readlines() if len(l.strip()) > 0]

        contents[-1] = "use_dev_bindings = False"

        with open(mode_file, "w") as f:
            f.write("\n".join(contents))

        is_in_git = True
        try:
            subprocess.check_output(["git", "rev-parse", "--is-inside-work-tree"])
        except:
            is_in_git = False

        if is_in_git:
            subprocess.check_call(
                ["git", "submodule", "update", "--init", "--recursive"]
            )

        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
        ]

        cfg = "Debug" if self.debug else "RelWithDebInfo"
        build_args = ["--config", cfg]

        if platform.system() == "Windows":
            cmake_args += [
                "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}".format(cfg.upper(), extdir)
            ]
            if sys.maxsize > 2 ** 32:
                cmake_args += ["-A", "x64"]
            build_args += ["--", "/m"]
        else:
            cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]
            build_args += ["--", "-j4"]

        cmake_args += ["-DBUILD_GUI_VIEWERS={}".format("ON" if not HEADLESS else "OFF")]

        env = os.environ.copy()
        env["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get("CXXFLAGS", ""), self.distribution.get_version()
        )
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )
        if not HEADLESS:
            link_dst = osp.join(osp.dirname(self.build_temp), "viewer")
            if not osp.islink(link_dst):
                os.symlink(
                    osp.abspath(osp.join(self.build_temp, "utils/viewer/viewer")),
                    link_dst,
                )

        print()  # Add an empty line for cleaner output


class InstallCommand(install):
    user_options = install.user_options + [
        ("headless", None, "Build with headless and multi-gpu support")
    ]
    boolean_options = install.boolean_options + ["headless"]

    def initialize_options(self):
        install.initialize_options(self)
        self.headless = False

    def finalize_options(self):
        global HEADLESS
        install.finalize_options(self)

        HEADLESS = self.headless


requirements = ["numpy", "pillow", "numpy-quaternion", "attrs"]

builtins.__HSIM_SETUP__ = True
import habitat_sim

setup(
    name="habitat_sim",
    version=habitat_sim.__version__,
    author="FAIR A-STAR",
    description="A high performance simulator for training embodied agents",
    long_description="",
    packages=find_packages(),
    install_requires=requirements,
    # add extension module
    ext_modules=[CMakeExtension("habitat_sim._ext.habitat_sim_bindings", "src")],
    # add custom build_ext command
    cmdclass=dict(build_ext=CMakeBuild, install=InstallCommand),
    zip_safe=False,
)
