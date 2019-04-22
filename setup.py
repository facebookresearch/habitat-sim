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
import subprocess
import builtins
import re
import glob
import json

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

HEADLESS = False
FORCE_CMAKE = False
BUILD_TESTS = False
cache_parser = re.compile(r"(?P<K>\w+?)(:\w+?|)=(?P<V>.*?)$")


def in_git():
    try:
        subprocess.check_output(["git", "rev-parse", "--is-inside-work-tree"])
        return True
    except:
        return False


def has_ninja():
    try:
        subprocess.check_output(["ninja", "--version"])
        return True
    except:
        return False


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        is_in_git = in_git()

        if is_in_git:
            subprocess.check_call(
                ["git", "submodule", "update", "--init", "--recursive"]
            )

        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        ]

        cfg = "Debug" if self.debug else "RelWithDebInfo"
        build_args = ["--config", cfg]

        cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]
        build_args += ["--"]

        if has_ninja():
            cmake_args += ["-GNinja"]
        else:
            build_args += ["-j"]

        cmake_args += ["-DBUILD_GUI_VIEWERS={}".format("ON" if not HEADLESS else "OFF")]
        cmake_args += ["-DBUILD_TESTS={}".format("ON" if BUILD_TESTS else "OFF")]

        env = os.environ.copy()
        env["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get("CXXFLAGS", ""), self.distribution.get_version()
        )

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        if self.run_cmake(cmake_args):
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

        if not osp.islink(osp.join(osp.dirname(self.build_temp), "utils")):
            os.symlink(
                osp.abspath(osp.join(self.build_temp, "utils")),
                osp.join(osp.dirname(self.build_temp), "utils"),
            )

        self.create_compile_commands()
        print()  # Add an empty line for cleaner output

    def run_cmake(self, cmake_args):
        if FORCE_CMAKE:
            return True

        cmake_cache = osp.join(self.build_temp, "CMakeCache.txt")
        if osp.exists(cmake_cache):
            with open(cmake_cache, "r") as f:
                cache_contents = f.readlines()

            for arg in cmake_args:
                if arg[0:2] == "-G":
                    continue

                k, v = arg.split("=")
                # Strip +D
                k = k[2:]
                for l in cache_contents:

                    match = cache_parser.match(l)
                    if match is None:
                        continue

                    if match.group("K") == k and match.group("V") != v:
                        return True

            return False

        return True

    def create_compile_commands(self):
        def load(filename):
            with open(filename) as f:
                return json.load(f)

        commands = glob.glob("build/*/compile_commands.json")
        all_commands = [entry for f in commands for entry in load(f)]

        # cquery does not like c++ compiles that start with gcc.
        # It forgets to include the c++ header directories.
        # We can work around this by replacing the gcc calls that python
        # setup.py generates with g++ calls instead
        for command in all_commands:
            if command["command"].startswith("gcc "):
                command["command"] = "g++ " + command["command"][4:]

        new_contents = json.dumps(all_commands, indent=2)
        contents = ""
        if os.path.exists("compile_commands.json"):
            with open("compile_commands.json", "r") as f:
                contents = f.read()
        if contents != new_contents:
            with open("compile_commands.json", "w") as f:
                f.write(new_contents)


filtered_args = []
for i, arg in enumerate(sys.argv):
    if arg == "--headless":
        HEADLESS = True
        continue

    if arg == "--force-cmake" or arg == "--cmake":
        FORCE_CMAKE = True
        continue

    if arg == "--build-tests":
        BUILD_TESTS = True
        continue

    if arg == "--":
        filtered_args += sys.argv[i:]
        break

    filtered_args.append(arg)

sys.argv = filtered_args


if __name__ == "__main__":
    if os.environ.get("HEADLESS", "").lower() == "true":
        HEADLESS = True

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
        cmdclass=dict(build_ext=CMakeBuild),
        zip_safe=False,
    )
