#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Adapted from: http://www.benjack.io/2017/06/12/python-cpp-tests.html
"""

import argparse
import builtins
import glob
import json
import os
import os.path as osp
import re
import shlex
import shutil
import subprocess
import sys
from distutils.util import strtobool
from distutils.version import StrictVersion

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

try:
    import cmake

    # If the cmake python package is installed, use that exe
    CMAKE_BIN_DIR = cmake.CMAKE_BIN_DIR
except ImportError:
    CMAKE_BIN_DIR = ""

sys.path.insert(0, osp.join(osp.dirname(__file__), "src_python"))

ARG_CACHE_BLACKLIST = {"force_cmake", "cache_args", "inplace"}


def str2bool(input_str: str) -> bool:
    return bool(strtobool(input_str.lower()))


def is_pip() -> bool:
    # This will end with python if driven with python setup.py or PEP517_BUILD_BACKEND will be set
    return (
        osp.basename(os.environ.get("_", "/pip/no")).startswith("pip")
        or os.environ.get("PEP517_BUILD_BACKEND") is not None
    )


# TODO refactor to the proper way to pass options to setup.py so pip can do so.
def build_parser():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--headless",
        dest="headless",
        default=str2bool(os.environ.get("HEADLESS", str(is_pip()))),
        action="store_true",
        help="""Build in headless mode.
Use "HEADLESS=True pip install ." to build in headless mode with pip""",
    )
    parser.add_argument(
        "--with-cuda",
        action="store_true",
        default=str2bool(os.environ.get("WITH_CUDA", "False")),
        dest="with_cuda",
        help="Build CUDA enabled features.  Requires CUDA to be installed",
    )
    parser.add_argument(
        "--bullet",
        "--with-bullet",
        dest="with_bullet",
        default=str2bool(os.environ.get("WITH_BULLET", str(is_pip()))),
        action="store_true",
        help="""Build with Bullet simulation engine. Default to True when pip installing.
 Default value is otherwise false or provided  WITH_BULLET=ON or WITH_BULLET_OFF when doing pip install.""",
    )
    parser.add_argument("--no-bullet", dest="with_bullet", action="store_false")
    parser.add_argument(
        "--vhacd",
        dest="with_vhacd",
        action="store_true",
        help="""Build with VHACD convex hull decomposition and voxelization engine.""",
    )
    parser.add_argument(
        "--cmake",
        "--force-cmake",
        dest="force_cmake",
        action="store_true",
        help="Forces cmake to be rerun.  This argument is not cached",
    )
    parser.add_argument(
        "--build-tests", dest="build_tests", action="store_true", help="Build tests"
    )
    parser.add_argument(
        "--build-datatool",
        dest="build_datatool",
        action="store_true",
        help="Build data tool",
    )
    parser.add_argument(
        "--cmake-args",
        type=str,
        default=os.environ.get("CMAKE_ARGS", ""),
        help="""Additional arguements to be passed to cmake.
Note that you will need to do `--cmake-args="..."` as `--cmake-args "..."`
will generally not be parsed correctly
You may need to use --force-cmake to ensure cmake is rerun with new args.
Use "CMAKE_ARGS="..." pip install ." to set cmake args with pip""",
    )
    parser.add_argument(
        "--no-update-submodules",
        dest="no_update_submodules",
        action="store_true",
        help="Don't update git submodules",
    )
    parser.add_argument(
        "--build-type",
        dest="build_type",
        default=None,
        help="CMake configuration to build with (Release, Debug, etc...)",
    )
    parser.add_argument(
        "--no-lto",
        dest="lto",
        default=None,
        action="store_false",
        help="Disables Link Time Optimization: faster compile times but worse performance.",
    )
    parser.add_argument(
        "--lto",
        dest="lto",
        action="store_true",
        help="Enables Link Time Optimization: better performance but longer compile time",
    )

    parser.add_argument(
        "--cache-args",
        dest="cache_args",
        action="store_true",
        help="""Caches the arguements sent to setup.py
        and reloads them on the next invocation.  This argument is not cached""",
    )

    parser.add_argument(
        "--skip-install-magnum",
        dest="skip_install_magnum",
        action="store_true",
        help="Don't install magnum.  "
        "This is nice for incrementally building for development but "
        "can cause install magnum bindings to fall out-of-sync",
    )

    parser.add_argument(
        "--build-basis-compressor",
        "--basis-compressor",
        dest="build_basis_compressor",
        action="store_true",
        help="Wether or not to build the basis compressor."
        "  Loading basis compressed meshes does NOT require this.",
    )

    parser.add_argument(
        "--audio",
        action="store_true",
        default=str2bool(os.environ.get("WITH_AUDIO", "False")),
        dest="with_audio",
        help="Build with audio features enabled.",
    )

    return parser


parseable_args = []
unparseable_args = []
for i, arg in enumerate(sys.argv):
    if arg == "--":
        unparseable_args = sys.argv[i:]
        break

    parseable_args.append(arg)


parser = build_parser()
args, filtered_args = parser.parse_known_args(args=parseable_args)

sys.argv = filtered_args + unparseable_args


def in_git():
    try:
        subprocess.check_output(["git", "rev-parse", "--is-inside-work-tree"])
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def has_ninja():
    try:
        subprocess.check_output(["ninja", "--version"])
        return True
    except (OSError, subprocess.SubprocessError):
        return False


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


# populated in CMakeBuild.build_extension()
_cmake_build_dir = None


class CMakeBuild(build_ext):
    def finalize_options(self):
        super().finalize_options()

        cacheable_params = [
            opt[0].replace("=", "").replace("-", "_") for opt in self.user_options
        ]

        args_cache_file = ".setuppy_args_cache.json"

        if not args.cache_args and osp.exists(args_cache_file):
            with open(args_cache_file, "r") as f:
                cached_args = json.load(f)

            for k, v in cached_args["args"].items():
                setattr(args, k, v)

            for k, v in cached_args["build_ext"].items():
                setattr(self, k, v)

        elif args.cache_args:
            cache = dict(
                args={
                    k: v for k, v in vars(args).items() if k not in ARG_CACHE_BLACKLIST
                },
                build_ext={
                    k: getattr(self, k)
                    for k in cacheable_params
                    if k not in ARG_CACHE_BLACKLIST
                },
            )
            with open(args_cache_file, "w") as f:
                json.dump(cache, f, indent=4, sort_keys=True)

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        # Save the CMake build directory -- that's where the generated setup.py
        # for magnum-bindings will appear which we need to run later
        global _cmake_build_dir
        _cmake_build_dir = self.build_temp

    def run(self):
        try:
            subprocess.check_output([osp.join(CMAKE_BIN_DIR, "cmake"), "--version"])
        except (OSError, subprocess.SubprocessError):
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Init & update all submodules if not already (the user might be pinned
        # on some particular commit or have working tree changes, don't destroy
        # those)
        if in_git() and not args.no_update_submodules:
            subprocess.check_call(
                ["git", "submodule", "update", "--init", "--recursive"]
            )

        cmake_args = [
            "-DBUILD_PYTHON_BINDINGS=ON",
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
            "-DCMAKE_EXPORT_COMPILE_COMMANDS={}".format("OFF" if is_pip() else "ON"),
            "-DREL_BUILD_RPATH={}".format("OFF" if self.inplace else "ON"),
        ]
        if args.lto is not None:
            cmake_args += [
                "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION={}".format(
                    "ON" if args.lto else "OFF"
                )
            ]
        cmake_args += shlex.split(args.cmake_args)

        build_type = args.build_type
        assert not (
            build_type is not None and self.debug
        ), f"Debug and Build-Type flags conflict: {self.debug}, {build_type}"
        if build_type is None:
            build_type = "Debug" if self.debug else "RelWithDebInfo"
        build_args = ["--config", build_type]

        cmake_args += ["-DCMAKE_BUILD_TYPE=" + build_type]

        build_args += ["--"]

        if has_ninja():
            cmake_args += ["-GNinja"]
        # Make it possible to *reduce* the number of jobs. Ninja requires a
        # number passed to -j (and builds on all cores by default), while make
        # doesn't require a number (but builds sequentially by default), so we
        # add the argument only when it's not ninja or the number of jobs is
        # specified.
        if not has_ninja() or self.parallel:
            build_args += ["-j{}".format(self.parallel) if self.parallel else "-j"]

        cmake_args += [
            "-DBUILD_GUI_VIEWERS={}".format("ON" if not args.headless else "OFF")
        ]

        if sys.platform not in ["darwin", "win32", "win64"]:
            cmake_args += [
                # So Magnum itself prefers EGL over GLX for windowless apps.
                # Makes sense only on platforms with EGL (Linux, BSD, ...).
                "-DTARGET_HEADLESS={}".format("ON" if args.headless else "OFF")
            ]
        # NOTE: BUILD_TEST is intentional as opposed to BUILD_TESTS which collides
        # with definition used by some of our dependencies
        cmake_args += ["-DBUILD_TEST={}".format("ON" if args.build_tests else "OFF")]
        cmake_args += [
            "-DBUILD_WITH_BULLET={}".format("ON" if args.with_bullet else "OFF")
        ]
        cmake_args += [
            "-DBUILD_WITH_VHACD={}".format("ON" if args.with_vhacd else "OFF")
        ]
        cmake_args += [
            "-DBUILD_DATATOOL={}".format("ON" if args.build_datatool else "OFF")
        ]
        cmake_args += ["-DBUILD_WITH_CUDA={}".format("ON" if args.with_cuda else "OFF")]
        cmake_args += [
            "-DBUILD_BASIS_COMPRESSOR={}".format(
                "ON" if args.build_basis_compressor else "OFF"
            )
        ]
        cmake_args += [
            "-DBUILD_WITH_AUDIO={}".format("ON" if args.with_audio else "OFF")
        ]

        env = os.environ.copy()
        env["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get("CXXFLAGS", ""), self.distribution.get_version()
        )

        if is_pip() or self.run_cmake(cmake_args):
            os.makedirs(self.build_temp, exist_ok=True)
            # Remove invalid cmakefiles if is is_pip()
            for cmake_cache_f in [
                "CMakeFiles",
                "CMakeCache.txt",
                "cmake_install.cmake",
            ]:
                cmake_cache_f = osp.join(self.build_temp, cmake_cache_f)
                if is_pip() and osp.exists(cmake_cache_f):
                    if osp.isdir(cmake_cache_f):
                        shutil.rmtree(cmake_cache_f)
                    else:
                        os.remove(cmake_cache_f)
            subprocess.check_call(
                [osp.join(CMAKE_BIN_DIR, "cmake")]
                + cmake_args
                + [osp.realpath(ext.sourcedir)],
                env=env,
                cwd=self.build_temp,
            )

        if not is_pip():
            self.create_compile_commands()

        subprocess.check_call(
            [osp.join(CMAKE_BIN_DIR, "cmake"), "--build", self.build_temp] + build_args
        )
        print()  # Add an empty line for cleaner output

        # The things following this don't work with pip
        if is_pip():
            return

        if not args.headless:
            link_dst = osp.join(self.build_temp, "viewer")
            if not osp.islink(link_dst):
                os.symlink(
                    osp.abspath(osp.join(self.build_temp, "utils/viewer/viewer")),
                    link_dst,
                )

    def run_cmake(self, cmake_args):
        if args.force_cmake:
            return True

        cache_parser = re.compile(r"(?P<K>\w+?)(:\w+?|)=(?P<V>.*?)$")

        cmake_cache = osp.join(self.build_temp, "CMakeCache.txt")
        if osp.exists(cmake_cache):
            with open(cmake_cache, "r") as f:
                cache_contents = f.readlines()

            for arg in cmake_args:
                if arg[0:2] == "-G":
                    continue

                k, v = arg.split("=", 1)
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

        command_files = [osp.join(self.build_temp, "compile_commands.json")]
        command_files += glob.glob("{}/*/compile_commands.json".format(self.build_temp))
        all_commands = [entry for f in command_files for entry in load(f)]

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


if __name__ == "__main__":
    assert StrictVersion(
        "{}.{}".format(sys.version_info[0], sys.version_info[1])
    ) >= StrictVersion("3.7"), "Must use python3.7 or newer"
    with open("./requirements.txt", "r") as f:
        requirements = [l.strip() for l in f.readlines() if len(l.strip()) > 0]

    builtins.__HSIM_SETUP__ = True
    import habitat_sim

    setup(
        name="habitat_sim",
        version=habitat_sim.__version__,
        author="FAIR A-STAR",
        description="A high performance simulator for training embodied agents",
        long_description="",
        packages=find_packages(where="src_python"),
        package_dir={"": "src_python"},
        install_requires=requirements,
        tests_require=["hypothesis", "pytest-benchmark", "pytest"],
        python_requires=">=3.7",
        # add extension module
        ext_modules=[CMakeExtension("habitat_sim._ext.habitat_sim_bindings", "src")],
        # add custom build_ext command
        cmdclass=dict(build_ext=CMakeBuild),
        zip_safe=False,
        include_package_data=True,
    )
    pymagnum_build_dir = osp.join(
        _cmake_build_dir, "deps", "magnum-bindings", "src", "python"
    )

    if (
        not args.skip_install_magnum
        and "sdist" not in sys.argv
        and os.path.exists(pymagnum_build_dir)
    ):
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", pymagnum_build_dir]
        )
    else:
        if not os.path.exists(pymagnum_build_dir) and "sdist" not in sys.argv:
            print(
                f"{pymagnum_build_dir} does not exist and therefore we cannot install magnum-bindings directly."
            )
        print(
            "Assuming magnum bindings are already installed (or we're inside pip and *\\_('-')_/*)"
        )
        print(
            f"Run '{sys.executable} -m pip install {pymagnum_build_dir}' if this assumption is incorrect"
        )
