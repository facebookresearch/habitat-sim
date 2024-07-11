#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

build_args=(--skip-install-magnum)
if [ "${LTO}" == "1" ]; then
  build_args+=("--lto")
fi
if [ "${HEADLESS}" == "1" ]; then
  build_args+=("--headless")
fi

if [ "${WITH_CUDA}" == "1" ]; then
  build_args+=("--with-cuda")
  export CUDA_HOME=/public/apps/cuda/${CUDA_VER}
  export PATH=/public/apps/cuda/${CUDA_VER}/bin:${PATH}
  build_args+=(--cmake-args="-DCUDA_TOOLKIT_ROOT_DIR=${CUDA_HOME}")
fi

if [ "${WITH_BULLET}" == "1" ]; then
  build_args+=("--with-bullet")
fi

if [ "$(uname)" == "Linux" ]; then
  cp -r /usr/include/EGL "${PREFIX}"/include/.
  cp -r /usr/include/X11 "${PREFIX}"/include/.
  export CMAKE_PREFIX_PATH=${PREFIX}:${CMAKE_PREFIX_PATH}
fi

${PYTHON} setup.py install "${build_args[@]}"
${PYTHON} -m pip install build/deps/magnum-bindings/src/python

if [ -f "build/viewer" ]; then
  cp -v build/viewer "${PREFIX}"/bin/habitat-viewer
fi



pushd "${PREFIX}" || exit

corrade_bindings=$(find . -name "*_corrade*so")
echo "${corrade_bindings}"
magnum_bindings=$(find . -name "*_magnum*so")
echo "${magnum_bindings}"
hsim_bindings=$(find . -name "*habitat_sim_bindings*so")
echo "${hsim_bindings}"
ext_folder=$(dirname "${corrade_bindings}")/habitat_sim/_ext

if [ "$(uname)" == "Darwin" ]; then
  install_name_tool -add_rpath @loader_path/habitat_sim/_ext "${corrade_bindings}"
  install_name_tool -add_rpath @loader_path/habitat_sim/_ext "${magnum_bindings}"
  #install_name_tool -add_rpath @loader_path "${hsim_bindings}"


  install_name_tool -add_rpath @loader_path/../"${ext_folder}" bin/habitat-viewer


  #find "$(dirname "${hsim_bindings}")" -name "*Corrade*dylib" -print0 | xargs -I {} install_name_tool -add_rpath @loader_path {}


  pushd "$(find . -name "corrade" -type d)" || exit
    find . -name "*so" -print0 | xargs -I {} install_name_tool -add_rpath @loader_path/../habitat_sim/_ext {}
  popd || exit
elif [ "$(uname)" == "Linux" ]; then
  # Adding rpath for everything to have both habitat_sim/_ext and the conda env's lib dir
  # All this is done relatively to the *.so's folder to make it relocatable
  patchelf --set-rpath "\$ORIGIN/habitat_sim/_ext:\$ORIGIN/../.." --force-rpath "${corrade_bindings}"
  patchelf --set-rpath "\$ORIGIN/habitat_sim/_ext:\$ORIGIN/../.." --force-rpath "${magnum_bindings}"

  patchelf --set-rpath "\$ORIGIN:\$ORIGIN/../../../.." --force-rpath "${hsim_bindings}"

  if [ -f 'bin/habitat-viewer' ]; then
    patchelf --set-rpath "\$ORIGIN/../${ext_folder}:\$ORIGIN/../lib" --force-rpath bin/habitat-viewer
  fi

  find "$(dirname "${hsim_bindings}")" -name "*Corrade*so" -print0 | xargs -I {} patchelf --set-rpath "\$ORIGIN:\$ORIGIN/../../../.." --force-rpath {}

  pushd "$(dirname "${corrade_bindings}")/corrade" || exit
    find . -name "*so" -print0 | xargs -I {} patchelf --set-rpath "\$ORIGIN/../habitat_sim/_ext:\$ORIGIN/../../.." --force-rpath {}
  popd || exit
fi


popd || exit
