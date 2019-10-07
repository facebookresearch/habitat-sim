#/bin/bash


build_args=()
if [ ${HEADLESS} == "1" ]; then
  build_args+=("--headless")
fi

if [ ${WITH_CUDA} = "1" ]; then
  build_args+=("--with-cuda")
fi

if [ ${WITH_BULLET} == "1" ]; then
  build_args+=("--with-bullet")
fi

${PYTHON} setup.py install "${build_args[@]}"



if [ -f "build/viewer" ]; then
  cp -v build/viewer ${PREFIX}/bin/habitat-viewer
fi

pushd ${PREFIX}

corrade_bindings=$(find . -name "*_corrade*so")
magnum_bindings=$(find . -name "*_magnum*so")
hsim_bindings=$(find . -name "*habitat_sim_bindings*so")
ext_folder=$(dirname ${corrade_bindings})/habitat_sim/_ext
echo ${ext_folder}

if [ $(uname) == "Darwin" ]; then
  install_name_tool -add_rpath @loader_path/habitat_sim/_ext ${corrade_bindings}
  install_name_tool -add_rpath @loader_path/habitat_sim/_ext ${magnum_bindings}
  install_name_tool -add_rpath @loader_path ${hsim_bindings}

  
  install_name_tool -add_rpath @loader_path/../${ext_folder} bin/habitat-viewer
  

  find ${ext_folder} -name "*Corrade*dylib" | xargs -I {} install_name_tool -add_rpath @loader_path {}


  pushd $(find . -name "corrade" -type d)
  find . -name "*so" | xargs -I {} install_name_tool -add_rpath @loader_path/../habitat_sim/_ext {}
  popd
fi


popd
