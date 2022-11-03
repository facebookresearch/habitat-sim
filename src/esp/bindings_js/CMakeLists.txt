# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

add_executable(hsim_bindings bindings_js.cpp)

target_link_libraries(
  hsim_bindings
  PUBLIC agent
         assets
         scene
         core
         gfx
         metadata
         nav
         sensor
         sim
)

set_target_properties(
  hsim_bindings
  PROPERTIES
    LINK_FLAGS "--bind"
    # Override the Magnum-set default which is in $<CONFIG>/bin or $<CONFIG>/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

if(BUILD_WEB_APPS)
  message("Building demo web apps")
  # Create symlinks for specific files as well as the entire data directory.
  set(
    resources
    ${SCENE_DATASETS}/habitat-test-scenes/skokloster-castle.glb
    ${SCENE_DATASETS}/habitat-test-scenes/skokloster-castle.navmesh
    ${SCENE_DATASETS}/habitat-test-scenes/van-gogh-room.glb
    ${SCENE_DATASETS}/habitat-test-scenes/van-gogh-room.navmesh
    ${MAGNUM_WINDOWLESSEMSCRIPTENAPPLICATION_JS}
    ${MAGNUM_WEBAPPLICATION_CSS}
    ${DATA_DIR}
  )

  foreach(resource ${resources})
    get_filename_component(filename ${resource} NAME)
    get_filename_component(path ${resource} ABSOLUTE)
    add_custom_command(
      TARGET hsim_bindings POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E create_symlink ${path}
              ${CMAKE_CURRENT_BINARY_DIR}/${filename}
    )
  endforeach()

  set(package_files ${PROJECT_SOURCE_DIR}/../package.json)

  add_custom_command(
    OUTPUT empty.stamp
    COMMAND npm install
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/empty.stamp
    DEPENDS ${package_files}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Trying npm install"
  )

  set(
    bundle_files
    bindings.html
    test_page.html
    viewer.html
    bindings.css
    viewer.css
    webpack.config.js
    index.js
    modules/object_sensor.js
    modules/navigate.js
    modules/simenv_embind.js
    modules/topdown.js
    modules/web_demo.js
    modules/viewer_demo.js
    modules/defaults.js
    modules/test_page.js
    modules/utils.js
    modules/vr_utils.js
  )

  add_custom_command(
    OUTPUT bundle.js
    COMMAND npm run lint-fix
    COMMAND npm run build -- --build_dir ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS hsim_bindings ${bundle_files} empty.stamp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Building JS files now"
  )

  add_custom_target(hsim_bundle ALL DEPENDS bundle.js)

  add_dependencies(hsim_bundle hsim_bindings)
endif()
