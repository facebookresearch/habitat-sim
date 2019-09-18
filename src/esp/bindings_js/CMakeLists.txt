add_executable(hsim_bindings bindings_js.cpp)

target_link_libraries(hsim_bindings
  PUBLIC
    agent
    assets
    scene
    core
    gfx
    nav
    sensor
    sim
)

set_target_properties(hsim_bindings PROPERTIES LINK_FLAGS "--bind")

# copy JS/HTML/CSS resources for WebGL build
set(resources
<<<<<<< HEAD
    bindings.html
    bindings.css
    navigate.js
    runtime.js
    simenv_embind.js
    topdown.js
=======
>>>>>>> Update CMakeLists
    ${MAGNUM_WINDOWLESSEMSCRIPTENAPPLICATION_JS}
    ${MAGNUM_WEBAPPLICATION_CSS}
)

foreach(resource ${resources})
  get_filename_component(filename ${resource} NAME)
  get_filename_component(path ${resource} ABSOLUTE)
  add_custom_command(
      TARGET hsim_bindings POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E create_symlink
          ${path} ${CMAKE_CURRENT_BINARY_DIR}/${filename})
endforeach()

set(build_command npm run build)
message(STATUS "Building JS files now")
execute_process(COMMAND ${build_command}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
