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

# copy JS/HTML/CSS scaffolding for WebGL build
install(FILES
  bindings.html
  navigate.js
  simenv_embind.js
  ${MAGNUM_WINDOWLESSEMSCRIPTENAPPLICATION_JS}
  ${MAGNUM_WEBAPPLICATION_CSS}
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
