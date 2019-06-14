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
)

set_target_properties(hsim_bindings PROPERTIES LINK_OPTIONS "--bind")

# copy JS/HTML/CSS scaffolding for WebGL build
install(FILES
  bindings.html
  ${MAGNUM_WINDOWLESSEMSCRIPTENAPPLICATION_JS}
  ${MAGNUM_WEBAPPLICATION_CSS}
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
