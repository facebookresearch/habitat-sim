set(viewer_SOURCES viewer.cpp)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
add_executable(viewer ${viewer_SOURCES})

target_link_libraries(viewer
  PRIVATE
    assets
    gfx
    nav
    scene
)

# copy JS/HTML/CSS scaffolding for WebGL build
if(BUILD_WEBGL)
  install(TARGETS viewer DESTINATION ${CMAKE_INSTALL_PREFIX})
  install(FILES
      viewer.html
      ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
      ${MAGNUM_WEBAPPLICATION_CSS}
      DESTINATION ${CMAKE_INSTALL_PREFIX})
  install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/viewer.js.mem
      ${CMAKE_CURRENT_BINARY_DIR}/viewer.wasm
      ${CMAKE_CURRENT_BINARY_DIR}/viewer.data
      DESTINATION ${CMAKE_INSTALL_PREFIX} OPTIONAL)
endif()
