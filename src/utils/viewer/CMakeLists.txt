find_package(Magnum REQUIRED DebugTools Text)
find_package(MagnumPlugins REQUIRED StbTrueTypeFont)

set(viewer_SOURCES viewer.cpp ObjectPickingHelper.cpp ObjectPickingHelper.h)

corrade_add_resource(viewer_RESOURCES resources.conf)

#set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
add_executable(viewer ${viewer_SOURCES} ${viewer_RESOURCES})

target_link_libraries(
  viewer
  PRIVATE assets
          gfx
          nav
          scene
          sim
          geo
          Magnum::DebugTools
          Magnum::Text
          MagnumPlugins::StbTrueTypeFont
)

# copy JS/HTML/CSS scaffolding for WebGL build
if(CORRADE_TARGET_EMSCRIPTEN)
  install(FILES viewer.html ${MAGNUM_EMSCRIPTENAPPLICATION_JS}
                ${MAGNUM_WEBAPPLICATION_CSS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()
