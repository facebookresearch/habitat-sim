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
