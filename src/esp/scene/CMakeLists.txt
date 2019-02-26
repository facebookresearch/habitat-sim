file(GLOB_RECURSE scene_SOURCES "*.cpp")

add_library(scene STATIC ${scene_SOURCES})

target_link_libraries(scene
  PUBLIC
    assets
    core
    geo
    gfx
    io
)
