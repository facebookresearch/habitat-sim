file(GLOB_RECURSE scene_SOURCES "*.cpp")

find_library(MagnumIntegration REQUIRED Eigen)

add_library(scene STATIC ${scene_SOURCES})

target_link_libraries(scene
  PRIVATE
    MagnumIntegration::Eigen
  PUBLIC
    assets
    core
    geo
    gfx
    io
)
