file(GLOB_RECURSE nav_SOURCES "*.cpp")

add_library(nav STATIC ${nav_SOURCES})

target_include_directories(nav
  PRIVATE
    "${DEPS_DIR}/recastnavigation/Detour/Include"
    "${DEPS_DIR}/recastnavigation/Recast/Include"
)

target_link_libraries(nav
  PUBLIC
    core
    agent
    scene
  PRIVATE
    Detour
    Recast
)
