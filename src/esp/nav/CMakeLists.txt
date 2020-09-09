add_library(
  nav STATIC
  GreedyFollower.cpp GreedyFollower.h PathFinder.cpp PathFinder.h
)

target_include_directories(
  nav PRIVATE "${DEPS_DIR}/recastnavigation/Detour/Include"
              "${DEPS_DIR}/recastnavigation/Recast/Include"
)

target_link_libraries(
  nav
  PUBLIC core agent scene
  PRIVATE Detour Recast
)

if(BUILD_TEST)
  add_subdirectory(test)
endif()
