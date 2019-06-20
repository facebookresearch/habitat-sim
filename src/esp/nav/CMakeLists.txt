add_library(nav STATIC
  GreedyFollower.cpp
  GreedyFollower.h
  PathFinder.cpp
  PathFinder.h
)

target_include_directories(nav
  PRIVATE
    "${DEPS_DIR}/recastnavigation/Detour/Include"
    "${DEPS_DIR}/recastnavigation/Recast/Include"
)

if(MSVC)
  # Define common math constants if we compile with MSVC.  This is needed because of Sophus.
  target_compile_definitions(nav PUBLIC _USE_MATH_DEFINES)
endif (MSVC)

target_link_libraries(nav
  PUBLIC
    core
    agent
    scene
  PRIVATE
    Detour
    Recast
)
