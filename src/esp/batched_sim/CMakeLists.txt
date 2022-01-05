set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

add_library(
  batched_sim STATIC
  BatchedSimAssert.h
  BatchedSimulator.cpp
  BatchedSimulator.h
  BpsSceneMapping.cpp
  BpsSceneMapping.h
  ColumnGrid.cpp
  ColumnGrid.h
  CollisionBroadphaseGrid.cpp
  CollisionBroadphaseGrid.h
  ColumnGridBuilder.cpp
  ColumnGridBuilder.h
  EpisodeSet.cpp
  EpisodeSet.h
  GlmUtils.cpp
  GlmUtils.h
  PlacementHelper.cpp
  PlacementHelper.h
  ProfilingScope.cpp
  ProfilingScope.h
  SerializeCollection.cpp
  SerializeCollection.h
)

target_link_libraries(
  batched_sim
  PUBLIC core
         sim
         io
         bps3D
         bps3D_core
         bps3D_vulkan
         stb
)
