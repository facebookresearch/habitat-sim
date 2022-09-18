set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

set(
  batched_sim_SRCS
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
  EpisodeGenerator.cpp
  EpisodeGenerator.h
  EpisodeSet.cpp
  EpisodeSet.h
  GlmUtils.cpp
  GlmUtils.h
  PlacementHelper.cpp
  PlacementHelper.h
  ProfilingScope.cpp
  ProfilingScope.h
  PythonBatchEnvironmentState.cpp
  PythonBatchEnvironmentState.h
  SerializeCollection.cpp
  SerializeCollection.h
)

pybind11_add_module(batched_sim STATIC ${batched_sim_SRCS})

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

add_library(
  batched_sim_no_pybind STATIC
  ${batched_sim_SRCS}
)
target_compile_definitions(batched_sim_no_pybind PUBLIC DISABLE_BATCHED_SIM_PYBIND=1)

target_link_libraries(
  batched_sim_no_pybind
  PUBLIC core
         sim
         io
         bps3D
         bps3D_core
         bps3D_vulkan
         stb
)
