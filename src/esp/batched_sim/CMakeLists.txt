set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

add_library(
  batched_sim STATIC
  BatchedSimulator.cpp
  BatchedSimulator.h
  BpsSceneMapping.cpp
  BpsSceneMapping.h
  GlmUtils.cpp
  GlmUtils.h
)

target_link_libraries(
  batched_sim
  PUBLIC core
         sim
         bps3D
         bps3D_core
         bps3D_vulkan
         stb
)
