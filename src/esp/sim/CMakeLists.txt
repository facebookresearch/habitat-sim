add_library(
  sim STATIC
  ReplayBatchRenderer.cpp
  ReplayBatchRenderer.h
  Simulator.cpp
  Simulator.h
  SimulatorConfiguration.cpp
  SimulatorConfiguration.h
)

target_link_libraries(
  sim
  PUBLIC nav
)
