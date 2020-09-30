add_library(
  sim STATIC
  Simulator.cpp Simulator.h SimulatorConfiguration.cpp SimulatorConfiguration.h
)

target_link_libraries(
  sim
  PUBLIC nav
)
