add_library(sim STATIC
  Simulator.cpp
  Simulator.h
  SimulatorWithAgents.cpp
  SimulatorWithAgents.h
)

target_link_libraries(sim
  PUBLIC
    nav
)
