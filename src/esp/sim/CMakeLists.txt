add_library(sim STATIC
  SimulatorWithAgents.cpp
  SimulatorWithAgents.h
)

target_link_libraries(sim
  PUBLIC
    nav
)
