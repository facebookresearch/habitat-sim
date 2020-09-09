add_library(
  sim STATIC
  Simulator.cpp Simulator.h
)

target_link_libraries(
  sim
  PUBLIC nav
)
