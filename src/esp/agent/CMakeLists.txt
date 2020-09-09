add_library(
  agent STATIC
  Agent.cpp Agent.h
)

target_link_libraries(
  agent
  PUBLIC core gfx sensor
)
