file(GLOB_RECURSE agent_SOURCES "*.cpp")

add_library(agent STATIC ${agent_SOURCES})

target_link_libraries(agent
  PUBLIC
    core
    gfx
    sensor
)
