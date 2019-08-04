file(GLOB_RECURSE sim_SOURCES "*.cpp")

add_library(sim STATIC ${sim_SOURCES})

target_link_libraries(sim
  PUBLIC
    nav
)
