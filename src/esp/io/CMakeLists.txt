file(GLOB_RECURSE io_SOURCES "*.cpp")

add_library(io STATIC ${io_SOURCES})

target_link_libraries(io
  PUBLIC
    core
)
