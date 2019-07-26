file(GLOB_RECURSE sensor_SOURCES "*.cpp")

add_library(sensor STATIC ${sensor_SOURCES})

target_link_libraries(sensor
  PUBLIC
    core
    gfx
    scene
)
