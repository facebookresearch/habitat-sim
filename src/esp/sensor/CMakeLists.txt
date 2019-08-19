add_library(sensor STATIC
  PinholeCamera.cpp
  PinholeCamera.h
  Sensor.cpp
  Sensor.h
)

target_link_libraries(sensor
  PUBLIC
    core
    gfx
    scene
)
